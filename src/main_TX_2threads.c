/*
O codigo abaixo usa o TX com 2 threads, um para leitura do teclado e put na fifo e outra para get na fifo
e transmissao no pino. O codigo esa funcioando perfeitamente, com uma terceira thread para leitura do pino. 
A frequencia esta igual, mas pode ser alterada mudando o tempo de leitura no define.

Ter um codigo dessa maneira (com 2 threads) é essencial para a aplicaçao do CSMA, pois agora é possivel travar
facilmente a thread de escrita no pino, evitando a sobreposiçao de pacotes.

MANDAR 19/12
*/
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 7 //prioridade da thread

#define Leitura 40
#define Escrita 40

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define MSG_SIZE 9 /* fila para armazenar até 10 mensagens (alinhada ao limite de 4 bytes) */

const struct device* stx = DEVICE_DT_GET(DT_NODELABEL(gpiob));

K_FIFO_DEFINE(envia_dados);

int cont1 = 0, cont2 = 0;
uint8_t SYNC = 0x16, STX = 0x02, ETX = 0x03;


struct pacote_dados {
    void *fifo_reserved;  // Primeiro membro necessário para usar k_fifo
    uint8_t id_tamanho;
    char mensagem[8];
};

/* fila para armazenar até 10 mensagens (alinhada ao limite de 4 bytes) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* buffer de recepção usado no callback da ISR UART */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/* Lê caracteres da UART até que o final da linha seja detectado. Depois, envia os dados para a fila de mensagens. */
void serial_cb(const struct device *dev, void *user_data) {
    uint8_t c;

    if (!uart_irq_update(uart_dev)) {
        return;
    }

    if (!uart_irq_rx_ready(uart_dev)) {
        return;
    }

    /* lê até que a FIFO esteja vazia */
    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
            /* termina a string */
            rx_buf[rx_buf_pos] = '\0';

            /* se a fila estiver cheia, a mensagem é descartada silenciosamente */
            k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

            /* reseta o buffer (ele foi copiado para a msgq) */
            rx_buf_pos = 0;
        } else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
            rx_buf[rx_buf_pos++] = c;
        }
        /* caso contrário: caracteres além do tamanho do buffer são descartados */
    }
}

void leitura(void){
    gpio_pin_configure(stx, 0x1, GPIO_INPUT);
    int value;

    while(1){
        value = gpio_pin_get(stx, 0x1);
        printk("%d", value);
        k_msleep(Leitura);
    }
}

void bits(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        if ((byte & 0x80) == 0x80) {
            gpio_pin_set(stx, 0x0, 1);
        } else {
            gpio_pin_set(stx, 0x0, 0);
        }
        k_msleep(Escrita);
        byte <<= 1;
    }
}

struct pacote_dados pacote;
void armazenar(char *buf) {
    int msg_len = strlen(buf);

    if (msg_len == 8 && cont1 == 0) { // verifica se é o id
        printk("Peguei o ID\n");
        uint8_t result = 0;
        for (int i = 0; i < msg_len; i++) {
            result <<= 1; // Desloca o resultado à esquerda para dar espaço ao próximo bit
            if (buf[i] == '1') {
                result |= 1; // Adiciona 1 no bit menos significativo
            }
        }
        pacote.id_tamanho = result;
        cont1++;

    } else if (msg_len < 8 && cont1 == 1) { //senao for o id é a mensagem
        for (int i = 0; i < msg_len; i++) {
            pacote.mensagem[i] = buf[i]; //armazena a mensagem
        }
        cont2++;
    } else if (msg_len >= 8 && cont1 > 0){
        cont1++;
    } else if (msg_len < 8){
        printk("Digite um ID valido!\n");
    }

    if (cont1 == 1 && cont2 == 1) {
        k_fifo_put(&envia_dados, &pacote); //adicona o pacote na fila
        cont1 = 0;
        cont2 = 0;
    } else if (cont1 > 1) {
        cont1 = 0;
        cont2 = 0;
		printk("Mensagem invalida\n");
        printk("Digite novamente o ID!\n");
    }
}

void enviando_dados() {
    gpio_pin_configure(stx, 0x0, GPIO_OUTPUT_ACTIVE);
    
    while (1) {
        struct pacote_dados* envio = k_fifo_get(&envia_dados, K_FOREVER);
        if(!envio) continue;

        bits(SYNC);
        bits(STX);
        bits(envio->id_tamanho);
        for (int n = 0; n < sizeof(envio->mensagem); n++) {
            int msg = envio->mensagem[n];
            if (msg == '\0') {
                break; // Pare ao encontrar o caractere nulo
            }
            bits(msg);
        }
        bits(ETX);
    }
}

void pega_dados(void) {
    char tx_buf[MSG_SIZE];

    if (!device_is_ready(uart_dev)) {
        printk("Dispositivo UART não encontrado!");
        return;
    }

    /* configura a interrupção e o callback para receber dados */
    int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

    if (ret < 0) {
        if (ret == -ENOTSUP) {
            printk("Suporte à API UART acionada por interrupção não habilitado\n");
        } else if (ret == -ENOSYS) {
            printk("O dispositivo UART não suporta a API acionada por interrupção\n");
        } else {
            printk("Erro ao definir o callback da UART: %d\n", ret);
        }
        return;
    }
    uart_irq_rx_enable(uart_dev);

    /* espera indefinidamente por entrada do usuário */
    while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {
		armazenar(tx_buf);
    }
}


/* Primeira thread le o que voce digita no teclado ate 7 bits (id) monta o pacote e trasnmite para a FIFO */
K_THREAD_DEFINE(pega_dados_id, MY_STACK_SIZE, pega_dados, NULL, NULL, NULL, MY_PRIORITY, 0, 0);
K_THREAD_DEFINE(leitura_id, MY_STACK_SIZE, leitura, NULL, NULL, NULL, MY_PRIORITY, 0, 0);
K_THREAD_DEFINE(enviando_dados_id, MY_STACK_SIZE, enviando_dados, NULL, NULL, NULL, MY_PRIORITY, 0, 0);