#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 7 //prioridade da thread

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define MSG_SIZE 9 /* fila para armazenar até 10 mensagens (alinhada ao limite de 4 bytes) */

const struct device* stx = DEVICE_DT_GET(DT_NODELABEL(gpiob));

K_FIFO_DEFINE(envia_dados);
int cont1 = 0, cont2 = 0;

struct pacote_dados {
    void *fifo_reserved;  // Primeiro membro necessário para usar k_fifo
    char sync;
    char stx;
    uint8_t id_tamanho;
    char mensagem[8];
    char etx;
};
static struct pacote_dados pacote = {
        .sync = 0x16,
        .stx = 0x02,
        .etx = 0x03
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

void enviando_dados() {
    struct pacote_dados* envio;
    envio = k_fifo_get(&envia_dados, K_FOREVER);

    printk("ID do pacote: ");
    int id = envio->id_tamanho;
    for (int j = 0; j < 8; j++) {
        if ((id & 0x80) == 0x80) {
            printk("1");
            gpio_pin_set(stx, 0x03, 1);
        } else {
            printk("0");
            gpio_pin_set(stx, 0x03, 0);
        }
        id <<= 1;
    }
    printk("\n");

    for (int n = 0; n < sizeof(envio->mensagem); n++) {
        int msg = envio->mensagem[n];
        if (msg == '\0') {
            break; // Pare ao encontrar o caractere nulo
        }
        printk("Mensagem da FIFO: %c\n", msg);

        printk("No pino: ");
        for (int i = 0; i < 8; i++) {
            if ((msg & 0x80) == 0x80) {
                printk("1");
                gpio_pin_set(stx, 0x03, 1);
                
            } else {
                printk("0");
                gpio_pin_set(stx, 0x03, 0);
                
            }
            msg <<= 1;
        }
        printk("\n");
    }
}

void chamador(void){
    gpio_pin_configure(stx, 0x03, GPIO_OUTPUT_ACTIVE);
    while(1){
        enviando_dados();
    }
}
void armazenar(char *buf) {
    int msg_len = strlen(buf);

    if (msg_len == 8) { // verifica se é o id
        printk("Achei o ID\n");
        uint8_t result = 0;
        for (int i = 0; i < msg_len; i++) {
            // Desloca o resultado à esquerda para dar espaço ao próximo bit
            result <<= 1;
            if (buf[i] == '1') {
                // Adiciona 1 no bit menos significativo
                result |= 1;
            }
        }
        pacote.id_tamanho = result;
        cont1++;

    } else { //senao for o id é a mensagem
        printk("msg.tamanho: %d - ", msg_len);
        for (int i = 0; i < msg_len; i++) {
            pacote.mensagem[i] = buf[i]; //armazena a mensagem
        }
        cont2++;
    }

    if (cont1 == 1 && cont2 == 1) {
        k_fifo_put(&envia_dados, &pacote); //adicona o pacote na fila
        printk("FIFO armazenada \n");

        pacote.id_tamanho = 0;
        memset(pacote.mensagem, 0, sizeof(pacote.mensagem));
        cont1 = 0;
        cont2 = 0;
    } else if (cont1 > 1 || cont2 > 1) {
        cont1 = 0;
        cont2 = 0;
		printk("2 id's\n");
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

/* Funçao chamador vai ser um while true que sempre vai chamar a funcao envia dados, essa tem um 
k_fifo_get com k_forever. Quando tiver um dado na FIFO ele vai tirar e enviar par ao transmissor */
K_THREAD_DEFINE(chamador_id, MY_STACK_SIZE, chamador, NULL, NULL, NULL, MY_PRIORITY, 0, 0);