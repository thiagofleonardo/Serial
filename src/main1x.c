/*
A primeira thread le o que o usuario digitar no teclado e armazena isso em uma fifo, que já é lida pela funcao 
enviando_dados e escrita no pino PTB0. OBS: se o usuario decidir digitar uma segunda mensagem enquanto a outra
estiver sendo transmitida, esta sera colocada na fila e depois transmitida. 

Outra thread ficará lendo para sempre o pino PTB1. Quando ela achar o sync (0x16) ela entra em um conjunto de 
açoes para ler o resto do pacote. Quando ela achar o etx (0x03) ela reinicia o processo.

Rode o codigo, digite o id (se for valido vai printar: "Achei o ID") depois digite a mensagem de no maximo 7
caracteres, se tudo ocorrer normalmente vai achar o sync, pular uma linha, printar o stx, pular uma linha, 
printar o id, pular uma linha... ate printar o etx. Depois disso, o processo reinicia a busca pelo sync.

Uma observação é que se o pino PTB1 estiver logicalmente alto quando o programa comecar a rodar, ele nao vai
identificar o pacote e vai passar batido.
*/

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
K_FIFO_DEFINE(recebe_dados);
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
struct pacote_dados* envio;
struct pacote_dados* recebido;

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

void bits(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        if ((byte & 0x80) == 0x80) {
            gpio_pin_set(stx, 0x0, 1);
        } else {
            gpio_pin_set(stx, 0x0, 0);
        }
        k_msleep(40);
        byte <<= 1;
    }
}

void enviando_dados() {
    gpio_pin_configure(stx, 0x0, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set(stx, 0x0, 1);
    
    while ((envio = k_fifo_get(&envia_dados, K_NO_WAIT)) != NULL) {
        bits(envio->sync);
        bits(envio->stx);
        bits(envio->id_tamanho);
        
        for (int n = 0; n < sizeof(envio->mensagem); n++) {
            int msg = envio->mensagem[n];
            if (msg == '\0') {
                break; // Pare ao encontrar o caractere nulo
            }
            bits(msg);
        }
        bits(envio->etx);
    }
}


void lendo_freq_igual(void){
    gpio_pin_configure(stx, 0x1, GPIO_INPUT);
    int armazena8 = 0, somadorl = 0, contador7 = 0;

    while(1){
        armazena8 = armazena8 & 0xFF;
        armazena8 <<= 1;
        if(gpio_pin_get(stx, 0x1) == 1){
            armazena8 = armazena8 | 0x1;
            printk("1");
        }else{
            armazena8 = armazena8 | 0x0;
            printk("0");
        }
		
        if(armazena8 == 0x03 && somadorl > 3){
            printk("\n");
            somadorl = 0;
			contador7 = 0;
        }
        if(armazena8 == 0x16 && somadorl == 0){
            printk("\n");
            somadorl++;
        }else if(armazena8 == 0x02 && somadorl == 1){
            printk("\n");
            somadorl++;
        }else if(somadorl >= 2){	
			contador7++;
		}
        if(contador7 == 8){
            printk("\n");
            contador7 = 0;
            somadorl++;
        }
        k_msleep(40);
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
        //printk("msg.tamanho: %d - ", msg_len);
        for (int i = 0; i < msg_len; i++) {
            pacote.mensagem[i] = buf[i]; //armazena a mensagem
        }
        cont2++;
    }

    if (cont1 == 1 && cont2 == 1) {
        k_fifo_put(&envia_dados, &pacote); //adicona o pacote na fila
        //printk("FIFO armazenada \n");

		enviando_dados();

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
K_THREAD_DEFINE(lendo_dados_id, MY_STACK_SIZE, lendo_freq_igual, NULL, NULL, NULL, MY_PRIORITY, 0, 0);