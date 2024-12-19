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
struct recebido {
    void *fifo_reserved;
    uint32_t armazena32;
};
static struct pacote_dados pacote = {
        .sync = 0x16,
        .stx = 0x02,
        .etx = 0x03
    };
struct pacote_dados* envio;
struct pacote_dados* recebido;
struct recebido leitura_pino = {
	.armazena32 = 0
};
struct recebido pacote_recebido = {
	.armazena32 = 0
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

uint8_t bit32_para_bit8(uint32_t entrada){
    uint8_t resultado = 0;

    for (int i = 0; i < 8; i++) {
        // Isolando 4 bits, começando do bit mais baixo
        uint8_t bloco = (entrada >> (i * 4)) & 0xF;

        // Extraindo os dois bits centrais
		uint8_t bit0 = bloco & 0x1;
        uint8_t bit1 = (bloco >> 1) & 0x1;
        uint8_t bit2 = (bloco >> 2) & 0x1;

        // Verificando se os dois bits centrais são iguais
        if (bit1 != bit2) {
            bit1 = bit0;  // Retorna -1 se houver um erro
        }

        // Calculando o valor do bit para o resultado
        resultado |= bit1 << i;
    }

    return resultado;
}

void lendo_freq_4x(void){
    gpio_pin_configure(stx, 0x1, GPIO_INPUT);
    int value = 0;
    uint32_t bits32 = 0;
    struct recebido novo_pacote;

    while(1){
        value = gpio_pin_get(stx, 0x1);
        //printk("%d", value);
        bits32 <<= 1;
        bits32 |= value;
		
        novo_pacote.armazena32 = bits32;
        k_fifo_put(&recebe_dados, &novo_pacote);
        k_msleep(10);
    }
}

void processa_leitura(void){
    int somadorl = 0, contador7 = 0;
    uint32_t entrada = 0;
    uint8_t resultado = 0, id_lido = 0, memoria = 0;
    char mensagem[8];

    while(1){
        struct recebido* processamento_analise = k_fifo_get(&recebe_dados, K_FOREVER);
        if(!processamento_analise) continue;

        entrada = processamento_analise->armazena32;
        resultado = bit32_para_bit8(entrada);
        
        /*if(somadorl > 2){
		    printk("-%d", resultado);
        }*/
        if(resultado == 0x03 && somadorl > 2){
            printk("\nAchei o etx\n");
            somadorl = 0;
            printk("ID: %d\n", id_lido);
            printk("Mensagem: ");
            for(int i = 0; i < (id_lido & 0x07); i++){
                printk("%c-", mensagem[i]);
                mensagem[i] = NULL;
            }
            printk("\n");
        }

        //contabits++;

        if (resultado == 0x16 && somadorl == 0) {
            printk("\nAchei o sync\n");
            somadorl++;
            //contabits = 0;
        } else if (resultado == 0x02 && somadorl == 1) {
            printk("Achei o stx\n");
            somadorl++;
            contador7 = 0;
        } else if (somadorl == 2 && contador7 == 31) {
            printk("Achei o ID: %d\n", resultado);
            contador7 = 0;
            somadorl++;
            id_lido = resultado;
        } else if(somadorl >= 2){
            contador7++;
        }
        if (somadorl > 2 && contador7 == 32 && (somadorl < (3+(id_lido & 0x7)))){
            contador7 = 0;
            if(memoria == (resultado-1) || memoria == (resultado+1) || resultado < 33 || resultado > 125){
                mensagem[somadorl - 3] = memoria;
                printk("Mensagem[%d]: %d", (somadorl-3), memoria);
            }else{
                mensagem[somadorl - 3] = resultado;
                printk("Mensagem[%d]: %d", (somadorl-3), resultado);
            }
            somadorl++;
            printk("\n");
        }

        /*if (contabits > 31) {
            somadorl = 0;
        }*/
       memoria = resultado;
    }
}

void armazenar(char *buf) {
    int msg_len = strlen(buf);

    if (msg_len == 8 && cont1 == 0) { // verifica se é o id
        printk("Peguei o ID\n");
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

    } else if (msg_len < 8 && cont1 == 1) { //senao for o id é a mensagem
        //printk("msg.tamanho: %d - ", msg_len);
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
        //printk("FIFO armazenada \n");

		enviando_dados();

        pacote.id_tamanho = 0;
        memset(pacote.mensagem, 0, sizeof(pacote.mensagem));
        cont1 = 0;
        cont2 = 0;
    } else if (cont1 > 1) {
        cont1 = 0;
        cont2 = 0;
		printk("Mensagem invalida\n");
        printk("Digite novamente o ID!\n");
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
K_THREAD_DEFINE(lendo_freq_4x_id, MY_STACK_SIZE, lendo_freq_4x, NULL, NULL, NULL, MY_PRIORITY, 0, 0);
K_THREAD_DEFINE(processa_leitura_id, MY_STACK_SIZE, processa_leitura, NULL, NULL, NULL, MY_PRIORITY, 0, 0);