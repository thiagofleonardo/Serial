/*
Código funcional para TX & RX sem CSMA.

Funcionamento: depois de dar flash no código, abra a serial, digite o ID (5 bits) e o tamanho da mensagem (3 bits)
tudo junto e pressione enter, se estiver correto aparecerá "Peguei o ID". Depois disso, digite a mensagem de no 
máximo 7 caracteres e pressione enter. Isso já basta para a transmissão, se a recepção estiver ligada, aparecerá
na serial nessa ordem: "Achei o sync", "Achei o stx", "Achei o ID: %d", "Mensagem[i]: %d", "Achei o etx". E no final,
após a transmissão ter acabado, vai printar o que foi armazenado: "ID: %d", "Mensagem: %c-%c-%c-%c-%c-%c-%c" (dependendo
do tamanho da mensagem).

A primeira thread declarada lê a entrada do usuario por um callback e armazena em uma fila de mensagem, essa fila
é lida e a mensagem é armazenada por um pacote em uma fifo chamada envia_dados. Para enviar uma mensgaem deve-se 
digitar primeiro o ID e depois a mensagem, a mensagem deve ter no máximo 7 caracteres. Serão lidos até 8 bits 
digitados (considerando que o ID tem 8), se for digitado masi do que isso, será ignorado.

A segunda thread é de escrita no pino PTB0. Ela pega o pacote na fifo armazena_dados e transmite cada bit a cada
40ms (Escrita). Utiliza a funcao bits para fazer isso.

A terceira thread é de leitura no pino PTB1. Ela lê o pino PTB1 a cada 10ms (Leitura), armazena o bit lido em uma
variável de 32 bits e depois armazena essa variável em uma fifo chamada recebe_dados. Ela nao faz a transformação 
de 32 para 8 bits, pois isso custaria um tempo e poderia dessincronizar a leitura com a transmissão.

A quarta thread é de processamento da leitura. Ela pega o pacote na fifo recebe_dados e processa a leitura. O 
primeiro passo é transformar os 32 bits lidos em 8 bits pela função bit32_para_bit8. Depois ela verifica se é o 
SYNC, se for ela começa um contador para ler o pacote. Logo em seguida, já procura o STX; achado esses dois ela 
guarda os proximos bytes até encontrador o etx, sabendo que o primeiro é o ID e os proximos são os caracteres da
mensagem, limitados a 7. Depois que achar o etx, ela printa na serial o ID e a mensagem.
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 7 //prioridade da thread

#define Leitura 10
#define Escrita 40

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define MSG_SIZE 9 /* fila para armazenar até 10 mensagens (alinhada ao limite de 4 bytes) */

const struct device* stx = DEVICE_DT_GET(DT_NODELABEL(gpiob));

K_FIFO_DEFINE(envia_dados);
K_FIFO_DEFINE(recebe_dados);

int cont1 = 0, cont2 = 0;
uint8_t SYNC = 0x16, STX = 0x02, ETX = 0x03;


struct pacote_dados {
    void *fifo_reserved;  // Primeiro membro necessário para usar k_fifo
    uint8_t id_tamanho;
    char mensagem[8];
};
struct recebido {
    void *fifo_reserved;
    uint32_t armazena32;
};

/* fila para armazenar até 10 mensagens (alinhada ao limite de 4 bytes) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* buffer de recepção usado no callback da ISR UART */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

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

void leitura_PTB1(void){
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
        k_msleep(Leitura);
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

/Escrita de bits no pino (TX)/
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

void escrita_PTB0() {
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
K_THREAD_DEFINE(escrita_PTB0_id, MY_STACK_SIZE, escrita_PTB0, NULL, NULL, NULL, MY_PRIORITY, 0, 0);
K_THREAD_DEFINE(leitura_PTB0_id, MY_STACK_SIZE, leitura_PTB1, NULL, NULL, NULL, MY_PRIORITY, 0, 0);
K_THREAD_DEFINE(processa_leitura_id, MY_STACK_SIZE, processa_leitura, NULL, NULL, NULL, MY_PRIORITY, 0, 0);