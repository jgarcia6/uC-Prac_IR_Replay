/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "myUart.h"

#include "driver/ledc.h"
#include "freertos/queue.h"
#include "driver/timer.h"

#include "soc/gpio_reg.h"

// UART 0 used for PC communication
#define PC_UART_PORT        0
#define PC_UART_RX_PIN      3
#define PC_UART_TX_PIN      1
#define PC_UARTS_BAUD_RATE  (115200)


#define UART_BUF_SIZE       (1024)
#define IR_FREQ             38000
#define IR_TX_TX_PIN        15
#define IR_RX_RX_PIN        16


#define BUFFER_SIZE (1<<11) //2048 //2K 

#define MOD(n)                      ( (n) & (BUFFER_SIZE-1))
#define IS_BUFFER_EMPTY(buff)       (buff.in_idx == buff.out_idx)
#define IS_BUFFER_FULL(buff)        (MOD(buff.in_idx + 1) == buff.out_idx)
#define BUFFER_INSERT(buff, value)  buff.buffer[buff.in_idx] = value; buff.in_idx = MOD(buff.in_idx + 1);

typedef struct{
    uint16_t buffer[BUFFER_SIZE];
    uint16_t in_idx;
    uint16_t out_idx;
}sBufferCircular_t;

sBufferCircular_t sTimingBuffer;

static bool startFlag = false;
static bool endFlag = false;

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (IR_TX_TX_PIN) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               ((((1 << 8) - 1) * 2) / 3) // Set duty to 66%. 
#define LEDC_FREQUENCY          (38000) // Frequency in Hertz. Set frequency at 38 kHz

static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 38 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}


/* Timer interrupt service routine */
static void IRAM_ATTR timer0_ISR(void *ptr)
{
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_group_set_counter_enable_in_isr(TIMER_GROUP_0, TIMER_0, TIMER_PAUSE);

    gpio_isr_handler_remove(IR_RX_RX_PIN);
    endFlag = true;
}

#define TIMEOUT_TICKS           15000           // 15 ms timeout
#define TIMER_DIVIDER          (80)             //  1us divider

/* Timer group0 TIMER_0 initialization */
static void timer0_init(void)
{
    esp_err_t ret;
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .intr_type = TIMER_INTR_LEVEL,
        .auto_reload = 0,
    };

    ret = timer_init(TIMER_GROUP_0, TIMER_0, &config);
    ESP_ERROR_CHECK(ret);
    ret = timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    ESP_ERROR_CHECK(ret);
    ret = timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMEOUT_TICKS);
    ESP_ERROR_CHECK(ret);
    ret = timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    ESP_ERROR_CHECK(ret);
    /* Register an ISR handler */
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer0_ISR, NULL, 0, NULL);
}


static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    
    if (gpio_num != IR_RX_RX_PIN)
        return;

    uint8_t gpioState = (GPIO_IN_REG >> IR_RX_RX_PIN) & 1;

    if (startFlag == true && gpioState == 0)
    {
        startFlag = false;
        //set_alarm_in_isr 
        timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
        //enable_timer_in_isr
        timer_group_set_counter_enable_in_isr(TIMER_GROUP_0, TIMER_0, TIMER_START);
    }
    else if (startFlag == false)
    {
        BUFFER_INSERT(sTimingBuffer, timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_0))
        timer_set_counter_value_in_isr(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
        //timer_group_set_counter_enable_in_isr(TIMER_GROUP_0, TIMER_0, TIMER_START);
    }
}

// timer interrupt, disable GPIO interrupt, disable timer

void startSampling(void)
{
    startFlag = true;
    sTimingBuffer.in_idx = 0;
    sTimingBuffer.out_idx = 0;
    timer0_init();
    gpio_isr_handler_add(IR_RX_RX_PIN, gpio_isr_handler, (void*) IR_RX_RX_PIN);
}



void uartInit(uart_port_t uart_num, uint32_t baudrate, uint8_t size, uint8_t parity, uint8_t stop, uint8_t txPin, uint8_t rxPin)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = size-5,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, txPin, rxPin,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

}

void delayMs(uint16_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void uartClrScr(uart_port_t uart_num)
{
    // Uso "const" para sugerir que el contenido del arreglo lo coloque en Flash y no en RAM
    const char caClearScr[] = "\e[2J";
    uart_write_bytes(uart_num, caClearScr, sizeof(caClearScr));
}
void uartGoto11(uart_port_t uart_num)
{
    // Limpie un poco el arreglo de caracteres, los siguientes tres son equivalentes:
     // "\e[1;1H" == "\x1B[1;1H" == {27,'[','1',';','1','H'}
    const char caGoto11[] = "\e[1;1H";
    uart_write_bytes(uart_num, caGoto11, sizeof(caGoto11));
}

bool uartKbhit(uart_port_t uart_num)
{
    uint8_t length;
    uart_get_buffered_data_len(uart_num, (size_t*)&length);
    return (length > 0);
}

void uartPutchar(uart_port_t uart_num, char c)
{
    uart_write_bytes(uart_num, &c, sizeof(c));
}

void uartPuts(uart_port_t uart_num, char *str)
{
    while(*str)
    {
        uartPutchar(uart_num, *str++);
    }
}

char uartGetchar(uart_port_t uart_num)
{
    char c;
    // Wait for a received byte
    while(!uartKbhit(uart_num))
    {
        delayMs(10);
    }
    // read byte, no wait
    uart_read_bytes(uart_num, &c, sizeof(c), 0);

    return c;
}

void app_main(void)
{
    char str[32];
    uartInit(PC_UART_PORT, PC_UARTS_BAUD_RATE, 8, 0, 1, PC_UART_TX_PIN, PC_UART_RX_PIN);
    delayMs(500);
    uartGoto11(PC_UART_PORT);
    delayMs(500);
    uartClrScr(PC_UART_PORT);
    // Init 38KHz LED modulation control
    //ledc_init();
    // Init Timer and ISR
    //timer0_init();
    //timer_start(TIMER_GROUP_0, TIMER_0);
    
    while(1)
    {
        uartGetchar(PC_UART_PORT);
        startSampling();
        while(!endFlag)
        {
            delayMs(1);
        }

        sprintf(str,"state = 0, duration = %d",sTimingBuffer.buffer[sTimingBuffer.out_idx]);
        sTimingBuffer.out_idx = MOD(sTimingBuffer.out_idx + 1);
        uartPuts(PC_UART_PORT, str);
        sprintf(str,"state = 1, duration = %d",sTimingBuffer.buffer[sTimingBuffer.out_idx]);
        sTimingBuffer.out_idx = MOD(sTimingBuffer.out_idx + 1);
        uartPuts(PC_UART_PORT, str);
    } 
}
