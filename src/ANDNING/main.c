#include "gd32vf103.h"
#include "drivers.h"
#include "lcd.h"
#include "adc.h"
#include "dac.h"
#include "delay.h"
#include <stdio.h>
#include "pwm.h"
#include "usart.h"  // Antag att detta är en anpassad header för UART-funktioner
#include <math.h>

#define EI 1
#define DI 0

#define SCREEN_WIDTH 160
#define SCREEN_HEIGHT 128
#define ZERO_COUNT_LIMIT 100
#define ANDNING 100
#define TIMER_LIMIT 4000

#define I2C_DEVICE_ADDR 0x5c
#define I2C_READ_REG 0x75



typedef struct {
    float temperature;
} am2320sensor_t;

am2320sensor_t get_temperature(uint32_t port);

void showTemperature(am2320sensor_t sensor);

void shift_array_right_and_draw(int array[]); 

int read_adc_potentiometer(void);

void updateScreen(int *screen_row, int adcr);

int i2c_get_ack(int32_t port, uint8_t addr, uint32_t timeout);
void i2c_read_register(int32_t port, uint8_t addr, uint8_t reg, uint8_t size, uint8_t *data);
void i2c_write_register(int32_t port, uint8_t addr, uint8_t reg, uint8_t size, uint8_t *data);

void timer(int *time, int speed2);

void usart_config();





int main(void) {
    int ms = 0, time=0, speed2 = 0; 
    int adcr = 0;
    int screen_row[SCREEN_WIDTH] = {0};
    int last_adc_value = -1;
    int stable_adc_count = 0;
    int has_reached_limit = 0; 

    am2320sensor_t sensor = {0};

    t5omsi();
    colinit();
    l88init();
    keyinit();
    ADC3powerUpInit(1);
    Lcd_SetType(LCD_NORMAL);
    Lcd_Init();
    LCD_Clear(BLACK);
    T1powerUpInitPWM(0x1);
    DAC0powerUpInit();   
    u0init(EI);

    eclic_global_interrupt_enable();        // !!! INTERRUPT ENABLED !!!


    rcu_periph_clock_enable(RCU_I2C0);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);

    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
    i2c_enable(I2C0);
    

while (1) {
        if (t5expq()) {  // Antagen funktion som kontrollerar något tillstånd
            ms++;

            if (ms % 1000 == 0) {
                sensor = get_temperature(I2C0);
                showTemperature(sensor);  
            }

            int new_adcr = read_adc_potentiometer();
            int difference = new_adcr - last_adc_value;

            if (difference <= 1 && difference >= -1) {
                stable_adc_count++;  // Behåll räknaren om inom tolerans
            } else {
                stable_adc_count = 0;  // Återställ räknaren om skillnaden är för stor
                has_reached_limit = 0;  // Återställ flaggan om skillnaden är utanför toleransen
            }

            last_adc_value = new_adcr;

            if (stable_adc_count >= TIMER_LIMIT) {
                T1setPWMmotorB(100);
                usart_data_transmit(USART1, 1);
                timer(&time, speed2);
            } else if (!has_reached_limit) {
                T1setPWMmotorB(0);
            }

            // Uppdatera skärmen periodiskt
            if (ms % 50 == 0) {
                updateScreen(screen_row, new_adcr);
            }
        }
    }
}

am2320sensor_t get_temperature(uint32_t port) {
    am2320sensor_t ret;
    uint8_t addr = 0x5C;
    uint8_t func_read = 0x03;
    uint8_t buffer[16] = {0x00, 0x04};
    
    while (!i2c_get_ack(port, addr, 1000000)) 
        delay_1ms(200);
    delay_1ms(2);

    i2c_write_register(port, addr, func_read, 2, buffer);
    delay_1ms(2);

    i2c_read_register(port, addr, 0x00, 8, buffer);
    ret.temperature = ((buffer[4] << 8) | buffer[5]) / 10.0f;

    return ret;
}

void showTemperature(am2320sensor_t sensor) {
    LCD_ShowStr(0, 5, "Temperature: ", GREEN, TRANSPARENT);
    LCD_ShowNum1(8 * 13, 5, sensor.temperature, 4, YELLOW);
    LCD_Wait_On_Queue();
}

void shift_array_right_and_draw(int array[]) {
    for (int i = SCREEN_WIDTH; i > 0; i--) {
        array[i] = array[i - 1];
        LCD_DrawPoint_big(i, array[i], WHITE);
        LCD_DrawPoint_big(i - 1, array[i - 1], BLACK);
    }
}

int read_adc_potentiometer(void) {
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
    while(!adc_flag_get(ADC0, ADC_FLAG_EOC));
    int adcr = adc_regular_data_read(ADC0) >> 6;
    adc_flag_clear(ADC0, ADC_FLAG_EOC);
    return adcr;
}

void updateScreen(int *screen_row, int adcr) {
    screen_row[0] = adcr;
    shift_array_right_and_draw(screen_row);
}

//For waking up the sensor, sends an adress and waits for ack until timeout
int i2c_get_ack(int32_t port, uint8_t addr, uint32_t timeout){
    /* send a start condition to I2C bus */
    i2c_start_on_bus(port);
    while(!i2c_flag_get(port, I2C_FLAG_SBSEND));

    /* send slave address to I2C bus with write flag */
    i2c_master_addressing(port, addr << 1, I2C_TRANSMITTER);
    /* Wait for sending address to finish */
    while(!i2c_flag_get(port, I2C_FLAG_ADDSEND) && (timeout--));
    i2c_flag_clear(port, I2C_FLAG_ADDSEND);

    i2c_stop_on_bus(port);
    while( I2C_CTL0(port) & I2C_CTL0_STOP );

    return (timeout > 0);

}

/* You can copy this function to be able to read from most i2c devices */
void i2c_read_register(int32_t port, uint8_t addr, uint8_t reg, uint8_t size, uint8_t *data){

    /* send a start condition to I2C bus */
    i2c_start_on_bus(port);
    while(!i2c_flag_get(port, I2C_FLAG_SBSEND));

    /* send slave address to I2C bus with write flag */
    i2c_master_addressing(port, addr << 1, I2C_TRANSMITTER);
    /* Wait for sending address to finish */
    while(!i2c_flag_get(port, I2C_FLAG_ADDSEND));
    i2c_flag_clear(port, I2C_FLAG_ADDSEND);

    /* Send which register to read */
    i2c_data_transmit(port, reg);
    /* wait until the data has been sent */
    while(!i2c_flag_get(port, I2C_FLAG_TBE));

    /* Send new start condition */
    i2c_start_on_bus(port);
    while( ! i2c_flag_get(port, I2C_FLAG_SBSEND) );

    /* Now send address with read flag */
    i2c_master_addressing(port, addr << 1, I2C_RECEIVER);
    while( ! i2c_flag_get(port, I2C_FLAG_ADDSEND) );
    i2c_flag_clear(port, I2C_FLAG_ADDSEND);

    /* Enable acknowledge for receiving multiple bytes */
    i2c_ack_config(port, I2C_ACK_ENABLE);

    /* Receive bytes, read into buffer. */
    for(int i = 0; i < size; i++) {
        if(i == size - 1) {
            /* If last byte, do not send ack */
            i2c_ack_config(port, I2C_ACK_DISABLE);
        }
        while(!i2c_flag_get(port, I2C_FLAG_RBNE));
        *data++ = i2c_data_receive(port);
    }

    i2c_stop_on_bus(port);
    while( I2C_CTL0(port) & I2C_CTL0_STOP );
}

/* You can copy this function to be able to write most i2c devices */
void i2c_write_register(int32_t port, uint8_t addr, uint8_t reg, uint8_t size, uint8_t *data)
{    
    /* send a NACK for the next data byte which will be received into the shift register */
    while(i2c_flag_get(port, I2C_FLAG_I2CBSY));

    /* send a start condition to I2C bus */
    i2c_start_on_bus(port);
    while(!i2c_flag_get(port, I2C_FLAG_SBSEND));

    /* send slave address to I2C bus */
    i2c_master_addressing(port, addr << 1, I2C_TRANSMITTER);
    while(!i2c_flag_get(port, I2C_FLAG_ADDSEND));
    i2c_flag_clear(port, I2C_FLAG_ADDSEND);

    
    /* Send which register to write */
    i2c_data_transmit(port, reg);
    /* wait until the TBE bit is set */
    while(!i2c_flag_get(port, I2C_FLAG_TBE));

    /* Send data */
    for(int i = 0; i < size; i++){
        i2c_data_transmit(port, *data++);
        /* wait until the TBE bit is set */
        while(!i2c_flag_get(port, I2C_FLAG_TBE));
    }

    /* Send stop condition */
    i2c_stop_on_bus(port);
    while( I2C_CTL0(port) & I2C_CTL0_STOP );
}

void timer(int *time, int speed2)
{
    static int i = 0; //9  sinusvåg
    int sinLookUpTbl[9] = {2047*sin(0), 2047*sin(45), 2047*sin(90), 2047*sin(135), 
                         2047*sin(180), 2047*sin(225), 2047*sin(270), 2047*sin(315),
                        2047*sin(360)};
    
        *time=0; //återställer inför en ny vågpunkt
        DAC0set(sinLookUpTbl[i]); // spela upp varje plats 
        i++; //flyytar fram i sinusvågstabellen
        if(i==9) i = 0;  // återställer när sista punkten har nåtts
}

