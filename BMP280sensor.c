#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

//Para altura
// Constantes
#define P0 760 // Presión de referencia al nivel del mar en Pascal (Pa)
#define U 0.0289644// Masa molar del aire en kg/mol
#define G 9.80665 // Aceleración debida a la gravedad en m/s^2
#define R 8.31432 // Constante del gas en J/(kg·K) para el aire seco

#define BMP280_ADDR 0x76 // Cambia a 0x77 si se conecta el pin SDO a Vcc

// Registros del BMP280
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_TEMP_MSB 0xFA
#define BMP280_REG_DIG_T 0x88


uint8_t reg;
uint8_t var1;
uint8_t var2;
uint8_t var3;
uint8_t reg2;

typedef long signed int BMP280_S32_t;
typedef long unsigned int BMP280_U32_t;

// Variables para almacenar los datos del sensor
unsigned short dig_T1 = 27504;
signed short dig_T2 = 26435;
signed short dig_T3 = -1000;

unsigned short dig_P1 = 36477;
signed short dig_P2 = -10685;
signed short dig_P3 = 3024;
signed short dig_P4 = 2855;
signed short dig_P5 = 140;
signed short dig_P6 = -7;
signed short dig_P7 = 15500;
signed short dig_P8 = -14600;
signed short dig_P9 = 6000;

static const char *TAG = "BMP280";

static esp_err_t i2c_master_init() {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21, // Cambia estos pines según tu conexión I2C
        .scl_io_num = 22, // Cambia estos pines según tu conexión I2C
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000 // Velocidad de comunicación I2C (100 kHz)
    };

    esp_err_t err = i2c_param_config(I2C_NUM_0, &i2c_config);
    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

static void bmp280_init() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // Crear un nuevo comando I2C

    // Configurar el modo de medición de temperatura y presión normal (0x77)
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_CTRL_MEAS, true);
    i2c_master_write_byte(cmd, 0x77, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error escribiendo en el registro: %d", err);
    }
    i2c_cmd_link_delete(cmd);

        cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &reg, I2C_MASTER_NACK);
     // Leyendo el último byte con NACK
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo el registro: %d", err);
    }
    i2c_cmd_link_delete(cmd);

    // Leer el valor del registro BMP280_REG_CONFIG
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_CONFIG, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error escribiendo en el registro: %d", err);
    }
    i2c_cmd_link_delete(cmd);

    // Leer el valor del registro BMP280_REG_CONFIG
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &var1, I2C_MASTER_NACK); // Leyendo el último byte con NACK
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo el registro: %d", err);
    }
    i2c_cmd_link_delete(cmd);

    var2 = var1 & 0b00000010;
    var3 = var2 | 0b00010000;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_CONFIG, true);
    i2c_master_write_byte(cmd, var3, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error escribiendo en el registro: %d", err);
    }
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &reg2, I2C_MASTER_NACK); // Leyendo el último byte con NACK
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo el registro: %d", err);
    }
    i2c_cmd_link_delete(cmd);



}

// Función para calcular la altura absoluta
double calculateAltitude(double Ph, double T) {
    double h;
    
    double T1 = T + 273;
    double P1 = Ph*0.00750062;
    // Calcular la altura utilizando la fórmula
    h = (-R * T1 / (U * G)) * log(P1 / P0);
    
    return h;
}



//Compensacion temperatura
BMP280_S32_t t_fine;
BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T){
    BMP280_S32_t var1,var2,T;

    var1 = ((((adc_T>>3) - ((BMP280_S32_t)dig_T1<<1))) * ((BMP280_S32_t)dig_T2)) >> 11;
    var2 = ((((adc_T>>4) - ((BMP280_S32_t)dig_T1)) * ((adc_T>>4) - ((BMP280_S32_t )dig_T1))) >> 12) * 
        ((BMP280_S32_t)dig_T3) >> 14;
    t_fine = var1 + var2;
    T = (t_fine  * 5 + 128) >> 8;
    return T;


}

BMP280_U32_t bmp280_compensate_P_int32(BMP280_S32_t adc_P)
{
    BMP280_S32_t var1, var2;
    BMP280_U32_t p;

    var1 = (((BMP280_S32_t)t_fine) >> 1) - ((BMP280_S32_t)64000);
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((BMP280_S32_t)dig_P6);
    var2 = var2 + ((var1 * ((BMP280_S32_t)dig_P5)) << 1);
    var2 = (var2 >> 2) + (((BMP280_S32_t)dig_P4) << 16);
    var1 = ((((dig_P3) * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((BMP280_S32_t)dig_P2) * var1) >> 1)) >> 18;
    var1 = (((32768 + var1) * ((BMP280_S32_t)dig_P1)) >> 15);

    if (var1 == 0)
    {
        return 0; // Evitar la excepción causada por la división por cero
    }

    p = (((BMP280_U32_t)(((BMP280_S32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;

    if (p < 0x80000000)
    {
        p = (p << 1) / ((BMP280_U32_t)var1);
    }
    else
    {
        p = (p / (BMP280_U32_t)var1) * 2;
    }

    var1 = (((BMP280_S32_t)dig_P9) * ((BMP280_S32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((BMP280_S32_t)(p >> 2)) * ((BMP280_S32_t)dig_P8)) >> 13;
    p = (BMP280_U32_t)((BMP280_S32_t)p + ((var1 + var2 + dig_P7) >> 4));

    return p;
}

static void bmp280_read_data()
{
    // Leer datos de temperatura y presión del BMP280
    uint8_t data[6];


    BMP280_U32_t temp;
    BMP280_U32_t pressure;
    //Lectura de REG_TEMP_MSB
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP280_REG_PRESS_MSB, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(50 / portTICK_RATE_MS); // Esperar a que se complete la conversión de temperatura

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, (uint8_t*)data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(50 / portTICK_RATE_MS);

    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);

    temp = bmp280_compensate_T_int32(adc_T);
    pressure = bmp280_compensate_P_int32(adc_P);


    float temperatura_decimal = (float)temp / 100.0;
    float presion_decimal = (float)pressure / 100.0;
    double altitude = calculateAltitude(pressure, temperatura_decimal);
    
    // Imprime los valores en el formato deseado
    printf("Temperatura: %.2f°C ", temperatura_decimal);
    printf("Presión: %.3fhPa ", presion_decimal);
    printf("Altitud absoluta: %.2lfmetros\n", altitude);;

}


void app_main() {
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando el bus I2C.");
        return;
    }

    bmp280_init();

    while (1) {
        
        bmp280_read_data();
        vTaskDelay(1000 / portTICK_RATE_MS); // Leer y mostrar el valor cada segundo
    }
}

