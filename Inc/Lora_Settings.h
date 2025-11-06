/* Lora_Settings.h (SADECE GEÇİCİ YAZMA) */

#ifndef INC_LORA_SETTINGS_H_
#define INC_LORA_SETTINGS_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

typedef enum {
    AIR_RATE_2_4K = 0b000,
    AIR_RATE_4_8K = 0b001,
    AIR_RATE_9_6K = 0b010,
    AIR_RATE_19_2K = 0b011,
    AIR_RATE_38_4K = 0b100,
    AIR_RATE_62_5K = 0b101
} E220_AirRate_t;

typedef enum {
    UART_BAUD_1200 = 0b000,
    UART_BAUD_2400 = 0b001,
    UART_BAUD_4800 = 0b010,
    UART_BAUD_9600 = 0b011,
    UART_BAUD_19200 = 0b100,
    UART_BAUD_38400 = 0b101,
    UART_BAUD_57600 = 0b110,
    UART_BAUD_115200 = 0b111
} E220_UARTBaud_t;

typedef enum {
    PARITY_8N1 = 0b00,
    PARITY_8O1 = 0b01,
    PARITY_8E1 = 0b10
} E220_Parity_t;

typedef enum {
    POWER_22DBM = 0b00,
    POWER_17DBM = 0b01,
    POWER_13DBM = 0b10,
    POWER_10DBM = 0b11
} E220_Power_t;

typedef enum {
    WOR_CYCLE_500MS = 0b000,
    WOR_CYCLE_1000MS = 0b001,
    WOR_CYCLE_1500MS = 0b010,
    WOR_CYCLE_2000MS = 0b011,
    WOR_CYCLE_2500MS = 0b100,
    WOR_CYCLE_3000MS = 0b101,
    WOR_CYCLE_3500MS = 0b110,
    WOR_CYCLE_4000MS = 0b111
} E220_WORCycle_t;

typedef enum {
    PACKET_SIZE_200 = 0b00,
    PACKET_SIZE_128 = 0b01,
    PACKET_SIZE_64 = 0b10,
    PACKET_SIZE_32 = 0b11
} E220_PacketSize_t;

typedef enum {
    MODE_TRANSPARENT = 0,
    MODE_FIXED = 1
} E220_TXMode_t;


/* --- Tüm Ayarları Tutan Ana Yapı --- */
typedef struct {
    uint16_t address;
    E220_UARTBaud_t uart_baud;
    E220_Parity_t   uart_parity;
    E220_AirRate_t  air_rate;

    E220_PacketSize_t packet_size;
    bool              enable_rssi_noise;
    E220_Power_t      power;

    uint8_t           channel;

    bool              enable_rssi_byte;
    E220_TXMode_t     tx_mode;
    bool              enable_lbt;
    E220_WORCycle_t   wor_cycle;

    uint16_t          encryption_key;
} E220_Settings_t;


typedef struct {
    UART_HandleTypeDef* huart;
    GPIO_TypeDef* m0_port;
    uint16_t            m0_pin;
    GPIO_TypeDef* m1_port;
    uint16_t            m1_pin;
    GPIO_TypeDef* aux_port;
    uint16_t            aux_pin;

    E220_Settings_t settings;
} E220_Handle_t;


/* --- Mod Türleri ve Durum Kodları --- */
typedef enum {
    MODE_0_NORMAL     = 0,
    MODE_1_WOR_TX     = 1,
    MODE_2_WOR_RX     = 2,
    MODE_3_CONFIG     = 3
} E220_Mode_t;

typedef enum {
    E220_OK           = 0,
    E220_ERROR        = 1,
    E220_TIMEOUT      = 2,
    E220_INVALID_RESPONSE = 3,
    E220_INVALID_PARAM = 4
} E220_Status_t;


/* --- FONKSİYON PROTOTİPLERİ --- */

// Temel Fonksiyonlar
void E220_Init(E220_Handle_t* dev, UART_HandleTypeDef* huart,
               GPIO_TypeDef* m0_port, uint16_t m0_pin,
               GPIO_TypeDef* m1_port, uint16_t m1_pin,
               GPIO_TypeDef* aux_port, uint16_t aux_pin);

E220_Status_t E220_SetMode(E220_Handle_t* dev, E220_Mode_t mode);

E220_Status_t E220_ReadRegister(E220_Handle_t* dev, uint8_t start_addr, uint8_t len, uint8_t* buffer);
E220_Status_t E220_WriteTempRegister(E220_Handle_t* dev, uint8_t start_addr, uint8_t len, uint8_t* data);

E220_Status_t E220_LoadSettingsFromModule(E220_Handle_t* dev);
E220_Status_t E220_SaveSettingsToRAM(E220_Handle_t* dev);



#endif /* INC_LORA_SETTINGS_H_ */
