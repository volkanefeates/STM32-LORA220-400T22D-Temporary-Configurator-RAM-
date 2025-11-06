#include "Lora_Settings.h"
#include <string.h>

#define E220_UART_TIMEOUT 1000
#define E220_AUX_TIMEOUT 5000


static E220_Status_t E220_WaitForAux(E220_Handle_t* dev, GPIO_PinState state, uint32_t timeout) {
    uint32_t start_tick = HAL_GetTick();
    while (HAL_GPIO_ReadPin(dev->aux_port, dev->aux_pin) != state) {
        if (HAL_GetTick() - start_tick > timeout) {
            return E220_TIMEOUT;
        }
    }
    return E220_OK;
}

void E220_Init(E220_Handle_t* dev, UART_HandleTypeDef* huart,
               GPIO_TypeDef* m0_port, uint16_t m0_pin,
               GPIO_TypeDef* m1_port, uint16_t m1_pin,
               GPIO_TypeDef* aux_port, uint16_t aux_pin) {
    dev->huart = huart;
    dev->m0_port = m0_port;
    dev->m0_pin = m0_pin;
    dev->m1_port = m1_port;
    dev->m1_pin = m1_pin;
    dev->aux_port = aux_port;
    dev->aux_pin = aux_pin;
}

E220_Status_t E220_SetMode(E220_Handle_t* dev, E220_Mode_t mode) {
    if (E220_WaitForAux(dev, GPIO_PIN_SET, E220_AUX_TIMEOUT) != E220_OK) {
        return E220_TIMEOUT;
    }

    switch (mode) {
        case MODE_0_NORMAL:
            HAL_GPIO_WritePin(dev->m0_port, dev->m0_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(dev->m1_port, dev->m1_pin, GPIO_PIN_RESET);
            break;
        case MODE_1_WOR_TX:
            HAL_GPIO_WritePin(dev->m0_port, dev->m0_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(dev->m1_port, dev->m1_pin, GPIO_PIN_RESET);
            break;
        case MODE_2_WOR_RX:
            HAL_GPIO_WritePin(dev->m0_port, dev->m0_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(dev->m1_port, dev->m1_pin, GPIO_PIN_SET);
            break;
        case MODE_3_CONFIG:
            HAL_GPIO_WritePin(dev->m0_port, dev->m0_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(dev->m1_port, dev->m1_pin, GPIO_PIN_SET);
            break;
    }

    HAL_Delay(10);
    return E220_WaitForAux(dev, GPIO_PIN_SET, E220_AUX_TIMEOUT);
}

E220_Status_t E220_ReadRegister(E220_Handle_t* dev, uint8_t start_addr, uint8_t len, uint8_t* buffer) {
    uint8_t tx_buffer[3] = {0xC1, start_addr, len};
    uint8_t rx_buffer[256];
    uint8_t expected_rx_len = len + 3;

    if (HAL_UART_Transmit(dev->huart, tx_buffer, 3, E220_UART_TIMEOUT) != HAL_OK) {
        return E220_ERROR;
    }

    if (HAL_UART_Receive(dev->huart, rx_buffer, expected_rx_len, E220_UART_TIMEOUT) != HAL_OK) {
        return E220_TIMEOUT;
    }

    if (rx_buffer[0] == 0xC1 && rx_buffer[1] == start_addr && rx_buffer[2] == len) {
        memcpy(buffer, &rx_buffer[3], len);
        return E220_OK;
    }

    return E220_INVALID_RESPONSE;
}

E220_Status_t E220_WriteTempRegister(E220_Handle_t* dev, uint8_t start_addr, uint8_t len, uint8_t* data) {
    uint8_t tx_buffer[256];
    uint8_t rx_buffer[256];
    uint8_t packet_len = len + 3;

    tx_buffer[0] = 0xC2; // Geçici yazma komutu
    tx_buffer[1] = start_addr;
    tx_buffer[2] = len;
    memcpy(&tx_buffer[3], data, len);

    if (HAL_UART_Transmit(dev->huart, tx_buffer, packet_len, E220_UART_TIMEOUT) != HAL_OK) {
        return E220_ERROR;
    }

    if (HAL_UART_Receive(dev->huart, rx_buffer, packet_len, E220_UART_TIMEOUT) != HAL_OK) {
        return E220_TIMEOUT;
    }

    if (rx_buffer[0] == 0xC1 && memcmp(&rx_buffer[1], &tx_buffer[1], len + 2) == 0) {
        return E220_OK;
    }

    return E220_INVALID_RESPONSE;
}

static void E220_ParseRegisters(E220_Handle_t* dev, uint8_t* reg_buffer) {
    E220_Settings_t* settings = &dev->settings;

    settings->address = (reg_buffer[0] << 8) | reg_buffer[1];
    settings->uart_baud = (reg_buffer[2] >> 5) & 0b111;
    settings->uart_parity = (reg_buffer[2] >> 3) & 0b011;
    settings->air_rate = reg_buffer[2] & 0b111;
    settings->packet_size = (reg_buffer[3] >> 6) & 0b011;
    settings->enable_rssi_noise = (reg_buffer[3] >> 5) & 0b1;
    settings->power = reg_buffer[3] & 0b011;
    settings->channel = reg_buffer[4];
    settings->enable_rssi_byte = (reg_buffer[5] >> 7) & 0b1;
    settings->tx_mode = (reg_buffer[5] >> 6) & 0b1;
    settings->enable_lbt = (reg_buffer[5] >> 4) & 0b1;
    settings->wor_cycle = reg_buffer[5] & 0b111;
}

static void E220_BuildRegisters(E220_Handle_t* dev, uint8_t* reg_buffer) {
    E220_Settings_t* settings = &dev->settings;

    reg_buffer[0] = (settings->address >> 8) & 0xFF;
    reg_buffer[1] = settings->address & 0xFF;
    reg_buffer[2] = (settings->uart_baud << 5) | (settings->uart_parity << 3) | settings->air_rate;
    reg_buffer[3] = (settings->packet_size << 6) | (settings->enable_rssi_noise << 5) | (settings->power);
    reg_buffer[4] = settings->channel;
    reg_buffer[5] = (settings->enable_rssi_byte << 7) | (settings->tx_mode << 6) | (settings->enable_lbt << 4) | (settings->wor_cycle);
    reg_buffer[6] = (settings->encryption_key >> 8) & 0xFF;
    reg_buffer[7] = settings->encryption_key & 0xFF;
}

E220_Status_t E220_LoadSettingsFromModule(E220_Handle_t* dev) {
    E220_Status_t status;

    status = E220_SetMode(dev, MODE_3_CONFIG);
    if (status != E220_OK) return status;

    uint8_t read_buffer[8];
    status = E220_ReadRegister(dev, 0x00, 8, read_buffer);

    if (status == E220_OK) {
        E220_ParseRegisters(dev, read_buffer);
    }

    E220_SetMode(dev, MODE_0_NORMAL);
    return status;
}

E220_Status_t E220_SaveSettingsToRAM(E220_Handle_t* dev) {
    E220_Status_t status;

    if (dev->settings.channel > 83) return E220_INVALID_PARAM;

    uint8_t write_buffer[8];
    E220_BuildRegisters(dev, write_buffer);

    status = E220_SetMode(dev, MODE_3_CONFIG);
    if (status != E220_OK) return status;

    // Parça 1: Adres 0x00, Uzunluk 3 (Adres ve REG0)
    status = E220_WriteTempRegister(dev, 0x00, 3, &write_buffer[0]);
    if (status != E220_OK) {
        E220_SetMode(dev, MODE_0_NORMAL);
        return status;
    }

    HAL_Delay(5);

    // Parça 2: Adres 0x03, Uzunluk 1 (REG1)
    status = E220_WriteTempRegister(dev, 0x03, 1, &write_buffer[3]);
    if (status != E220_OK) {
        E220_SetMode(dev, MODE_0_NORMAL);
        return status;
    }

    HAL_Delay(5);

    // Parça 3: Adres 0x04, Uzunluk 1 (REG2 - Kanal)
    status = E220_WriteTempRegister(dev, 0x04, 1, &write_buffer[4]);
    if (status != E220_OK) {
        E220_SetMode(dev, MODE_0_NORMAL);
        return status;
    }

    HAL_Delay(5);

    // Parça 4: Adres 0x05, Uzunluk 1 (REG3)
    status = E220_WriteTempRegister(dev, 0x05, 1, &write_buffer[5]);
    if (status != E220_OK) {
        E220_SetMode(dev, MODE_0_NORMAL);
        return status;
    }

    HAL_Delay(5);

    // Parça 5: Adres 0x06, Uzunluk 2 (Şifreleme Key)
    status = E220_WriteTempRegister(dev, 0x06, 2, &write_buffer[6]);
    if (status != E220_OK) {
        E220_SetMode(dev, MODE_0_NORMAL);
        return status;
    }

    return E220_SetMode(dev, MODE_0_NORMAL);
}
