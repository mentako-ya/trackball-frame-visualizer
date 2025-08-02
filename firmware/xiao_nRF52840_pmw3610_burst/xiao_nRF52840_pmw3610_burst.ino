// This code is based on https://github.com/inorichi/zmk-pmw3610-driver
#include "Adafruit_TinyUSB.h"
#include "nrf.h"

//#define SCLK D9
//#define SDIO D10
//#define MOTION D8
//#define NCS D7  // Required for burst read!

#define SCLK D4
#define SDIO D5
#define NCS  9
#define MOTION 10

#define BURST_INTERVAL_MS 100  // バーストリードの最小間隔（ミリ秒）

/* Sensor registers (addresses) */
#define PMW3610_REG_PRODUCT_ID 0x00
#define PMW3610_REG_REVISION_ID 0x01
#define PMW3610_REG_MOTION 0x02
#define PMW3610_REG_DELTA_X_L 0x03
#define PMW3610_REG_DELTA_Y_L 0x04
#define PMW3610_REG_DELTA_XY_H 0x05
#define PMW3610_REG_SQUAL 0x06
#define PMW3610_REG_SHUTTER_HIGHER 0x07
#define PMW3610_REG_SHUTTER_LOWER 0x08
#define PMW3610_REG_PIX_MAX 0x09
#define PMW3610_REG_PIX_AVG 0x0A
#define PMW3610_REG_PIX_MIN 0x0B

#define PMW3610_REG_CRC0 0x0C
#define PMW3610_REG_CRC1 0x0D
#define PMW3610_REG_CRC2 0x0E
#define PMW3610_REG_CRC3 0x0F
#define PMW3610_REG_SELF_TEST 0x10

#define PMW3610_REG_PERFORMANCE 0x11
#define PMW3610_REG_MOTION_BURST 0x12

#define PMW3610_REG_RUN_DOWNSHIFT 0x1B
#define PMW3610_REG_REST1_PERIOD 0x1C
#define PMW3610_REG_REST1_DOWNSHIFT 0x1D
#define PMW3610_REG_REST2_PERIOD 0x1E
#define PMW3610_REG_REST2_DOWNSHIFT 0x1F
#define PMW3610_REG_REST3_PERIOD 0x20
#define PMW3610_REG_OBSERVATION 0x2D

#define PMW3610_REG_PIXEL_GRAB 0x35
#define PMW3610_REG_FRAME_GRAB 0x36

#define PMW3610_REG_POWER_UP_RESET 0x3A
#define PMW3610_REG_SHUTDOWN 0x3B

#define PMW3610_REG_SPI_CLK_ON_REQ 0x41
#define PMW3610_REG_RES_STEP 0x85

#define PMW3610_REG_NOT_REV_ID 0x3E
#define PMW3610_REG_NOT_PROD_ID 0x3F

#define PMW3610_REG_PRBS_TEST_CTL 0x47
#define PMW3610_REG_SPI_PAGE0 0x7F
#define PMW3610_REG_VCSEL_CTL 0x9E
#define PMW3610_REG_LSR_CONTROL 0x9F
#define PMW3610_REG_SPI_PAGE1 0xFF

/* Sensor identification values */
#define PMW3610_PRODUCT_ID 0x3E

/* Power-up register commands */
#define PMW3610_POWERUP_CMD_RESET 0x5A
#define PMW3610_POWERUP_CMD_WAKEUP 0x96

/* spi clock enable/disable commands */
#define PMW3610_SPI_CLOCK_CMD_ENABLE 0xBA
#define PMW3610_SPI_CLOCK_CMD_DISABLE 0xB5

/* write command bit position */
#define BIT(i) (1 << i)
#define PMW3610_WRITE_BIT BIT(7)

#define PMW3610_MOTION_MOT BIT(7) #1 : motion occured
#define PMW3610_MOTION_OVF BIT(4) #1 : motion overflow

#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

static uint8_t reg_read(uint8_t reg_address) {
    pinMode(SDIO, OUTPUT);
    digitalWrite(NCS, 0);
    for (int8_t i = 7; i >= 0; i--) {
        digitalWrite(SCLK, 0);
        digitalWrite(SDIO, reg_address & BIT(i));
        delayMicroseconds(1);
        digitalWrite(SCLK, 1);
        delayMicroseconds(1);
    }
    pinMode(SDIO, INPUT);
    delayMicroseconds(4);
    uint8_t r = 0;
    for (int8_t i = 7; i >= 0; i--) {
        digitalWrite(SCLK, 0);
        delayMicroseconds(1);
        digitalWrite(SCLK, 1);
        delayMicroseconds(1);
        r |= digitalRead(SDIO) << i;
    }
    digitalWrite(NCS, 1);
    delayMicroseconds(10);
    return r;
}

static void reg_burst_read(uint8_t *output, size_t size) {
    pinMode(SDIO, OUTPUT);
    digitalWrite(NCS, 0);
    uint8_t reg_address = 0x12;
    for (int8_t i = 7; i >= 0; i--) {
        digitalWrite(SCLK, 0);
        digitalWrite(SDIO, reg_address & BIT(i));
        delayMicroseconds(1);
        digitalWrite(SCLK, 1);
        delayMicroseconds(1);
    }
    pinMode(SDIO, INPUT);
    delayMicroseconds(4);
    for (size_t j = 0; j < size; j++) {
        uint8_t r = 0;
        for (int8_t i = 7; i >= 0; i--) {
            digitalWrite(SCLK, 0);
            delayMicroseconds(1);
            digitalWrite(SCLK, 1);
            delayMicroseconds(1);
            r |= digitalRead(SDIO) << i;
        }
        output[j] = r;
    }
    digitalWrite(NCS, 1);
    delayMicroseconds(10);
}

static void _reg_write(uint8_t reg_address, uint8_t data) {
    reg_address |= PMW3610_WRITE_BIT;
    digitalWrite(NCS, 0);
    pinMode(SDIO, OUTPUT);
    for (int8_t i = 7; i >= 0; i--) {
        digitalWrite(SCLK, 0);
        digitalWrite(SDIO, reg_address & BIT(i));
        delayMicroseconds(1);
        digitalWrite(SCLK, 1);
        delayMicroseconds(1);
    }
    for (int8_t i = 7; i >= 0; i--) {
        digitalWrite(SCLK, 0);
        digitalWrite(SDIO, data & BIT(i));
        delayMicroseconds(1);
        digitalWrite(SCLK, 1);
        delayMicroseconds(1);
    }
    digitalWrite(NCS, 1);
    delayMicroseconds(4);
}

static void reg_write(uint8_t reg_address, uint8_t data) {
    _reg_write(PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    _reg_write(reg_address, data);
    _reg_write(PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
}

// 200~3200, 1200 by default
static void set_cpi(uint32_t cpi) {
    uint8_t value = cpi / 200;
    reg_write(PMW3610_REG_SPI_PAGE0, 0xFF);
    reg_write(PMW3610_REG_RES_STEP, value);
    reg_write(PMW3610_REG_SPI_PAGE1, 0x00);
}

bool initialized = false;

void setup() {
    // NFCPINSレジスタを操作してGPIOとして有効化
    //uint32_t *p_NFCPINS = (uint32_t *)0x1000120C;
    //*p_NFCPINS = 0xFFFFFFF8;
    // NFCピン保護解除（GPIOとして使う）
    if ((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) == (UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos)) {
        // 書き込み許可
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy);

        // 保護フラグ解除
        NRF_UICR->NFCPINS &= ~UICR_NFCPINS_PROTECT_Msk;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy);

        // 書き込み禁止
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy);

        // MCUリセット（設定反映のため）
        NVIC_SystemReset();
    }

    pinMode(VBAT_ENABLE, OUTPUT);
    digitalWrite(VBAT_ENABLE, 0);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    pinMode(SCLK, OUTPUT);
    pinMode(SDIO, OUTPUT);
    pinMode(MOTION, INPUT);
    pinMode(NCS, OUTPUT);

    Serial.begin(115200);
    delay(1000);
    //Serial.println("DEBUG: setup() start"); // デバッグ用

    // ピン状態のデバッグ出力
    //Serial.print("DEBUG: SCLK(pin "); Serial.print(SCLK); Serial.print(") = "); Serial.println(digitalRead(SCLK));
    //Serial.print("DEBUG: SDIO(pin "); Serial.print(SDIO); Serial.print(") = "); Serial.println(digitalRead(SDIO));
    //Serial.print("DEBUG: MOTION(pin "); Serial.print(MOTION); Serial.print(") = "); Serial.println(digitalRead(MOTION));
    //Serial.print("DEBUG: NCS(pin "); Serial.print(NCS); Serial.print(") = "); Serial.println(digitalRead(NCS));

    initialized = init_pmw3610();
}

bool init_pmw3610() {
    reg_write(PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
    delay(50);

    reg_write(PMW3610_REG_OBSERVATION, 0x00);
    delay(10);
    uint8_t observation = reg_read(0x2D);
    if ((observation & 0x0F) != 0x0F) {
        Serial.println("Observation failure");
        digitalWrite(LED_RED, LOW);
        return false;
    } else {
        Serial.println("Observation OK");
        digitalWrite(LED_GREEN, LOW);
    }

    reg_read(PMW3610_REG_MOTION);
    reg_read(PMW3610_REG_DELTA_X_L);
    reg_read(PMW3610_REG_DELTA_Y_L);
    reg_read(PMW3610_REG_DELTA_XY_H);

    Serial.write("Product Id = ");
    Serial.println(reg_read(PMW3610_REG_PRODUCT_ID));
    Serial.write("Revision Id = ");
    Serial.println(reg_read(PMW3610_REG_REVISION_ID));
//    if (reg_read(PMW3610_REG_PRODUCT_ID) != PMW3610_PRODUCT_ID) {
//        Serial.println("Product ID mismatch");
//        digitalWrite(LED_RED, LOW);
//        return false;
//    } else {
        Serial.println("Product ID OK");
        digitalWrite(LED_GREEN, LOW);
//    }

    reg_write(PMW3610_REG_PERFORMANCE, 0xF1);  // disable rest mode

    set_cpi(200);
    return true;
}

uint8_t cnt = 0;

void burst_read_frame() {
    _reg_write(PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    // Turn on test clock
    _reg_write(0x32, 0x10);
    // Set FG_EN to enable frame grab
    _reg_write(PMW3610_REG_FRAME_GRAB, BIT(7));
    delay(10);

    Serial.println("P: Start"); // Start of frame

    uint8_t frames[484];  // 22 * 22 = 484 bytes
    reg_burst_read(frames, 484);
    for (int i = 0; i < 484; i++) {
        Serial.printf("P[%d]: %d\n", i, frames[i]);
    }
    digitalWrite(LED_BLUE, cnt++ % 2 == 0 ? HIGH : LOW);
}

unsigned long last_burst_time = 0;

void loop() {
    if (!initialized) {
        digitalWrite(LED_RED, cnt++ % 2 == 0 ? HIGH : LOW);
        delay(100);
        return;
    }
    if (Serial.available()) {
        String s = Serial.readString();
        if (s.startsWith("reset")) {
            init_pmw3610();
        }
    }
    unsigned long now = millis();
    if (now - last_burst_time < BURST_INTERVAL_MS) {
        delay(BURST_INTERVAL_MS - (now - last_burst_time));
    }
    last_burst_time = millis();
    burst_read_frame();
}
