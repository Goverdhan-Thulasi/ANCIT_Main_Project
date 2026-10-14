#include "sdk_project_config.h"
#include <stdio.h>
#include <string.h>

#define USE_UART            1
#define DEBUG_UNLOCK_ENABLE 1

#define EEE_SIZE_CODE       0x06u
#define DEPART_CODE         0x03u

#define EEE_BASE            0x14000000UL
#define EE_LOCK_OFFSET      0u
#define EE_DATA_OFFSET      4u
#define LOCK_MAGIC          0xA5u
#define BLANK_BYTE          0xFFu

#define LED_PORT            PTD
#define LED_PIN             15u
#define BTN_PORT            PTC
#define BTN_PIN             12u
#define LED_ON()            PINS_DRV_ClearPins(LED_PORT, (1u << LED_PIN))
#define LED_OFF()           PINS_DRV_SetPins(LED_PORT, (1u << LED_PIN))
#define BTN_PRESSED()       ((PINS_DRV_ReadPins(BTN_PORT) & (1u << BTN_PIN)) != 0u)

static flash_ssd_config_t flashSSDConfig;

static volatile uint8_t g_locked   = 0u;
static volatile uint8_t g_initDone = 0u;
static uint32_t         g_attempt  = 0u;

typedef enum {
    WR_OK = 0,
    WR_REJ_NOT_INIT,
    WR_REJ_LOCKED,
    WR_ERR_TIMEOUT,
    WR_ERR_FLASH,
    WR_ERR_VERIFY_DATA,
    WR_ERR_VERIFY_LOCK
} WriteResult_t;

static void delay_ms(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms * 6000u; i++) { }
}

static void print(const char *s)
{
#if USE_UART
    LPUART_DRV_SendDataBlocking(INST_LPUART_1, (const uint8_t *)s, (uint32_t)strlen(s), 100u);
#else
    (void)s;
#endif
}

static void printf_uart(const char *fmt, uint32_t v)
{
    char buf[80];
    (void)snprintf(buf, sizeof(buf), fmt, (unsigned long)v);
    print(buf);
}

static bool EE_WaitReady(void)
{
    uint32_t t = 2000000u;
    while ((FTFC->FCNFG & FTFC_FCNFG_EEERDY_MASK) == 0u) {
        if (--t == 0u) { return false; }
    }
    return true;
}

static bool EE_NoError(void)
{
    uint8_t s = FTFC->FSTAT & (FTFC_FSTAT_ACCERR_MASK | FTFC_FSTAT_FPVIOL_MASK);
    if (s != 0u) {
        FTFC->FSTAT = s;
        return false;
    }
    return true;
}

static bool EE_IsPartitioned(void)
{
    uint32_t sz = (SIM->FCFG1 & SIM_FCFG1_EEERAMSIZE_MASK) >> SIM_FCFG1_EEERAMSIZE_SHIFT;
    return (sz != 0xFu);
}

static bool EE_Init(void)
{
    status_t st = FLASH_DRV_Init(&Flash_InitConfig0, &flashSSDConfig);
    if (st != STATUS_SUCCESS) { return false; }

    if (!EE_IsPartitioned()) {
        st = FLASH_DRV_DEFlashPartition(&flashSSDConfig,
                                        EEE_SIZE_CODE,
                                        DEPART_CODE,
                                        0u,
                                        false,
                                        true);
        if (st != STATUS_SUCCESS) { return false; }
        S32_SCB->AIRCR = S32_SCB_AIRCR_VECTKEY(0x05FAu) | S32_SCB_AIRCR_SYSRESETREQ_MASK;
        for (;;) { }
    }

    if ((FTFC->FCNFG & FTFC_FCNFG_EEERDY_MASK) == 0u) {
        st = FLASH_DRV_SetFlexRamFunction(&flashSSDConfig, EEE_ENABLE, 0u, NULL);
        if (st != STATUS_SUCCESS) { return false; }
    }
    return EE_WaitReady();
}

static bool EE_WriteU8(uint32_t offset, uint8_t v)
{
    if (!EE_WaitReady()) { return false; }
    *(volatile uint8_t *)(EEE_BASE + offset) = v;
    if (!EE_WaitReady()) { return false; }
    return EE_NoError();
}

static bool EE_WriteU32(uint32_t offset, uint32_t v)
{
    if (!EE_WaitReady()) { return false; }
    *(volatile uint32_t *)(EEE_BASE + offset) = v;
    if (!EE_WaitReady()) { return false; }
    return EE_NoError();
}

static uint8_t EE_ReadU8(uint32_t offset)
{
    return *(volatile uint8_t *)(EEE_BASE + offset);
}

static uint32_t EE_ReadU32(uint32_t offset)
{
    return *(volatile uint32_t *)(EEE_BASE + offset);
}

static void ShowLockState(void)
{
    if (g_locked) { LED_ON(); } else { LED_OFF(); }
}

static void Lock_Restore(void)
{
    uint8_t lockByte = EE_ReadU8(EE_LOCK_OFFSET);

    if (lockByte == BLANK_BYTE) {
        g_locked = 0u;
        print("\r\n[BOOT] Lock byte blank -> UNLOCKED\r\n");
    } else if (lockByte == LOCK_MAGIC) {
        g_locked = 1u;
        printf_uart("\r\n[BOOT] Lock byte OK -> LOCKED, data = 0x%08lX\r\n", EE_ReadU32(EE_DATA_OFFSET));
    } else {
        g_locked = 1u;
        printf_uart("\r\n[BOOT] Lock byte CORRUPT (0x%02lX) -> treating as LOCKED\r\n", lockByte);
    }
    g_initDone = 1u;
    ShowLockState();
}

static WriteResult_t WriteOnce(uint32_t value)
{
    if (!g_initDone) { return WR_REJ_NOT_INIT; }
    if (g_locked)    { return WR_REJ_LOCKED;   }

    if (!EE_WaitReady())                         { return WR_ERR_TIMEOUT; }
    if (!EE_WriteU32(EE_DATA_OFFSET, value))     { return WR_ERR_FLASH; }
    if (EE_ReadU32(EE_DATA_OFFSET) != value)     { return WR_ERR_VERIFY_DATA; }

    if (!EE_WriteU8(EE_LOCK_OFFSET, LOCK_MAGIC)) { return WR_ERR_FLASH; }
    if (EE_ReadU8(EE_LOCK_OFFSET) != LOCK_MAGIC) { return WR_ERR_VERIFY_LOCK; }

    g_locked = 1u;
    ShowLockState();
    return WR_OK;
}

static void ReportResult(WriteResult_t r, uint32_t value)
{
    switch (r) {
    case WR_OK:              printf_uart("[WRITE] OK, data = 0x%08lX, now LOCKED\r\n", value); break;
    case WR_REJ_NOT_INIT:    print("[WRITE] REJECTED: not initialised\r\n");                  break;
    case WR_REJ_LOCKED:      print("[WRITE] REJECTED: already LOCKED\r\n");                    break;
    case WR_ERR_TIMEOUT:     print("[WRITE] ERROR: EEPROM timeout\r\n");                       break;
    case WR_ERR_FLASH:       print("[WRITE] ERROR: flash access/protection error\r\n");        break;
    case WR_ERR_VERIFY_DATA: print("[WRITE] ERROR: data read-back mismatch (still unlocked)\r\n"); break;
    case WR_ERR_VERIFY_LOCK: print("[WRITE] ERROR: lock read-back mismatch\r\n");              break;
    default:                 print("[WRITE] ERROR: unknown\r\n");                              break;
    }
}

int main(void)
{
    CLOCK_DRV_Init(&clockMan1_InitConfig0);
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS0, g_pin_mux_InitConfigArr0);

#if USE_UART
    LPUART_DRV_Init(INST_LPUART_1, &lpUartState0, &lpuart_0_InitConfig0);
#endif

    LED_OFF();
    print("\r\n=== Write-once EEPROM lock demo ===\r\n");

    if (!EE_Init()) {
        print("[FATAL] EEPROM init failed\r\n");
        for (;;) { }
    }

#if DEBUG_UNLOCK_ENABLE
    if (BTN_PRESSED()) {
        delay_ms(50);
        if (BTN_PRESSED()) {
            (void)EE_WriteU8(EE_LOCK_OFFSET, BLANK_BYTE);
            print("[DEBUG] Lock byte cleared (test-only unlock)\r\n");
            while (BTN_PRESSED()) { }
            delay_ms(50);
        }
    }
#endif

    Lock_Restore();
    print("Press SW2 to attempt a write.\r\n");

    for (;;) {
        if (BTN_PRESSED()) {
            delay_ms(30);
            if (BTN_PRESSED()) {
                g_attempt++;
                uint32_t value = 0xD0000000u + g_attempt;
                WriteResult_t r = WriteOnce(value);
                ReportResult(r, value);
                while (BTN_PRESSED()) { }
                delay_ms(30);
            }
        }
    }
}
