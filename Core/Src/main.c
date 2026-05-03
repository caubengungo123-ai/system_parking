/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : HỆ THỐNG BÃI ĐỖ XE THÔNG MINH
  *                   STM32F103C8Tx + FreeRTOS + RFID RC522 + DS1307 RTC
  *                   + LCD I2C + 2x Servo + MQ-2 + KY-026 + Buzzer + LED
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* USER CODE BEGIN Includes */
#include "CLCD_I2C.h"
#include "MFRC522_STM32.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
    CAR_OUT = 0,
    CAR_IN  = 1
} CarState_t;

typedef struct {
    uint8_t uid[4];
    CarState_t state;
    uint8_t entryHour;
    uint8_t entryMin;
} UserRecord_t;

typedef struct {
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} DS1307_Time_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FLASH_PAGE_ADDR     0x0801F800UL
#define WHITELIST_MAGIC     0xA55A
#define MAX_USERS           20

#define DS1307_ADDR         0xD0

#define SERVO1_CLOSED       90
#define SERVO1_OPEN         180
#define SERVO2_CLOSED       180
#define SERVO2_OPEN         90

#define GAS_THRESHOLD       2500

#define PRICE_PER_HOUR      5000

#define BTN1_PRESSED()  (HAL_GPIO_ReadPin(nut_bam_1_GPIO_Port, nut_bam_1_Pin) == GPIO_PIN_RESET)
#define BTN2_PRESSED()  (HAL_GPIO_ReadPin(nut_bam_2_GPIO_Port, nut_bam_2_Pin) == GPIO_PIN_RESET)

#define LCD_I2C_ADDR        0x4E
#define LCD_COLS            16
#define LCD_ROWS            2
#define ADMIN_CARD_UID   {0xB3, 0x04, 0xF6, 0x13}

#define Fan_On()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define Fan_Off()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BCD2DEC(x)  ((((x) >> 4) & 0x0F) * 10 + ((x) & 0x0F))
#define DEC2BCD(x)  ((((x) / 10) << 4) | ((x) % 10))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

TaskHandle_t defaultTaskHandle;

/* USER CODE BEGIN PV */

CLCD_I2C_Name LCD;
MFRC522_t rfid;

typedef struct {
    uint16_t magic;
    uint8_t  count;
    UserRecord_t users[MAX_USERS];
} WhitelistStore_t;

static WhitelistStore_t wlStore;

static volatile uint8_t emergencyFlag = 0;
static const uint8_t adminCardUID[4] = ADMIN_CARD_UID;

/* ---- Mutex LCD ---- */
SemaphoreHandle_t lcdMutex;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void     Flash_ReadWhitelist(void);
static void     Flash_WriteWhitelist(void);
static void     Flash_ErasePage(uint32_t addr);
static int      WL_FindUser(const uint8_t *uid);
static uint8_t  WL_AddUser(const uint8_t *uid);
static void     WL_Reset(void);
static uint8_t  RFID_GetUID(uint8_t *uid);
static uint8_t  UID_Equal(const uint8_t *a, const uint8_t *b);
static void     DS1307_Init(void);
static void     DS1307_GetTime(DS1307_Time_t *t);
static void DS1307_SetTime(uint8_t hour, uint8_t min, uint8_t sec,
                           uint8_t date, uint8_t month, uint8_t year);
static void     Servo1_Open(void);
static void     Servo1_Close(void);
static void     Servo2_Open(void);
static void     Servo2_Close(void);
static void     Buzzer_Beep(uint16_t ms);
static void     Buzzer_On(void);
static void     Buzzer_Off(void);
static void     LED_On(void);
static void     LED_Off(void);
static void     LCD_Show2Lines(const char *line1, const char *line2);
static void     LCD_ShowTime(const char *label, uint8_t h, uint8_t m);
static uint32_t ADC_ReadMQ2(void);
static uint32_t CalcFee(uint8_t inHour, uint8_t inMin, uint8_t outHour, uint8_t outMin);
static void     AdminMode(void);
static void     Task_Safety(void *arg);
static void     Task_RFID(void *arg);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* ---- printf → UART2 ---- */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* ---- Stack overflow hook ---- */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    printf("[ERROR] Stack overflow: %s\r\n", pcTaskName);
    while (1) {}
}

/* ---- Malloc failed hook ---- */
void vApplicationMallocFailedHook(void)
{
    printf("[ERROR] Malloc failed! Heap exhausted.\r\n");
    while (1) {}
}

/* ============================================================
   FLASH
   ============================================================ */
static void Flash_ErasePage(uint32_t addr)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef eraseInit;
    eraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = addr;
    eraseInit.NbPages     = 1;
    uint32_t pageError    = 0;
    HAL_FLASHEx_Erase(&eraseInit, &pageError);
    HAL_FLASH_Lock();
}

static void Flash_WriteWhitelist(void)
{
    wlStore.magic = WHITELIST_MAGIC;
    Flash_ErasePage(FLASH_PAGE_ADDR);
    HAL_FLASH_Unlock();
    uint32_t addr  = FLASH_PAGE_ADDR;
    uint16_t *p    = (uint16_t *)&wlStore;
    uint32_t words = (sizeof(WhitelistStore_t) + 1) / 2;
    for (uint32_t i = 0; i < words; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, p[i]);
        addr += 2;
    }
    HAL_FLASH_Lock();
    printf("[FLASH] Saved, count=%d\r\n", wlStore.count);
}

static void Flash_ReadWhitelist(void)
{
    memcpy(&wlStore, (void *)FLASH_PAGE_ADDR, sizeof(WhitelistStore_t));
    if (wlStore.magic != WHITELIST_MAGIC) {
        memset(&wlStore, 0, sizeof(WhitelistStore_t));
        wlStore.magic = WHITELIST_MAGIC;
        wlStore.count = 0;
        printf("[FLASH] No whitelist – init empty\r\n");
    } else {
        printf("[FLASH] Loaded, count=%d\r\n", wlStore.count);
    }
}

/* ============================================================
   WHITELIST
   ============================================================ */
static int WL_FindUser(const uint8_t *uid)
{
    for (int i = 0; i < wlStore.count; i++)
        if (UID_Equal(wlStore.users[i].uid, uid)) return i;
    return -1;
}

static uint8_t WL_AddUser(const uint8_t *uid)
{
    if (wlStore.count >= MAX_USERS) return 0;
    if (WL_FindUser(uid) >= 0)      return 0;
    memcpy(wlStore.users[wlStore.count].uid, uid, 4);
    wlStore.users[wlStore.count].state = CAR_OUT;
    wlStore.count++;
    Flash_WriteWhitelist();
    return 1;
}

static void WL_Reset(void)
{
    memset(&wlStore, 0, sizeof(WhitelistStore_t));
    wlStore.magic = WHITELIST_MAGIC;
    wlStore.count = 0;
    Flash_WriteWhitelist();
}

/* ============================================================
   RFID
   ============================================================ */
static uint8_t UID_Equal(const uint8_t *a, const uint8_t *b)
{
    return (memcmp(a, b, 4) == 0);
}

static uint8_t RFID_GetUID(uint8_t *uid)
{
    uint8_t atqa[2];
    if (MFRC522_RequestA(&rfid, atqa) != STATUS_OK) return STATUS_TIMEOUT;
    return MFRC522_ReadUid(&rfid, uid);
}

/* ============================================================
   DS1307 RTC
   ============================================================ */
static void DS1307_Init(void)
{
    uint8_t reg0 = 0;
    HAL_I2C_Mem_Read(&hi2c1, DS1307_ADDR, 0x00, 1, &reg0, 1, 100);
    if (reg0 & 0x80) {
        reg0 &= ~0x80;
        HAL_I2C_Mem_Write(&hi2c1, DS1307_ADDR, 0x00, 1, &reg0, 1, 100);
        printf("[RTC] DS1307 clock started\r\n");
    }
}

static void DS1307_GetTime(DS1307_Time_t *t)
{
    uint8_t buf[7];
    HAL_I2C_Mem_Read(&hi2c1, DS1307_ADDR, 0x00, 1, buf, 7, 100);
    t->sec   = BCD2DEC(buf[0] & 0x7F);
    t->min   = BCD2DEC(buf[1]);
    t->hour  = BCD2DEC(buf[2] & 0x3F);
    t->day   = buf[3];
    t->date  = BCD2DEC(buf[4]);
    t->month = BCD2DEC(buf[5]);
    t->year  = BCD2DEC(buf[6]);
}

static void DS1307_SetTime(uint8_t hour, uint8_t min, uint8_t sec,
                           uint8_t date, uint8_t month, uint8_t year)
{
    uint8_t buf[7];
    buf[0] = DEC2BCD(sec);
    buf[1] = DEC2BCD(min);
    buf[2] = DEC2BCD(hour);
    buf[3] = 1;
    buf[4] = DEC2BCD(date);
    buf[5] = DEC2BCD(month);
    buf[6] = DEC2BCD(year);
    HAL_I2C_Mem_Write(&hi2c1, DS1307_ADDR, 0x00, 1, buf, 7, 100);
}
/* ============================================================
   SERVO
   ============================================================ */
static void Servo1_Open(void)  { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO1_OPEN);   }
static void Servo1_Close(void) { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO1_CLOSED); }
static void Servo2_Open(void)  { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SERVO2_OPEN);   }
static void Servo2_Close(void) { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SERVO2_CLOSED); }
/* ============================================================
   BUZZER / LED
   ============================================================ */
static void Buzzer_On(void)  { HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_SET);   }
static void Buzzer_Off(void) { HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET); }
static void Buzzer_Beep(uint16_t ms) { Buzzer_On(); vTaskDelay(pdMS_TO_TICKS(ms)); Buzzer_Off(); }
static void LED_On(void)  { HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);   }
static void LED_Off(void) { HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET); }

/* ============================================================
   ADC – MQ-2
   ============================================================ */
static uint32_t ADC_ReadMQ2(void)
{
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        return HAL_ADC_GetValue(&hadc1);
    return 0;
}

/* ============================================================
   LCD (thread-safe)
   ============================================================ */
static void LCD_Show2Lines(const char *line1, const char *line2)
{
    if (lcdMutex == NULL) return;   /* guard – mutex chưa tạo */
    xSemaphoreTake(lcdMutex, portMAX_DELAY);
    CLCD_I2C_Clear(&LCD);
    CLCD_I2C_SetCursor(&LCD, 0, 0);
    CLCD_I2C_WriteString(&LCD, (char *)line1);
    CLCD_I2C_SetCursor(&LCD, 0, 1);
    CLCD_I2C_WriteString(&LCD, (char *)line2);
    xSemaphoreGive(lcdMutex);
}

static void LCD_ShowTime(const char *label, uint8_t h, uint8_t m)
{
    char buf[17];
    snprintf(buf, sizeof(buf), "%s%02d:%02d", label, h, m);
    if (lcdMutex == NULL) return;
    xSemaphoreTake(lcdMutex, portMAX_DELAY);
    CLCD_I2C_SetCursor(&LCD, 0, 1);
    CLCD_I2C_WriteString(&LCD, buf);
    xSemaphoreGive(lcdMutex);
}

/* ============================================================
   TÍNH PHÍ
   ============================================================ */
static uint32_t CalcFee(uint8_t inHour, uint8_t inMin,
                        uint8_t outHour, uint8_t outMin)
{
    int32_t totalMin = ((int32_t)outHour * 60 + outMin)
                     - ((int32_t)inHour  * 60 + inMin);
    if (totalMin < 0) totalMin += 24 * 60;
    uint32_t hours = (uint32_t)(totalMin + 59) / 60;
    if (hours == 0) hours = 1;
    return hours * PRICE_PER_HOUR;
}

/* ============================================================
   ADMIN MODE
   ============================================================ */
static void AdminMode(void)
{
    printf("[ADMIN] Entered admin mode\r\n");
    LCD_Show2Lines("  Admin Mode  ", "Nhan B1 de ra");

    while (1) {
        /* ---- BTN1 giữ 3s: THOÁT admin ---- */
        if (BTN1_PRESSED()) {
            vTaskDelay(pdMS_TO_TICKS(50));  /* debounce */
            if (BTN1_PRESSED()) {
                uint32_t holdStart = HAL_GetTick();
                LCD_Show2Lines("Giu BTN1...   ", "de thoat Admin");
                while (BTN1_PRESSED()) {
                    if ((HAL_GetTick() - holdStart) >= 3000) {
                        LCD_Show2Lines(" Exit Admin   ", "");
                        printf("[ADMIN] Exit\r\n");
                        Buzzer_Beep(200);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        return;
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                /* Nhấn ngắn BTN1 → về màn hình admin */
                LCD_Show2Lines("  Admin Mode  ", "Nhan B1 de ra");
            }
        }

        /* ---- BTN2: Reset whitelist ---- */
        if (BTN2_PRESSED()) {
            vTaskDelay(pdMS_TO_TICKS(50));
            if (BTN2_PRESSED()) {
                /* Xác nhận: giữ BTN2 thêm 2s */
                LCD_Show2Lines("Giu BTN2...   ", "de xoa DS the");
                uint32_t holdStart = HAL_GetTick();
                while (BTN2_PRESSED()) {
                    if ((HAL_GetTick() - holdStart) >= 2000) {
                        WL_Reset();
                        LCD_Show2Lines("Whitelist Reset", "  Thanh cong! ");
                        printf("[ADMIN] Whitelist reset\r\n");
                        Buzzer_Beep(500);
                        vTaskDelay(pdMS_TO_TICKS(1500));
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                while (BTN2_PRESSED()) vTaskDelay(pdMS_TO_TICKS(10));
                LCD_Show2Lines("  Admin Mode  ", "Nhan B1 de ra");
            }
        }

        /* ---- Quét thẻ user ---- */
        uint8_t newUid[4];
        if (RFID_GetUID(newUid) == STATUS_OK) {

            /* Hiện UID ngay lập tức */
            char uidBuf[17];
            snprintf(uidBuf, sizeof(uidBuf), "%02X%02X%02X%02X",
                     newUid[0], newUid[1], newUid[2], newUid[3]);

            printf("[ADMIN] Card scanned: %s\r\n", uidBuf);
            Buzzer_Beep(100);  /* beep báo đọc được thẻ */

            /* Kiểm tra thẻ đã có trong DS chưa */
            if (WL_FindUser(newUid) >= 0) {
                LCD_Show2Lines("Da co trong DS", uidBuf);
                printf("[ADMIN] Already exists\r\n");
                waitcardRemoval(&rfid);
                vTaskDelay(pdMS_TO_TICKS(1500));
                LCD_Show2Lines("  Admin Mode  ", "Nhan B1 de ra");
                continue;
            }

            /* Thẻ mới */
            LCD_Show2Lines("Nha the ra!   ", uidBuf);
            waitcardRemoval(&rfid);
            vTaskDelay(pdMS_TO_TICKS(200));

            LCD_Show2Lines("BTN2=Them the ", uidBuf);
            printf("[ADMIN] Waiting confirm for: %s\r\n", uidBuf);

            /* Chờ xác nhận BTN2 hoặc huỷ BTN1 (timeout 5s) */
            uint8_t confirmed = 0;
            uint32_t waitStart = HAL_GetTick();
            while ((HAL_GetTick() - waitStart) < 5000) {
                if (BTN2_PRESSED()) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                    if (BTN2_PRESSED()) {
                        confirmed = 1;
                        while (BTN2_PRESSED()) vTaskDelay(pdMS_TO_TICKS(10));
                        break;
                    }
                }
                if (BTN1_PRESSED()) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                    if (BTN1_PRESSED()) {
                        /* Huỷ */
                        while (BTN1_PRESSED()) vTaskDelay(pdMS_TO_TICKS(10));
                        break;
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            if (confirmed) {
                if (WL_AddUser(newUid)) {
                    LCD_Show2Lines("  Them OK!!!  ", uidBuf);
                    printf("[ADMIN] Added: %s\r\n", uidBuf);
                    Buzzer_Beep(100);
                    vTaskDelay(pdMS_TO_TICKS(80));
                    Buzzer_Beep(100);
                } else {
                    LCD_Show2Lines("Them that bai ", uidBuf);
                    Buzzer_Beep(500);
                }
            } else {
                LCD_Show2Lines("  Da huy them ", uidBuf);
                printf("[ADMIN] Cancelled\r\n");
            }

            vTaskDelay(pdMS_TO_TICKS(1500));
            LCD_Show2Lines("  Admin Mode  ", "Nhan B1 de ra");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ============================================================
   SAFETY TASK – Gas + Fire (100ms period)
   ============================================================ */
static void Task_Safety(void *arg)
{
    uint8_t emergencyActive = 0;
    uint8_t ledToggle       = 0;
    uint8_t buzzerToggle    = 0;
    uint8_t buzzerCount     = 0;  // đếm tick để tạo nhịp
    for (;;) {
        uint32_t gasVal  = ADC_ReadMQ2();
        uint8_t  fireVal = HAL_GPIO_ReadPin(ky_026_GPIO_Port, ky_026_Pin);

        uint8_t danger = (gasVal > GAS_THRESHOLD) || (fireVal == GPIO_PIN_RESET);

        if (danger) {
                 if (!emergencyActive) {
                     emergencyActive = 1;
                     emergencyFlag   = 1;
                     Servo1_Open();
                     Servo2_Open();
                     printf("[SAFETY] EMERGENCY! gas=%lu fire=%d\r\n",
                            (unsigned long)gasVal, fireVal);
                     if (gasVal > GAS_THRESHOLD) {
                         Fan_On();   // bật quạt khi có khí gas
                         printf("[SAFETY] Fan ON\r\n");
                     }
                 } else {
                     /* Cập nhật quạt nếu gas xuất hiện sau */
                     if (gasVal > GAS_THRESHOLD) Fan_On();
                 }
            /* Nhịp còi: ON 3 tick (300ms) - OFF 2 tick (200ms) */
                      buzzerCount++;
                      if (buzzerCount <= 1) {
                          Buzzer_On();
                      } else if (buzzerCount <= 2) {
                          Buzzer_Off();
                      } else {
                          buzzerCount = 0;  // reset chu kỳ
                      }

                      /* LED nhấp nháy */
                      ledToggle = !ledToggle;
                      if (ledToggle) LED_On(); else LED_Off();

                      /* Hiện cảnh báo phân biệt gas / lửa / cả 2 */
                                if (xSemaphoreTake(lcdMutex, 0) == pdTRUE) {
                                    CLCD_I2C_Clear(&LCD);
                                    CLCD_I2C_SetCursor(&LCD, 0, 0);

                                    if ((gasVal > GAS_THRESHOLD) && (fireVal == GPIO_PIN_RESET)) {
                                        /* Cả 2 */
                                        CLCD_I2C_WriteString(&LCD, "CANH BAO: GAS");
                                        CLCD_I2C_SetCursor(&LCD, 0, 1);
                                        CLCD_I2C_WriteString(&LCD, "+ LUA! THOAT!");
                                    }
                                    else if (gasVal > GAS_THRESHOLD) {
                                        /* Chỉ gas */
                                        CLCD_I2C_WriteString(&LCD, "CANH BAO: GAS!");
                                        CLCD_I2C_SetCursor(&LCD, 0, 1);
                                        CLCD_I2C_WriteString(&LCD, "Khi doc! Thoat");
                                    }
                                    else {
                                        /* Chỉ lửa */
                                        CLCD_I2C_WriteString(&LCD, "CANH BAO: LUA!");
                                        CLCD_I2C_SetCursor(&LCD, 0, 1);
                                        CLCD_I2C_WriteString(&LCD, "Phat hien lua!");
                                    }

                                    xSemaphoreGive(lcdMutex);
                                }
        } else {
            if (emergencyActive) {
                emergencyActive = 0;
                emergencyFlag   = 0;
                Buzzer_Off();
                LED_Off();
                Fan_Off();
                Servo1_Close();
                Servo2_Close();
                LCD_Show2Lines(" BAI DO XE TM<3  ", "XIN MOI QUET THE");
                printf("[SAFETY] Emergency cleared\r\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ============================================================
   RFID TASK – Main parking logic
   ============================================================ */
static void Task_RFID(void *arg)
{
    uint8_t uid[4];
    DS1307_Time_t now;

    uint32_t adminHoldStart = 0;
    uint8_t  adminHolding   = 0;

    vTaskDelay(pdMS_TO_TICKS(500));
    LCD_Show2Lines(" BAI DO XE TM<3  ", "XIN MOI QUET THE ");

    for (;;) {
        /* Dừng nếu đang khẩn cấp */
        if (emergencyFlag) {
            adminHolding = 0;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        /* ------------------------------------------------
           GIỮ BTN1 + BTN2 CÙNG LÚC 2 GIÂY → VÀO ADMIN
           ------------------------------------------------ */
        if (BTN1_PRESSED() && BTN2_PRESSED()) {
            if (!adminHolding) {
                adminHolding   = 1;
                adminHoldStart = HAL_GetTick();
                LCD_Show2Lines("  Giu de vao  ", " ADMIN mode ");
            } else if ((HAL_GetTick() - adminHoldStart) >= 2000) {
                adminHolding = 0;
                Buzzer_Beep(150);
                vTaskDelay(pdMS_TO_TICKS(80));
                Buzzer_Beep(150);
                vTaskDelay(pdMS_TO_TICKS(200));
                AdminMode();
                LCD_Show2Lines(" BAI DO XE TM<3  ", "XIN MOI QUET THE ");
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        } else {
            if (adminHolding) {
                adminHolding = 0;
                LCD_Show2Lines(" BAI DO XE TM<3  ", "XIN MOI QUET THE ");
            }
        }

        /* ------------------------------------------------
           ĐỌC THẺ RFID
           ------------------------------------------------ */
        if (RFID_GetUID(uid) != STATUS_OK) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        printf("[RFID] Card: %02X %02X %02X %02X\r\n",
               uid[0], uid[1], uid[2], uid[3]);

        /* ---- KIỂM TRA THẺ ADMIN ---- */
        if (UID_Equal(uid, adminCardUID)) {
            printf("[RFID] Admin card detected\r\n");
            LCD_Show2Lines(" The Admin!   ", " Vao Admin... ");
            Buzzer_Beep(150);
            vTaskDelay(pdMS_TO_TICKS(80));
            Buzzer_Beep(150);
            waitcardRemoval(&rfid);
            vTaskDelay(pdMS_TO_TICKS(200));
            AdminMode();
            LCD_Show2Lines(" BAI DO XE TM<3  ", "XIN MOI QUET THE ");
            continue;   /* quay lại vòng lặp, không xử lý whitelist */
        }

        /* Kiểm tra whitelist */
        int idx = WL_FindUser(uid);
        if (idx < 0) {
            LCD_Show2Lines("   The Sai!   ", "Khong hop le");
            printf("[RFID] Unauthorised\r\n");
            Buzzer_Beep(100); vTaskDelay(pdMS_TO_TICKS(50));
            Buzzer_Beep(100); vTaskDelay(pdMS_TO_TICKS(50));
            Buzzer_Beep(100);
            waitcardRemoval(&rfid);
            vTaskDelay(pdMS_TO_TICKS(1500));
            LCD_Show2Lines(" BAI DO XE TM<3  ", "XIN MOI QUET THE ");
            continue;
        }

        UserRecord_t *user = &wlStore.users[idx];
        DS1307_GetTime(&now);

        if (user->state == CAR_OUT) {
            /* ---- XE VÀO ---- */
            user->state     = CAR_IN;
            user->entryHour = now.hour;
            user->entryMin  = now.min;

            Servo1_Open();
            Buzzer_Beep(200);

            char buf[17];
            char l1[17];
                     snprintf(l1, sizeof(l1), "%02d/%02d  %02d:%02d",
                              now.date, now.month, now.hour, now.min);
                     LCD_Show2Lines("  Xe vao bai  ", l1);
            printf("[RFID] Car IN at %02d:%02d\r\n", now.hour, now.min);
            Flash_WriteWhitelist();
            vTaskDelay(pdMS_TO_TICKS(2000));
            Servo1_Close();

        } else {
            /* ---- XE RA ---- */
            user->state  = CAR_OUT;
            uint32_t fee = CalcFee(user->entryHour, user->entryMin,
                                   now.hour, now.min);
            Servo2_Open();
            Buzzer_Beep(200);

            char l1[17], l2[17];
            snprintf(l1, sizeof(l1), "V:%02d:%02d R:%02d:%02d",
                                 user->entryHour, user->entryMin,
                                 now.hour, now.min);
            snprintf(l2, sizeof(l2), "Phi:%5lu VND", (unsigned long)fee);
            LCD_Show2Lines(l1, l2);
            printf("[RFID] Car OUT In=%02d:%02d Out=%02d:%02d Fee=%lu VND\r\n",
                   user->entryHour, user->entryMin,
                   now.hour, now.min, (unsigned long)fee);
            Flash_WriteWhitelist();
            vTaskDelay(pdMS_TO_TICKS(3000));
            Servo2_Close();
        }

        waitcardRemoval(&rfid);
        vTaskDelay(pdMS_TO_TICKS(500));
        LCD_Show2Lines(" BAI DO XE TM<3  ", "XIN MOI QUET THE ");
    }
}

/* USER CODE END 0 */

/* ============================================================
   MAIN
   ============================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();

    /* USER CODE BEGIN 2 */

    /* Servo PWM */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    Servo1_Close();
    Servo2_Close();

    /* LCD */
    CLCD_I2C_Init(&LCD, &hi2c1, LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

    /* RTC */
    DS1307_Init();
    DS1307_SetTime(2, 44, 0, 20, 4, 26);
    /* RFID */
    rfid.hspi    = &hspi1;
    rfid.csPort  = cs_GPIO_Port;
    rfid.csPin   = cs_Pin;
    rfid.rstPort = rst_GPIO_Port;
    rfid.rstPin  = rst_Pin;
    MFRC522_Init(&rfid);

    /* Whitelist từ Flash */
    Flash_ReadWhitelist();

    printf("[MAIN] Smart Parking System started\r\n");

    /* USER CODE END 2 */

    /* ---- FreeRTOS: Mutex LCD ---- */
    /* QUAN TRỌNG: phải tạo mutex TRƯỚC khi tạo task */
    lcdMutex = xSemaphoreCreateMutex();
    if (lcdMutex == NULL) {
        printf("[ERROR] LCD mutex creation failed!\r\n");
        Error_Handler();
    }

    /* ---- FreeRTOS: Tasks ---- */

    /* Default task – monitor heap */
    xTaskCreate(StartDefaultTask, "defaultTask", 256, NULL, tskIDLE_PRIORITY + 1, &defaultTaskHandle);

    /* Safety task – cao hơn để preempt RFID khi khẩn cấp */
    xTaskCreate(Task_Safety, "safetyTask", 384, NULL, tskIDLE_PRIORITY + 2, NULL);

    /* RFID/parking task – stack lớn hơn vì dùng snprintf + AdminMode */
    xTaskCreate(Task_RFID, "rfidTask", 768, NULL, tskIDLE_PRIORITY + 1, NULL);

    /* Start FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Không bao giờ đến đây */
    while (1) {}
}

/* ============================================================
   DEFAULT TASK – Monitor heap (debug)
   ============================================================ */
void StartDefaultTask(void *argument)
{
    for (;;) {
        size_t freeHeap = xPortGetFreeHeapSize();
        printf("[HEAP] Free: %u bytes\r\n", (unsigned)freeHeap);
        vTaskDelay(pdMS_TO_TICKS(10000));   /* in mỗi 10 giây */
    }
}

/* ============================================================
   PERIPHERAL INIT (CubeMX generated)
   ============================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct   = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct   = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState        = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState        = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState    = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL      = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_0;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};
    TIM_OC_InitTypeDef      sConfigOC          = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 799;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 1799;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim2);
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(cs_GPIO_Port, cs_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, rst_Pin | buzzer_Pin | led_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        GPIO_InitStruct.Pin   = GPIO_PIN_12;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = cs_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(cs_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = rst_Pin | buzzer_Pin | led_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = ky_026_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(ky_026_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = nut_bam_2_Pin | nut_bam_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("Assert: %s line %lu\r\n", file, (unsigned long)line);
}
#endif
