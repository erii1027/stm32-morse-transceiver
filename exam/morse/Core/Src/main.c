/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 摩斯電碼收發器 (NUCLEO-G474RE, STM32CubeMX + HAL)
  *
  *  PC8 : 摩斯按鍵 (按下為低電位)
  *  PC9 : 發送按鈕 (按下為低電位)
  *  PC6 : LED
  *  LPUART1 (PA2/PA3) : ST-LINK 虛擬串口 115200 8N1
  *  TIM6 : 1ms 中斷 -> ms_tick
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
TIM_HandleTypeDef  htim6;

/* USER CODE BEGIN PTD */
/* ---- 按鍵事件 ---- */
typedef enum { BTN_NONE = 0, BTN_PRESS, BTN_RELEASE } BtnEvt_t;

/* ---- 每個按鍵獨立的防彈跳資料 ---- */
typedef struct
{
    /* 以下三個欄位會被 ISR 與主迴圈共用 */
    volatile uint8_t  pending;      /* ISR 設 1:有邊緣尚待確認                   */
    volatile uint32_t first_tick;   /* 這一串邊緣中「第一個」邊緣的時間            */
    volatile uint32_t last_tick;    /* 這一串邊緣中「最後一個」邊緣的時間          */
    /* 以下只有主迴圈使用 */
    uint8_t           stable_pressed; /* 已確認的穩定狀態 (1 = 按下)              */
} Btn_t;

/* ---- LED 播放事件表 ---- */
typedef struct
{
    uint8_t  on;    /* 1 = LED 亮, 0 = LED 滅 */
    uint16_t ms;    /* 持續時間 (ms)          */
} LedEvent_t;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
/* ============================ 時間參數 (集中放) ============================ */
#define DEBOUNCE_MS         20U     /* 防彈跳時間                                */
#define DASH_MS             250U    /* 按壓 >= 此值為「劃」, 否則為「點」        */
#define LETTER_GAP_MS       600U    /* 放開超過此時間 -> 一個字母結束            */
#define WORD_GAP_MS         1500U   /* 放開超過此時間 -> 插入空格                */
#define UNIT_MS             200U    /* LED 播放的 1 個單位                       */

/* LED 播放時間 (以單位數表示) */
#define DOT_UNITS           1U
#define DASH_UNITS          3U
#define SYMBOL_GAP_UNITS    1U      /* 符號間隔 */
#define LETTER_GAP_UNITS    3U      /* 字母間隔 */
#define WORD_GAP_UNITS      7U      /* 單字間隔 */

/* ============================ 其他設定 ============================ */
#define MSG_MAX_LEN         64U     /* msg 緩衝最大字元數                        */
#define MAX_SYMS            7U      /* 單一字母最多記錄的符號數 (超過視為 '?')   */
#define RX_LINE_MAX         64U     /* UART 一行文字最大長度                     */
#define RX_RING_SIZE        128U    /* UART 接收環形緩衝大小                     */
#define EVT_MAX             700U    /* 事件表容量 (64 字 x 每字最多 10 事件)     */

/* ============================ 腳位定義 ============================ */
#define MORSE_LED_PORT      GPIOC
#define MORSE_LED_PIN       GPIO_PIN_6
#define KEY_MORSE_PORT      GPIOC
#define KEY_MORSE_PIN       GPIO_PIN_8
#define KEY_SEND_PORT       GPIOC
#define KEY_SEND_PIN        GPIO_PIN_9

#define LED_ON()            HAL_GPIO_WritePin(MORSE_LED_PORT, MORSE_LED_PIN, GPIO_PIN_SET)
#define LED_OFF()           HAL_GPIO_WritePin(MORSE_LED_PORT, MORSE_LED_PIN, GPIO_PIN_RESET)
/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM6_Init(void);

/* USER CODE BEGIN PV */
/* ---------------------------- ISR 共用資料 ---------------------------- */
volatile uint32_t ms_tick = 0;                 /* 全域毫秒計數 (TIM6 每 1ms +1)  */

static Btn_t btn_morse;                        /* PC8 */
static Btn_t btn_send;                         /* PC9 */

static uint8_t           rx_byte;              /* HAL_UART_Receive_IT 的 1 byte  */
static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;          /* ISR 寫入                       */
static volatile uint16_t rx_tail = 0;          /* 主迴圈讀取                     */
static volatile uint8_t  uart_err_flag = 0;    /* UART 錯誤旗標 (ISR 設, 主迴圈印) */

/* ---------------------------- 只有主迴圈使用 ---------------------------- */
/* 摩斯輸入 */
static uint8_t  key_down      = 0;             /* PC8 目前是否按著               */
static uint32_t press_tick    = 0;             /* 按下的時間                     */
static uint32_t release_tick  = 0;             /* 放開的時間                     */
static uint8_t  have_release  = 0;             /* 1 = 放開後的計時有效           */
static char     sym_buf[MAX_SYMS + 1];         /* 目前累積的 '.' '-'             */
static uint8_t  sym_len       = 0;
static uint8_t  sym_overflow  = 0;
static char     msg[MSG_MAX_LEN + 1];          /* 已解碼的訊息                   */
static uint8_t  msg_len       = 0;

/* LED 播放 */
static LedEvent_t ev_tbl[EVT_MAX];
static uint16_t   ev_count    = 0;
static uint16_t   ev_idx      = 0;
static uint32_t   ev_deadline = 0;
static uint8_t    playing     = 0;

/* UART 行組合 */
static char     rx_line[RX_LINE_MAX + 1];
static uint8_t  rx_line_len   = 0;

/* 摩斯電碼表 (A-Z, 0-9) */
static const struct { char ch; const char *code; } morse_table[] =
{
    {'A', ".-"},    {'B', "-..."},  {'C', "-.-."},  {'D', "-.."},   {'E', "."},
    {'F', "..-."},  {'G', "--."},   {'H', "...."},  {'I', ".."},    {'J', ".---"},
    {'K', "-.-"},   {'L', ".-.."},  {'M', "--"},    {'N', "-."},    {'O', "---"},
    {'P', ".--."},  {'Q', "--.-"},  {'R', ".-."},   {'S', "..."},   {'T', "-"},
    {'U', "..-"},   {'V', "...-"},  {'W', ".--"},   {'X', "-..-"},  {'Y', "-.--"},
    {'Z', "--.."},
    {'0', "-----"}, {'1', ".----"}, {'2', "..---"}, {'3', "...--"}, {'4', "....-"},
    {'5', "....."}, {'6', "-...."}, {'7', "--..."}, {'8', "---.."}, {'9', "----."}
};
#define MORSE_TABLE_SIZE  (sizeof(morse_table) / sizeof(morse_table[0]))
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
void Morse_Init(void);
void Morse_Process(void);
/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_LPUART1_UART_Init();
    MX_TIM6_Init();

    /* USER CODE BEGIN 2 */
    Morse_Init();
    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN 3 */
        Morse_Process();
        /* USER CODE END 3 */
    }
}

/**
  * @brief System Clock Configuration
  *        HSI 16MHz -> PLL (M=4, N=85, P/Q/R=2) = 170MHz
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* 電壓 Scale1 Boost */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN            = 85;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief LPUART1 Initialization: 115200, 8N1, 無流控, 關閉 FIFO
  */
static void MX_LPUART1_UART_Init(void)
{
    hlpuart1.Instance                    = LPUART1;
    hlpuart1.Init.BaudRate               = 115200;
    hlpuart1.Init.WordLength             = UART_WORDLENGTH_8B;
    hlpuart1.Init.StopBits               = UART_STOPBITS_1;
    hlpuart1.Init.Parity                 = UART_PARITY_NONE;
    hlpuart1.Init.Mode                   = UART_MODE_TX_RX;
    hlpuart1.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    hlpuart1.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    hlpuart1.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&hlpuart1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)    /* 關閉 FIFO */
    {
        Error_Handler();
    }
}

/**
  * @brief TIM6 Initialization: 170MHz / 170 / 1000 = 1kHz (1ms)
  */
static void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = 169;
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = 999;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization
  *        PC6 : 推挽輸出 (LED)
  *        PC8 / PC9 : 雙緣觸發外部中斷 + 內部上拉
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = GPIO_PIN_6;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* EXTI9_5 中斷優先權 0 */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */

/* ========================================================================== */
/*                               工具函式                                      */
/* ========================================================================== */

/* 以關中斷保護的方式讀取 ms_tick */
static uint32_t Tick_Now(void)
{
    uint32_t t;
    __disable_irq();
    t = ms_tick;
    __enable_irq();
    return t;
}

/* UART 輸出字串 (阻塞式; 絕對不可在 ISR 內呼叫) */
static void uart_print(const char *s)
{
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)s, (uint16_t)strlen(s), 200);
}

/* 格式化輸出 (絕對不可在 ISR 內呼叫) */
static void uart_printf(const char *fmt, ...)
{
    char    buf[128];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uart_print(buf);
}

/* ========================================================================== */
/*                          摩斯編碼 / 解碼                                    */
/* ========================================================================== */

/* 符號字串 (如 ".-") -> 字元, 查不到回傳 '?' */
static char Morse_Decode(const char *code)
{
    for (uint32_t i = 0; i < MORSE_TABLE_SIZE; i++)
    {
        if (strcmp(morse_table[i].code, code) == 0)
        {
            return morse_table[i].ch;
        }
    }
    return '?';
}

/* 字元 -> 符號字串, 不支援的字元回傳 NULL (小寫自動轉大寫) */
static const char *Morse_Encode(char c)
{
    if (c >= 'a' && c <= 'z')
    {
        c = (char)(c - 'a' + 'A');
    }
    for (uint32_t i = 0; i < MORSE_TABLE_SIZE; i++)
    {
        if (morse_table[i].ch == c)
        {
            return morse_table[i].code;
        }
    }
    return NULL;
}

/* ========================================================================== */
/*                       LED 播放 (非阻塞, 事件表)                              */
/* ========================================================================== */

/* 在事件表尾端加入一個事件 */
static void Ev_Add(uint8_t on, uint16_t units)
{
    if (ev_count < EVT_MAX)
    {
        ev_tbl[ev_count].on = on;
        ev_tbl[ev_count].ms = (uint16_t)(units * UNIT_MS);
        ev_count++;
    }
}

/* 把文字轉成 LED 事件表
 * 規則: 點 1、劃 3、符號間隔 1、字母間隔 3、單字間隔 7 (單位)
 * 不支援的字元略過; 前後的空格略過; 連續空格視為一個單字間隔 */
static void Morse_BuildEvents(const char *text)
{
    uint8_t first_char = 1;
    uint8_t gap_units  = LETTER_GAP_UNITS;

    ev_count = 0;
    for (const char *p = text; *p != '\0'; p++)
    {
        if (*p == ' ')
        {
            if (!first_char)
            {
                gap_units = WORD_GAP_UNITS;
            }
            continue;
        }

        const char *code = Morse_Encode(*p);
        if (code == NULL)
        {
            continue;
        }

        if (!first_char)
        {
            Ev_Add(0, gap_units);               /* 字母間隔或單字間隔 */
        }
        first_char = 0;
        gap_units  = LETTER_GAP_UNITS;

        for (const char *q = code; *q != '\0'; q++)
        {
            if (q != code)
            {
                Ev_Add(0, SYMBOL_GAP_UNITS);    /* 符號間隔 */
            }
            Ev_Add(1, (*q == '.') ? DOT_UNITS : DASH_UNITS);
        }
    }
}

/* 開始播放 (呼叫前須確認 playing == 0) */
static void Play_Start(const char *text)
{
    Morse_BuildEvents(text);
    if (ev_count == 0)
    {
        uart_print("[LED] nothing to play\r\n");
        return;
    }

    ev_idx = 0;
    if (ev_tbl[0].on) { LED_ON(); } else { LED_OFF(); }
    ev_deadline  = Tick_Now() + ev_tbl[0].ms;
    have_release = 0;           /* 播放期間不做輸入計時 */
    playing      = 1;
}

/* 在主迴圈中推進播放狀態 */
static void Play_Process(void)
{
    if (!playing)
    {
        return;
    }
    if ((int32_t)(Tick_Now() - ev_deadline) < 0)    /* 還沒到時間 */
    {
        return;
    }

    ev_idx++;
    if (ev_idx >= ev_count)                         /* 全部播完 */
    {
        LED_OFF();
        playing = 0;
        uart_print("[LED] done\r\n");
        return;
    }

    if (ev_tbl[ev_idx].on) { LED_ON(); } else { LED_OFF(); }
    ev_deadline += ev_tbl[ev_idx].ms;               /* 用累加避免時間漂移 */
}

/* ========================================================================== */
/*                         按鍵 (ISR 端 + 主迴圈端)                            */
/* ========================================================================== */

/* ISR 端: 只記錄「有邊緣」與時間, 不做任何耗時動作 */
static void Btn_Edge(Btn_t *b)
{
    uint32_t now = ms_tick;
    if (!b->pending)
    {
        b->first_tick = now;        /* 這串邊緣的第一個時間 (按壓時間以此計算) */
        b->pending    = 1;
    }
    b->last_tick = now;             /* 每個邊緣都更新, 用來判斷彈跳是否已平息 */
}

/* 主迴圈端: 邊緣平息超過 DEBOUNCE_MS 後, 讀取腳位確認真正狀態
 * 回傳 BTN_PRESS / BTN_RELEASE 時, *t_out 為該動作的起始時間 */
static BtnEvt_t Btn_Poll(Btn_t *b, GPIO_TypeDef *port, uint16_t pin, uint32_t *t_out)
{
    uint8_t  pend;
    uint32_t first, last;

    __disable_irq();
    pend  = b->pending;
    first = b->first_tick;
    last  = b->last_tick;
    __enable_irq();

    if (!pend)
    {
        return BTN_NONE;
    }
    if ((uint32_t)(Tick_Now() - last) < DEBOUNCE_MS)    /* 還在彈跳中 */
    {
        return BTN_NONE;
    }

    __disable_irq();
    if (b->last_tick != last)       /* 剛好又有新邊緣進來, 下一輪再處理 */
    {
        __enable_irq();
        return BTN_NONE;
    }
    b->pending = 0;
    __enable_irq();

    uint8_t pressed = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1 : 0;
    if (pressed == b->stable_pressed)   /* 只是雜訊, 狀態沒變 */
    {
        return BTN_NONE;
    }

    b->stable_pressed = pressed;
    *t_out = first;
    return pressed ? BTN_PRESS : BTN_RELEASE;
}

/* 下降緣 = 按下 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_MORSE_PIN)      { Btn_Edge(&btn_morse); }
    else if (GPIO_Pin == KEY_SEND_PIN)  { Btn_Edge(&btn_send);  }
}

/* 上升緣 = 放開 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_MORSE_PIN)      { Btn_Edge(&btn_morse); }
    else if (GPIO_Pin == KEY_SEND_PIN)  { Btn_Edge(&btn_send);  }
}

/* 相容舊版 CubeG4 韌體包: 舊版 HAL_GPIO_EXTI_IRQHandler 只會呼叫這個通用回呼,
 * 不會呼叫上面的 Rising / Falling 版本 (新版則相反, 兩者不會同時被呼叫) */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_MORSE_PIN)      { Btn_Edge(&btn_morse); }
    else if (GPIO_Pin == KEY_SEND_PIN)  { Btn_Edge(&btn_send);  }
}

/* ========================================================================== */
/*                           TIM6 / UART 回呼 (ISR)                            */
/* ========================================================================== */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        ms_tick++;
    }
    /* 若 HAL 時基仍使用 SysTick, HAL_IncTick 由 SysTick_Handler 處理, 這裡不需動作 */
}

/* 收到 1 byte: 放進環形緩衝, 並重新啟動接收 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {
        uint16_t next = (uint16_t)((rx_head + 1U) % RX_RING_SIZE);
        if (next != rx_tail)                /* 緩衝未滿才存, 滿了就丟棄 */
        {
            rx_ring[rx_head] = rx_byte;
            rx_head = next;
        }
        HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);
    }
}

/* UART 錯誤 (如 Overrun): 清旗標並重新開啟接收 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                     UART_CLEAR_PEF  | UART_CLEAR_FEF);
        HAL_UART_AbortReceive(huart);               /* 確保接收狀態回到 READY */
        HAL_UART_Receive_IT(huart, &rx_byte, 1);    /* 重新開啟接收 */
        uart_err_flag = 1;                          /* 讓主迴圈印出訊息 */
    }
}

/* ========================================================================== */
/*                              主迴圈邏輯                                     */
/* ========================================================================== */

/* 是否正在輸入中: 按鍵按著, 或字母尚未結束 */
static uint8_t Input_Busy(void)
{
    return (key_down || sym_len > 0) ? 1 : 0;
}

/* 從環形緩衝取出 1 byte (關中斷保護), 無資料回傳 0 */
static uint8_t Rx_Pop(uint8_t *b)
{
    __disable_irq();
    if (rx_head == rx_tail)
    {
        __enable_irq();
        return 0;
    }
    *b = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % RX_RING_SIZE);
    __enable_irq();
    return 1;
}

/* 收到完整一行文字 */
static void Rx_HandleLine(const char *line)
{
    uart_printf("[RX] %s\r\n", line);

    /* 播放中或輸入中不接受新的播放要求 (避免干擾) */
    if (playing || Input_Busy())
    {
        uart_print("busy\r\n");
        return;
    }
    Play_Start(line);
}

/* 處理 UART 收到的資料: 組成一行 (\r 或 \n 結尾) */
static void Rx_Process(void)
{
    uint8_t b;

    if (uart_err_flag)
    {
        uart_err_flag = 0;
        uart_print("[UART] error, rx restarted\r\n");
    }

    while (Rx_Pop(&b))
    {
        if (b == '\r' || b == '\n')
        {
            if (rx_line_len > 0)            /* 空行 (例如 \r\n 的第二個字元) 忽略 */
            {
                rx_line[rx_line_len] = '\0';
                rx_line_len = 0;
                Rx_HandleLine(rx_line);
            }
        }
        else if (b >= 0x20 && b < 0x7F)     /* 只收可顯示字元; 超過長度的字元丟棄 */
        {
            if (rx_line_len < RX_LINE_MAX)
            {
                rx_line[rx_line_len++] = (char)b;
            }
        }
    }
}

/* 把目前累積的符號解碼, 加進 msg */
static void Letter_Commit(void)
{
    char c;

    sym_buf[sym_len] = '\0';
    c = sym_overflow ? '?' : Morse_Decode(sym_buf);

    if (msg_len < MSG_MAX_LEN)
    {
        msg[msg_len++] = c;
        msg[msg_len]   = '\0';
        uart_printf("letter: %s -> %c | msg: %s\r\n", sym_buf, c, msg);
    }
    else
    {
        uart_printf("letter: %s -> %c | msg full (%u), dropped\r\n",
                    sym_buf, c, (unsigned)MSG_MAX_LEN);
    }

    sym_len      = 0;
    sym_overflow = 0;
}

/* PC9 發送按鈕被按下 */
static void Send_Handle(void)
{
    if (playing || Input_Busy())
    {
        uart_print("busy\r\n");
        return;
    }
    if (msg_len == 0)
    {
        uart_print("(empty)\r\n");
        return;
    }

    /* 去掉尾端空格後再印出 / 播放 */
    while (msg_len > 0 && msg[msg_len - 1] == ' ')
    {
        msg_len--;
    }
    msg[msg_len] = '\0';

    if (msg_len == 0)
    {
        uart_print("(empty)\r\n");
        return;
    }

    uart_printf("[MSG] %s\r\n", msg);
    Play_Start(msg);                /* 事件表已複製完成, 之後即可清空 msg */

    msg_len = 0;
    msg[0]  = '\0';
}

/* PC8 摩斯按鍵事件處理 */
static void Key_Handle(BtnEvt_t evt, uint32_t t)
{
    if (playing)                    /* 播放中不接受按鍵輸入 */
    {
        return;
    }

    if (evt == BTN_PRESS)
    {
        key_down   = 1;
        press_tick = t;
        LED_ON();
    }
    else if (evt == BTN_RELEASE && key_down)
    {
        uint32_t dur = t - press_tick;
        char     sym = (dur >= DASH_MS) ? '-' : '.';

        key_down = 0;
        LED_OFF();

        if (sym_len < MAX_SYMS)
        {
            sym_buf[sym_len++] = sym;
        }
        else
        {
            sym_overflow = 1;       /* 符號太多, 最後解碼為 '?' */
        }

        uart_printf("press %lu ms -> %c\r\n", (unsigned long)dur, sym);

        release_tick = t;
        have_release = 1;
    }
}

/* 放開後的逾時判斷: 字母結束 (600ms) 與單字空格 (1500ms) */
static void Gap_Process(void)
{
    if (playing || key_down || !have_release)
    {
        return;
    }

    uint32_t idle = Tick_Now() - release_tick;

    if (sym_len > 0 && idle >= LETTER_GAP_MS)
    {
        Letter_Commit();
    }

    if (sym_len == 0 && idle >= WORD_GAP_MS)
    {
        /* 只在 msg 非空且尾端不是空格時才插入空格 */
        if (msg_len > 0 && msg_len < MSG_MAX_LEN && msg[msg_len - 1] != ' ')
        {
            msg[msg_len++] = ' ';
            msg[msg_len]   = '\0';
            uart_printf("space | msg: %s\r\n", msg);
        }
        have_release = 0;           /* 這次放開的計時結束 */
    }
}

/* ========================================================================== */
/*                           對外介面 (main 只呼叫這兩個)                        */
/* ========================================================================== */

void Morse_Init(void)
{
    /* 狀態初始化 */
    msg[0] = '\0';
    LED_OFF();

    btn_morse.pending        = 0;
    btn_morse.stable_pressed = (HAL_GPIO_ReadPin(KEY_MORSE_PORT, KEY_MORSE_PIN) == GPIO_PIN_RESET);
    btn_send.pending         = 0;
    btn_send.stable_pressed  = (HAL_GPIO_ReadPin(KEY_SEND_PORT, KEY_SEND_PIN) == GPIO_PIN_RESET);

    /* 啟動 1ms 計時中斷與 UART 中斷接收 */
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);

    /* 開機訊息與操作說明 */
    uart_print("\r\n=== Morse transceiver ready ===\r\n");
    uart_print("PC8 key : press < 250ms = '.', >= 250ms = '-'\r\n");
    uart_print("          release > 600ms = letter end, > 1500ms = space\r\n");
    uart_print("PC9 btn : print msg and blink it on LED, then clear msg\r\n");
    uart_print("UART    : type a line + Enter -> print [RX] and blink it\r\n");
}

void Morse_Process(void)
{
    BtnEvt_t evt;
    uint32_t t = 0;

    /* 1. UART 收到的資料 */
    Rx_Process();

    /* 2. PC8 摩斯按鍵 (獨立 20ms 防彈跳) */
    evt = Btn_Poll(&btn_morse, KEY_MORSE_PORT, KEY_MORSE_PIN, &t);
    if (evt != BTN_NONE)
    {
        Key_Handle(evt, t);
    }

    /* 3. PC9 發送按鈕 (獨立 20ms 防彈跳), 只處理「按下」 */
    evt = Btn_Poll(&btn_send, KEY_SEND_PORT, KEY_SEND_PIN, &t);
    if (evt == BTN_PRESS)
    {
        Send_Handle();
    }

    /* 4. LED 非阻塞播放 */
    Play_Process();

    /* 5. 字母結束 / 空格 逾時判斷 */
    Gap_Process();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif /* USE_FULL_ASSERT */
