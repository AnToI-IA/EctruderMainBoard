#include "main.h"

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* Параметры двигателей */
typedef struct {
    uint32_t current_speed;   // Текущая скорость (Гц)
    uint32_t target_speed;    // Установленная скорость (Гц)
    uint32_t start_speed;     // Начальная скорость (Гц)
    uint32_t acceleration;    // Ускорение (Гц/сек)
    uint32_t min_speed;       // Минимальная скорость (Гц)
} MotorParams;

MotorParams motor_x = {0, 100, 100, 50, 50}; // Двигатель X
MotorParams motor_y = {0, 500, 200, 50, 50};  // Двигатель Y
MotorParams motor_z = {0, 1500, 800, 200, 50}; // Двигатель Z

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
void UpdateMotorSpeed(MotorParams *motor, TIM_HandleTypeDef *htim, uint32_t channel);

/* Main program --------------------------------------------------------------*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();

    // Запуск PWM для всех двигателей
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // Двигатель X
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Двигатель Y
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Двигатель Z

    // Установка начального направления
    HAL_GPIO_WritePin(GPIOB, DIR_X_Pin, GPIO_PIN_SET);  // DIR_X (прямое направление)
    HAL_GPIO_WritePin(GPIOB, DIR_Y_Pin, GPIO_PIN_RESET); // DIR_Y (обратное направление)
    HAL_GPIO_WritePin(GPIOA, DIR_Z_Pin, GPIO_PIN_SET);  // DIR_Z (прямое направление)

    // Установка начальных скоростей
    motor_x.current_speed = motor_x.start_speed;
    motor_y.current_speed = motor_y.start_speed;
    motor_z.current_speed = motor_z.start_speed;

    // Установка выхода DIR_EN в HIGH (включение драйвера)
    HAL_GPIO_WritePin(GPIOA, EN__Pin, GPIO_PIN_SET); // EN (включение драйвера)

    while (1)
    {
        // Обновление скоростей двигателей
        UpdateMotorSpeed(&motor_x, &htim1, TIM_CHANNEL_3);
        UpdateMotorSpeed(&motor_y, &htim2, TIM_CHANNEL_2);
        UpdateMotorSpeed(&motor_z, &htim3, TIM_CHANNEL_2);

        // Проверка кнопки B1
        if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) // Кнопка нажата
        {
            uint32_t press_start = HAL_GetTick(); // Запоминаем время начала нажатия
            HAL_Delay(25); // Антидребезг

            while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
            {
                if (HAL_GetTick() - press_start > 500) // Долгое нажатие (> 1 сек)
                {
                    // Резрвируем текущую скорость
                    motor_x.start_speed = motor_x.current_speed;
                    // Замедление двигателя X до минимальной скорости
                    motor_x.target_speed = motor_x.min_speed;
                    while (motor_x.current_speed > motor_x.min_speed)
                    {
                        UpdateMotorSpeed(&motor_x, &htim1, TIM_CHANNEL_3);
                        HAL_Delay(10); // Задержка для плавного замедления
                    }

                    // Меняем направление вращения двигателя X
                    HAL_GPIO_TogglePin(GPIOB, DIR_X_Pin);

                    // Переключение светодиода LD2
                    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

                    // Разгон двигателя X до текущей скорости
                    motor_x.target_speed = motor_x.start_speed;
                    while (motor_x.current_speed < motor_x.target_speed)
                    {
                        UpdateMotorSpeed(&motor_x, &htim1, TIM_CHANNEL_3);
                        HAL_Delay(10); // Задержка для плавного разгона
                    }

                    // Ждем отпускания кнопки
                    while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET);
                    break;
                }
            }

            if (HAL_GetTick() - press_start <= 500) // Короткое нажатие
            {
                // Увеличение скорости двигателя X
                motor_x.target_speed += 250;
                if (motor_x.target_speed > 100000) // Ограничение до 100 кГц
                {
                    motor_x.target_speed = 1000;
                }

                // Переключение светодиода LD2
                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            }
        }

        HAL_Delay(5); // Задержка для плавного изменения скорости
    }
}

/* Функция обновления скорости двигателя */
void UpdateMotorSpeed(MotorParams *motor, TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (motor->current_speed < motor->target_speed)
    {
        // Увеличение текущей скорости с учетом ускорения
        motor->current_speed += motor->acceleration; // Ускорение на 100 мс
        if (motor->current_speed > motor->target_speed)
        {
            motor->current_speed = motor->target_speed;
        }
    }
    else if (motor->current_speed > motor->target_speed)
    {
        // Уменьшение текущей скорости с учетом ускорения
        if (motor->current_speed <= motor->acceleration )
        motor->current_speed = motor->min_speed; // Установим минимальную скорость
        else motor->current_speed -= motor->acceleration; // Ускорение на 100 мс
        if (motor->current_speed < motor->target_speed)
        {
            motor->current_speed = motor->target_speed;
        }
    }

    // Обновление периода таймера для изменения частоты
    if (motor->current_speed > 0)
    {
        uint32_t period = 1000000 / motor->current_speed; // Период в микросекундах
        
        __HAL_TIM_SET_COMPARE(htim, channel, period / 2); // 50% PWM
        __HAL_TIM_SET_AUTORELOAD(htim, period - 1);
    }
    else
    {
        // Остановка двигателя
        __HAL_TIM_SET_COMPARE(htim, channel, 0);
    }
}

/* TIM1 Initialization Function (Двигатель X) */
static void MX_TIM1_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 72 - 1; // Предделитель для 1 МГц
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 1000 - 1; // Частота PWM = 1 кГц
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim1);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500; // 50% PWM
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);

    HAL_TIM_MspPostInit(&htim1);
}

/* TIM2 Initialization Function (Двигатель Y) */
static void MX_TIM2_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 72 - 1; // Предделитель для 1 МГц
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 2000 - 1; // Частота PWM = 500 Гц
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim2);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1000; // 50% PWM
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

    HAL_TIM_MspPostInit(&htim2);
}

/* TIM3 Initialization Function (Двигатель Z) */
static void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 72 - 1; // Предделитель для 1 МГц
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 1500 - 1; // Частота PWM = 666 Гц
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim3);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 750; // 50% PWM
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

    HAL_TIM_MspPostInit(&htim3);
}

/* UART2 Initialization Function */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        while (1); // Обработка ошибки
    }
}

/* GPIO Initialization Function */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure GPIO pin for LD2 */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    /* Configure GPIO pins for direction control */
    GPIO_InitStruct.Pin = DIR_X_Pin | DIR_Y_Pin; // DIR_X, DIR_Y
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DIR_Z_Pin; // DIR_Z
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure GPIO pin for DIR_EN */
    GPIO_InitStruct.Pin = EN__Pin; // EN (включение драйвера)
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure GPIO pin for B1 */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    // Настройка HSE (внешний кварц 8 МГц)
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON; // RTC на 32.768 кГц
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; // Умножение на 9 для получения 72 МГц
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        while (1); // Обработка ошибки
    }

    // Настройка шин (AHB, APB1, APB2)
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; // APB1 = 36 МГц
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; // APB2 = 72 МГц
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        while (1); // Обработка ошибки
    }

    // Настройка RTC
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        while (1); // Обработка ошибки
    }
    __HAL_RCC_RTC_ENABLE();
}