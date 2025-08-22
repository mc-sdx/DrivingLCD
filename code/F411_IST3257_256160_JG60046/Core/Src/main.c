/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for USB Video Player
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdarg.h>
#include <string.h> // For memset
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// --- 视频播放参数和缓冲区定义 ---
#define IST3257_WIDTH       256
#define IST3257_HEIGHT      162
#define FRAME_SIZE_BYTES    (IST3257_WIDTH * IST3257_HEIGHT / 2)
#define IMAGE_SIZE_BYTES    (256 * 160 / 2) // PC发送的帧大小

// 用于显示的双缓冲区
static uint8_t framebuffer1[FRAME_SIZE_BYTES];
static uint8_t framebuffer2[FRAME_SIZE_BYTES];

// 指向当前“后台”缓冲区的指针（CPU/USB正在写入的缓冲区）
static uint8_t *back_buffer_ptr;
// 指向当前“前台”缓冲区的指针（DMA正在发送的缓冲区）
static uint8_t *front_buffer_ptr;

// USB接收状态
volatile uint8_t new_frame_received_flag = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SdCmd(uint16_t Command);
void SdData(uint16_t DData);
void IST3257_UpdateScreen_DMA(uint8_t* buffer_to_send);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint8_t spi_dma_busy = 0;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
        spi_dma_busy = 0;
    }
}

#define ACK_CHAR 'A' // Acknowledge character
#define NACK_CHAR 'N' // Not Acknowledge character
// --- USB CDC 接收处理 ---
// 这个回调函数在 usbd_cdc_if.c 中被 __weak 定义
// 我们在这里提供强定义来接管USB数据接收
extern USBD_HandleTypeDef hUsbDeviceFS;
static uint32_t received_count = 0;

void CDC_ReceiveCallBack(uint8_t* Buf, uint32_t Len)
{
    // 将接收到的数据包拷贝到后台缓冲区的正确位置
    if ((received_count + Len) <= IMAGE_SIZE_BYTES)
    {
        memcpy(back_buffer_ptr + received_count, Buf, Len);
        received_count += Len;
    }
    // 如果收到的数据包会导致溢出，则忽略这个包（或者进行其他错误处理）
    // 这可以增加系统的鲁棒性
    else {
        // 可选：在这里处理错误，例如重置接收计数器
        // received_count = 0;
    }

    // 检查是否已接收到一帧完整的数据
    if (received_count >= IMAGE_SIZE_BYTES)
    {
        // 设置新帧接收完成标志
        new_frame_received_flag = 1;
        
        // 复位计数器，为下一帧做准备
        received_count = 0;

        // 【已移除】不再发送ACK，以解决上位机卡死问题
        // char ack = ACK_CHAR;
        // CDC_Transmit_FS((uint8_t*)&ack, 1);
    }

    // 【已移除】不再需要手动调用，系统会在 usbd_cdc_if.c 中自动调用
    // USBD_CDC_ReceivePacket(&hUsbDeviceFS); 
}
// --- LCD底层驱动函数 (8位SPI) ---
void SdCmd(uint16_t Command)
{
    uint8_t cmd_buffer[2];
    cmd_buffer[0] = (Command >> 8) & 0xFF; // High Byte
    cmd_buffer[1] = Command & 0xFF;        // Low Byte

    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, cmd_buffer, 2, 100);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

void SdData(uint16_t DData)
{
    uint8_t data_buffer[2];
    data_buffer[0] = (DData >> 8) & 0xFF; // High Byte
    data_buffer[1] = DData & 0xFF;        // Low Byte

    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, data_buffer, 2, 100);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}


void IST3257_UpdateScreen_DMA(uint8_t* buffer_to_send)
{
    while (spi_dma_busy) { __NOP(); }
    spi_dma_busy = 1;
    SdCmd(0x0008); SdData(0x0000);
    SdCmd(0x0009);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(&hspi1, buffer_to_send, FRAME_SIZE_BYTES);
}

// 保留您之前的阻塞式刷新函数
void IST3257_UpdateScreen(void)
{
    SdCmd(0x0008);
    SdData(0x0000);
    SdCmd(0x0009);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, back_buffer_ptr, FRAME_SIZE_BYTES, 1000);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}
// --- 保留您之前的函数 (修正了绘图目标) ---
void IST3257_DrawPixel(uint16_t x, uint16_t y, uint8_t level) {
    if (x >= IST3257_WIDTH || y >= IST3257_HEIGHT) {
        return;
    }
    level &= 0x0F;
    // 修正：让绘图函数总是画在 back_buffer_ptr 指向的缓冲区
    uint8_t *p = (uint8_t*)back_buffer_ptr + (y * (IST3257_WIDTH / 2)) + (x / 2);

    if (x % 2 == 0) {
        *p = (*p & 0x0F) | (level << 4);
    } else {
        *p = (*p & 0xF0) | level;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000); // 等待USB枚举完成

  // --- LCD 初始化 ---

  // --- 第一步：硬件复位 ---
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(20); // 手册要求复位低电平至少保持10ms [cite: 1324]
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(20); // 手册要求复位后至少等待10ms再发送指令 [cite: 1327]


  // --- 第二步：开启振荡器与基础应用设置 ---

  // 必须先开启振荡器，为后续指令提供时钟
  SdCmd(0x0000);    // Index: R00h, Start Oscillator 
  SdData(0x0001);   // Data:  OSC bit = 1, 启动振荡器 [cite: 1018]
  HAL_Delay(5);     // 短暂延时等待振荡器稳定

  // 设置驱动模式 (例如: 1/162 Duty, 16G模式)
  SdCmd(0x0001);    // Index: R01h, Driver control 
  SdData(0x0000);   // Data:  NL=00 -> Duty 1/162, SHL=0, SGS=0 [cite: 1037]

  SdCmd(0x0023);    // Index: R23h, Display Mode Control 
  SdData(0x0003);   // Data:  DSPM=100

  SdCmd(0x0028);      // Index: R28h, Frame Rate Control
  SdData(0x0088);
  // 设置电源参数 (Bias 和 Booster)
  SdCmd(0x0004);    // Index: R04h, Power control (2) 
  SdData(0x0065);   // Data:  Bias=1/12 (110), Booster=x7 (101) [cite: 1075, 1077]
                    //        0b... 0 010 0 100

  //方案B1: N-Line INV?
  SdCmd(0x0002);
  SdData(0x0100 + 20);//20-Line 反转
  //SdData(0x0000);//Frame Inv
  // 设置对比度
  SdCmd(0x0005);    // Index: R05h, Contrast control 
  SdData(0x0061);   // Data:  设置一个中间对比度值 (61/127)


  // --- 第三步：分三步开启内部电源 (最关键的修正) ---

  // 3a. 开启 VOUT1 和 VM
  SdCmd(0x0003);    // Index: R03h, Power control (1) 
  SdData(0x0940);   // Data: VSON=1, DC1=1, VMEN=1 
  HAL_Delay(15);    // 等待电压稳定，手册要求至少10ms [cite: 1335]

  // 3b. 开启 VG 和 VOUT2
  SdCmd(0x0003);    // Index: R03h, Power control (1) 
  SdData(0x0B60);   // Data: 在上一步基础上，增加 VGEN=1, DC2=1 
  HAL_Delay(15);    // 等待电压稳定，手册要求至少10ms [cite: 1339]

  // 3c. 开启 VO 和 XV0
  SdCmd(0x0003);    // Index: R03h, Power control (1) 
  SdData(0x0F70);   // Data: 在上一步基础上，增加 VOEN=1, DC3=1 
  HAL_Delay(15);    // 等待电压稳定，手册要求至少10ms [cite: 1343]


  // --- 第四步：开启显示 ---
  SdCmd(0x0007);    // Index: R07h, Display control 
  SdData(0x0001);   // Data:  D=1, 开启显示驱动输出 [cite: 1128
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // --- USB VCP 视频播放循环 ---

  // 初始化缓冲区指针
  front_buffer_ptr = framebuffer1;
  back_buffer_ptr = framebuffer2;
  memset(front_buffer_ptr, 0, FRAME_SIZE_BYTES);
  memset(back_buffer_ptr, 0, FRAME_SIZE_BYTES);
  // 首次启动USB接收
  for(int i = 32; i < 256-32; i++)
    for(int j = 32; j < 160-32; j++)
      IST3257_DrawPixel(i, j, (i/2) % 16);
  IST3257_UpdateScreen();
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // 检查是否有新的一帧数据被完整接收
    if (new_frame_received_flag)
    {
        // 清除标志
        new_frame_received_flag = 0;

        // 等待上一帧的DMA传输完成
        while (spi_dma_busy) {
            __NOP();
        }

        // 交换前后缓冲区指针
        uint8_t *temp_ptr = front_buffer_ptr;
        front_buffer_ptr = back_buffer_ptr;
        back_buffer_ptr = temp_ptr;

        // 启动DMA，开始显示新的“前台”缓冲区
        IST3257_UpdateScreen_DMA(front_buffer_ptr);

        // 清空新的后台缓冲区，为下一次USB接收做准备
        // (因为视频帧小于屏幕，所以需要清空以避免底部有残留)
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
