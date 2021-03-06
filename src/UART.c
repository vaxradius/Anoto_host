
//*****************************************************************************

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "sbc.h"

#define SBC_IN_RING_BUFF_SIZE           (128*2)

//
// If using SBC compression, select audio transfer compression ratio
// 1:1 = 256000 bps, 4:1 = 64000 bps, 8:1 = 32000 bps, 16:1 = 16000 bps
//
#define SBC_BLUEZ_COMPRESS_BPS          64000
#define SBC_OUT_RING_BUFF_SIZE          (SBC_BLUEZ_COMPRESS_BPS / 1000)

#define CODEC_IN_RING_BUFF_SIZE     SBC_IN_RING_BUFF_SIZE
#define CODEC_OUT_RING_BUFF_SIZE    SBC_OUT_RING_BUFF_SIZE

int8_t codecInputBuffer[CODEC_IN_RING_BUFF_SIZE];
uint8_t codecOutputBuffer[CODEC_OUT_RING_BUFF_SIZE];
sbc_t   g_BluezSBdecodeCInstance;

//*****************************************************************************
//
// UART handle.
//
//*****************************************************************************
void *phUART;

#define CHECK_ERRORS(x)                                                       \
    if ((x) != AM_HAL_STATUS_SUCCESS)                                         \
    {                                                                         \
        error_handler(x);                                                     \
    }

volatile uint32_t ui32LastError;

//*****************************************************************************
//
// Catch HAL errors.
//
//*****************************************************************************
void
error_handler(uint32_t ui32ErrorStatus)
{
    ui32LastError = ui32ErrorStatus;

    while (1);
}

//*****************************************************************************
//
// UART buffers.
//
//*****************************************************************************
uint8_t g_pui8TxBuffer[256];
uint8_t g_pui8RxBuffer[256];

//*****************************************************************************
//
// UART configuration.
//
//*****************************************************************************
const am_hal_uart_config_t g_sUartConfig =
{
    //
    // Standard UART settings: 115200-8-N-1
    //
    .ui32BaudRate = 115200,
    .ui32DataBits = AM_HAL_UART_DATA_BITS_8,
    .ui32Parity = AM_HAL_UART_PARITY_NONE,
    .ui32StopBits = AM_HAL_UART_ONE_STOP_BIT,
    .ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE,

    //
    // Set TX and RX FIFOs to interrupt at half-full.
    //
    .ui32FifoLevels = (AM_HAL_UART_TX_FIFO_1_2 |
                       AM_HAL_UART_RX_FIFO_1_2),

    //
    // Buffers
    //
    .pui8TxBuffer = g_pui8TxBuffer,
    .ui32TxBufferSize = sizeof(g_pui8TxBuffer),
    .pui8RxBuffer = g_pui8RxBuffer,
    .ui32RxBufferSize = sizeof(g_pui8RxBuffer),
};

//*****************************************************************************
//
// UART0 interrupt handler.
//
//*****************************************************************************
void
am_uart_isr(void)
{
    //
    // Service the FIFOs as necessary, and clear the interrupts.
    //
    uint32_t ui32Status, ui32Idle;
#if 0	
    am_hal_uart_interrupt_status_get(phUART, &ui32Status, true);
    am_hal_uart_interrupt_clear(phUART, ui32Status);
    am_hal_uart_interrupt_service(phUART, ui32Status, &ui32Idle);
#else	
    am_hal_uart_interrupt_status_get(phUART, &ui32Status, true);
    am_hal_uart_interrupt_clear(phUART, ui32Status);
    am_hal_uart_interrupt_service(phUART, ui32Status, 0);
#endif
}

char getChar(void)
{

	uint32_t ret = AM_HAL_STATUS_SUCCESS;
	uint32_t transfer_size = 0;
	char data = 0;

	am_hal_uart_transfer_t sUartRead = {
		.ui32Direction = AM_HAL_UART_READ,
		.pui8Data = (uint8_t *) &data,
		.ui32NumBytes = 1,
		.ui32TimeoutMs = AM_HAL_UART_WAIT_FOREVER,
		.pui32BytesTransferred = &transfer_size,
	};

	ret = am_hal_uart_transfer(phUART, &sUartRead);

	if (ret != AM_HAL_STATUS_SUCCESS || transfer_size != 1)
		return 0;
	else
		return data;

}

//*****************************************************************************
//
// UART print string
//
//*****************************************************************************
void
uart_print(char *pcStr)
{
    uint32_t ui32StrLen = 0;
    uint32_t ui32BytesWritten = 0;

    //
    // Measure the length of the string.
    //
    while (pcStr[ui32StrLen] != 0)
    {
        ui32StrLen++;
    }

    //
    // Print the string via the UART.
    //
    const am_hal_uart_transfer_t sUartWrite =
    {
        .ui32Direction = AM_HAL_UART_WRITE,
        .pui8Data = (uint8_t *) pcStr,
        .ui32NumBytes = ui32StrLen,
        .ui32TimeoutMs = AM_HAL_UART_WAIT_FOREVER,
        .pui32BytesTransferred = &ui32BytesWritten,
    };

    CHECK_ERRORS(am_hal_uart_transfer(phUART, &sUartWrite));

    if (ui32BytesWritten != ui32StrLen)
    {
        //
        // Couldn't send the whole string!!
        //
        while(1);
    }
}


//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
Uart_Init(void)
{
    //
    // Initialize the printf interface for UART output.
    //
    CHECK_ERRORS(am_hal_uart_initialize(0, &phUART));
    CHECK_ERRORS(am_hal_uart_power_control(phUART, AM_HAL_SYSCTRL_WAKE, false));
    CHECK_ERRORS(am_hal_uart_configure(phUART, &g_sUartConfig));

    //
    // Enable the UART pins.
    //
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, g_AM_BSP_GPIO_COM_UART_TX);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, g_AM_BSP_GPIO_COM_UART_RX);

    //
    // Enable interrupts.
    //
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST));

    am_hal_uart_interrupt_enable(phUART,
                                 (AM_HAL_UART_INT_RX | AM_HAL_UART_INT_TX |
                                  AM_HAL_UART_INT_RX_TMOUT | AM_HAL_UART_INT_TXCMP));

    //
    // Set the main print interface to use the UART print function we defined.
    //
    am_util_stdio_printf_init(uart_print);

}


#define PDM_DUMP_SIZE                (1024*(180-12))
//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
uint16_t g_ui16PDMDataBuffer[PDM_DUMP_SIZE];

void
pcm_print(void)
{
	int16_t *pi16PDMData = (int16_t *) g_ui16PDMDataBuffer;

	for (uint32_t i = 0; i < PDM_DUMP_SIZE; i++)
	{
		am_util_stdio_printf("%d\n", pi16PDMData[i]);
	}

	while(1);
}


uint32_t index = 0;
/*retun true means sram is full*/
bool store2sram(uint16_t *in,uint32_t len)
{
	int32_t CompressedLen;
	
	if(len != CODEC_OUT_RING_BUFF_SIZE/2)
	{
		am_util_stdio_printf("len(%d) != (%d)CODEC_OUT_RING_BUFF_SIZE/2\r\n", len, CODEC_OUT_RING_BUFF_SIZE/2);
		while(1);
	}
	
	sbc_decoder_decode(&g_BluezSBdecodeCInstance, (const void *) in, CODEC_OUT_RING_BUFF_SIZE, 
						(void *) codecInputBuffer, CODEC_IN_RING_BUFF_SIZE, &CompressedLen);
	
	memcpy(g_ui16PDMDataBuffer+index, codecInputBuffer, CODEC_IN_RING_BUFF_SIZE);
	
	index += (CODEC_IN_RING_BUFF_SIZE/2);
	if(index >= PDM_DUMP_SIZE)
	{
		index = 0;
		return true;
	}
	else
		return false;

}

void SBC_init(void)
{
	sbc_decode_init(&g_BluezSBdecodeCInstance, 0);  //0: SBC
	if(g_BluezSBdecodeCInstance.priv_alloc_base == 0)
	{
		am_util_stdio_printf("g_BluezSBdecodeCInstance.priv_alloc_base == 0\r\n");
		while(1);
	}
	g_BluezSBdecodeCInstance.endian = SBC_LE;
}


