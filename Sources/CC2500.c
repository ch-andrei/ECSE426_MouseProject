/* Includes ------------------------------------------------------------------*/
#include "cc2500_settings.h"
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_spi.h>
#include "CC2500.h"
#include <cmsis_os.h>
#include "main.h"

/* Private variables ---------------------------------------------------------*/
/* SPI handler declaration */
SPI_HandleTypeDef SpiHandleCC2500;

/* Buffer used for transmission */
uint8_t aTxBuffer[] = "****SPI - Two Boards communication based on Polling **** SPI Message ******** SPI Message ******** SPI Message ****";

/* Buffer used for reception */
uint8_t aRxBuffer[BUFFERSIZE];

uint16_t CC2500_Timeout;

/* Private 'helper' functions -----------------------------------------------*/
void CC2500_SPI_read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
void CC2500_SPI_write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void CC2500_SPI_strobe(uint8_t *status, uint8_t StrobeAddr);
void CC2500_SPI_readFIFO(uint8_t *status, uint8_t num_bytes);
void CC2500_ReadRXFIFOtoBuffer(uint8_t *buffer, uint8_t num_bytes);
void CC2500_WriteBufferToTXFIFO(uint8_t *buffer, uint8_t num_bytes);
void CC2500_SPI_init(void);
void CC2500_HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void CC2500_stateRX(void);
void CC2500_stateTX(void);
void CC2500_stateIDLE(void);
uint8_t CC2500_ReadState(uint8_t StateAddr);
void CC2500_testing(void);

static void Error_Handler(void);
int CC2500_Timeout_Error_Handler(void);
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
void CC2500_settings_configure(void);

/**
  * @brief  Configures and tests the SPI communication with the CC2500 chip.
  * @param  None
  * @retval None
  */
void CC2500_configure(void){
	CC2500_SPI_init();
	CC2500_settings_configure();
	
	// run tests
	CC2500_testing();
	
	// set recei
	uint8_t state;
	#ifdef RECEIVER
		// flush RX FIFO and ensure that the CC2500 chip is in RX state
		CC2500_CommandStrobe(&state, CC2500_SFRX);
		CC2500_stateRX();
	#endif
	
	state = CC2500_ReadState(CC2500_MARCSTATE);
}

/**
  * @brief  Configures and starts the SPI instance
  * @param  None
  * @retval None
  */
void CC2500_SPI_init(void){
  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
	
	/* init SPI clock */
	__SPI2_CLK_ENABLE();
  HAL_SPI_DeInit(&SpiHandleCC2500);
	
	SpiHandleCC2500.Instance               = SPI2;
  SpiHandleCC2500.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  SpiHandleCC2500.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandleCC2500.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandleCC2500.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandleCC2500.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
  SpiHandleCC2500.Init.CRCPolynomial     = 7;
  SpiHandleCC2500.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandleCC2500.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandleCC2500.Init.NSS               = SPI_NSS_SOFT;
  SpiHandleCC2500.Init.TIMode            = SPI_TIMODE_DISABLED;
  SpiHandleCC2500.Init.Mode              = SPI_MODE_MASTER;
	
  if(HAL_SPI_Init(&SpiHandleCC2500) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
	
	__HAL_SPI_ENABLE(&SpiHandleCC2500);
	
	CC2500_HAL_SPI_MspInit(&SpiHandleCC2500);
}

/**
  * @brief  Configures the settings on the CC2500 chip by writing to relevant registers
  * @param  None
  * @retval None
  */
void CC2500_settings_configure(void){
	uint8_t temp;
  temp = VAL_CC2500_IOCFG2;  
  CC2500_WriteRegister(&temp,CC2500_IOCFG2);
  temp = VAL_CC2500_IOCFG0;
  CC2500_WriteRegister(&temp,CC2500_IOCFG0);
  temp = VAL_CC2500_FIFOTHR;
  CC2500_WriteRegister(&temp,CC2500_FIFOTHR);
  temp = VAL_CC2500_PKTLEN;
  CC2500_WriteRegister(&temp,CC2500_PKTLEN);
  temp = VAL_CC2500_PKTCTRL1;
  CC2500_WriteRegister(&temp,CC2500_PKTCTRL1);
  temp = VAL_CC2500_PKTCTRL0;
  CC2500_WriteRegister(&temp,CC2500_PKTCTRL0);
  temp = VAL_CC2500_ADDR;
  CC2500_WriteRegister(&temp,CC2500_ADDR);
  temp = VAL_CC2500_CHANNR;
  CC2500_WriteRegister(&temp,CC2500_CHANNR);
  temp = VAL_CC2500_FSCTRL1;
  CC2500_WriteRegister(&temp,CC2500_FSCTRL1);
  temp = VAL_CC2500_FSCTRL0;
  CC2500_WriteRegister(&temp,CC2500_FSCTRL0);
  temp = VAL_CC2500_FREQ2;
  CC2500_WriteRegister(&temp,CC2500_FREQ2);
  temp = VAL_CC2500_FREQ1;
  CC2500_WriteRegister(&temp,CC2500_FREQ1);
  temp = VAL_CC2500_FREQ0;
  CC2500_WriteRegister(&temp,CC2500_FREQ0);
  temp = VAL_CC2500_MDMCFG4;
  CC2500_WriteRegister(&temp,CC2500_MDMCFG4);
  temp = VAL_CC2500_MDMCFG3;
  CC2500_WriteRegister(&temp,CC2500_MDMCFG3);
  temp = VAL_CC2500_MDMCFG2;
  CC2500_WriteRegister(&temp,CC2500_MDMCFG2);
  temp = VAL_CC2500_MDMCFG1;
  CC2500_WriteRegister(&temp,CC2500_MDMCFG1);
  temp = VAL_CC2500_MDMCFG0;
  CC2500_WriteRegister(&temp,CC2500_MDMCFG0);
  temp = VAL_CC2500_DEVIATN;
  CC2500_WriteRegister(&temp,CC2500_DEVIATN);
  temp = VAL_CC2500_MCSM1;
  CC2500_WriteRegister(&temp,CC2500_MCSM1);
  temp = VAL_CC2500_MCSM0;
  CC2500_WriteRegister(&temp,CC2500_MCSM0);
  temp = VAL_CC2500_FOCCFG;
  CC2500_WriteRegister(&temp,CC2500_FOCCFG);
  temp = VAL_CC2500_BSCFG;
  CC2500_WriteRegister(&temp,CC2500_BSCFG);
  temp = VAL_CC2500_AGCTRL2;
  CC2500_WriteRegister(&temp,CC2500_AGCCTRL2);
  temp = VAL_CC2500_AGCTRL1;
  CC2500_WriteRegister(&temp,CC2500_AGCCTRL1);
  temp = VAL_CC2500_AGCTRL0;
  CC2500_WriteRegister(&temp,CC2500_AGCCTRL0);
  temp = VAL_CC2500_FREND1;
  CC2500_WriteRegister(&temp,CC2500_FREND1);
  temp = VAL_CC2500_FREND0;
  CC2500_WriteRegister(&temp,CC2500_FREND0);
  temp = VAL_CC2500_FSCAL3;
  CC2500_WriteRegister(&temp,CC2500_FSCAL3);
  temp = VAL_CC2500_FSCAL2;
  CC2500_WriteRegister(&temp,CC2500_FSCAL2);
  temp = VAL_CC2500_FSCAL1;
  CC2500_WriteRegister(&temp,CC2500_FSCAL1);
  temp = VAL_CC2500_FSCAL0;
  CC2500_WriteRegister(&temp,CC2500_FSCAL0);
  temp = VAL_CC2500_FSTEST;
  CC2500_WriteRegister(&temp,CC2500_FSTEST);
  temp = VAL_CC2500_TEST2;
  CC2500_WriteRegister(&temp,CC2500_TEST2);
  temp = VAL_CC2500_TEST1;
  CC2500_WriteRegister(&temp,CC2500_TEST1);
  temp = VAL_CC2500_TEST0;
	CC2500_WriteRegister(&temp,CC2500_TEST0);  
}

/**
  * @brief  Initializes GPIO for the SPI communication.
  * @param  *hspi: Pointer to the SPI handle. Its member Instance can point to either SPI1, SPI2 or SPI3 
  * @retval None
  */
void CC2500_HAL_SPI_MspInit(SPI_HandleTypeDef *hspi){
	GPIO_InitTypeDef GPIO_InitStructure;

	/* init SPI clock */
	__SPI2_CLK_ENABLE();
	
	/* init GPIO clocks for SCK/MISO/MOSI ports */
	__GPIOB_CLK_ENABLE();
	
	/* init GPIO clocks for CSn port */
	__GPIOD_CLK_ENABLE();

  GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull  = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStructure.Alternate = CC2500_SPIx_AF;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.Pin = CC2500_SPIx_SCK_PIN;
  HAL_GPIO_Init(CC2500_SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.Pin =  CC2500_SPIx_MOSI_PIN;
  HAL_GPIO_Init(CC2500_SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.Pin = CC2500_SPIx_MISO_PIN;
  HAL_GPIO_Init(CC2500_SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin   = CC2500_SPI_CS_PIN;
  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(CC2500_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  CC2500_CS_HIGH();
}

uint8_t CC2500_ReadState(uint8_t StateAddr)
{
	uint8_t status;
	CC2500_ReadRegister(&status, StateAddr);
	return status;
}

/**
  * @brief  Sets CC2500 to RX state; verifies for overflow in RX FIFO
  * @param  None.
  * @retval None.
  */
void CC2500_stateRX(void)
{
	//Check if the CC2500 is in RX mode, if not set it to RX state
	uint8_t checkState = CC2500_ReadState(CC2500_MARCSTATE);
	if (checkState != CC2500_STATE_RX)
	{
		// check if overflow
		if (checkState == CC2500_STATE_RXOVERFLOW)
		{
			// flush the RX FIFO
			CC2500_CommandStrobe(&checkState, CC2500_STROBE_FLUSHRX);
		}
		CC2500_CommandStrobe(&checkState, CC2500_STROBE_RX);
  }
}

/**
  * @brief  Sets the CC2500 to IDLE state
  * @param  None.
  * @retval None.
  */
void CC2500_stateIDLE(void)
{
	//Check if the CC2500 is in IDLE state
	uint8_t checkState = CC2500_ReadState(CC2500_MARCSTATE);
	if (checkState != CC2500_STATE_IDLE)
	{
		CC2500_CommandStrobe(&checkState, CC2500_STROBE_IDLE);
  }
}


/**
* @brief Sets the CC2500 to TX state;
* @param None.
* @retval None.
*/
void CC2500_stateTX(void)
{
	//Check if the CC2500 is in RX mode, if not strobe to place in RX
	uint8_t checkState;
	CC2500_ReadState(CC2500_MARCSTATE);
	if (checkState != CC2500_STATE_TX)
	{
		// check if underflow
		if(checkState == CC2500_STATE_TXUNDERFLOW)
		{
			// if in underflow, flush the FIFO
			CC2500_CommandStrobe(&checkState, CC2500_STROBE_FLUSHTX);
		}
		CC2500_CommandStrobe(&checkState, CC2500_STROBE_TX);
	}
}

/**
  * @brief  Recieve a packet from a transmitter.
  * @param  pBuffer: Pointer to the buffer that receives the data from the CC2500 RX FIFO.
  * @retval None.
  */
void CC2500_ReceivePacket(uint8_t* receivedPacket)
{	
	uint8_t bytesFIFO, packetLength;
	
	//ensure that the CC2500 chip is in RX state and check for overflow
	CC2500_stateRX();

	packetLength = VAL_CC2500_PKTLEN;

	//Reads the number of bytes in the RX FIFO and makes sure there is something to read
	bytesFIFO = (CC2500_ReadState(CC2500_RXBYTES)) & 0x7F;
	if (bytesFIFO > 0)
	{
		// Read a packet from RX FIFO
		CC2500_ReadRXFIFOtoBuffer(receivedPacket, packetLength);
	}
	else
	{
		printf("RX FIFO empty; cant't read from it.\n");
	}
}

/**
  * @brief  Send a packet wirelessly. Writes a packet to the TX FIFO and sets CC2500 to TX state.
  * @param  pBuffer: Pointer to the buffer where the data to be sent is located.
  * @retval None.
  */
void CC2500_SendPacket(uint8_t* sendPacket)
{
	uint8_t packetLength, status;
	
	packetLength = VAL_CC2500_PKTLEN;
	
	// flush TX and set idle state 
	// idle state is convenient for updatng the TX FIFO
	CC2500_CommandStrobe(&status, CC2500_STROBE_FLUSHTX);
	CC2500_CommandStrobe(&status, CC2500_STROBE_IDLE);
	
	// write a packet to TX FIFO
	CC2500_WriteBufferToTXFIFO(sendPacket, packetLength);

	// set TX mode on
	CC2500_stateTX();
}

/**
  * @brief  Writes a single byte value from a pointer to a register address
	* @param  reg_addr: address of the destination register
	* @param 	*value: pointer to the value to write
  * @retval None
  */
void CC2500_WriteRegister(uint8_t *value, uint8_t reg_addr){
	/* Set chip select Low at the start of the transmission */
  CC2500_CS_LOW();
  /* wait for CHIP_RDYn */
	while ( HAL_GPIO_ReadPin(CC2500_SPIx_MISO_GPIO_PORT, CC2500_SPIx_MISO_PIN) != RESET ){
	}
	
	uint8_t readback = 0;
	// loop handles incorrect writes
	// write, then read the register to verify if it the value was written correctly.
	// repeat until register contents are as expected.
	do {
		/* write through SPI */
		CC2500_SPI_write(value, reg_addr, 1);
		/* read from the register to verify that correct value was written */
		CC2500_ReadRegister(&readback, reg_addr);
	} while (readback != *value);
	
	/* Set chip select High at the end of the transmission */
  CC2500_CS_HIGH();
}

/**
  * @brief  Reads a single byte value from a register address to a pointer
	* @param  reg_addr: address of the destination register
	* @param 	*value: pointer to the value to write
  * @retval None
  */
void CC2500_ReadRegister(uint8_t *value, uint8_t reg_addr){
	/* Set chip select Low at the start of the transmission */
  CC2500_CS_LOW();
  /* wait for CHIP_RDYn */
	while ( HAL_GPIO_ReadPin(CC2500_SPIx_MISO_GPIO_PORT, CC2500_SPIx_MISO_PIN) != RESET ){
	}

	// set header R/W and BURST bits for read operation
  if(reg_addr >= 0x30) {
    reg_addr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  } else {
    reg_addr |= (uint8_t)READWRITE_CMD;
  }	
	// read 1 byte from the SPI bus
	CC2500_SPI_read(value, reg_addr, 1);
	/* Set chip select High at the end of the transmission */
  CC2500_CS_HIGH();
}

/**
  * @brief  Sets slave select low, sends a strobe command to the CC2500 using a helper function, and sets slave select high
  * @param  status : pointer to the variable to which CC2500 chip status will be written to
  * @param  strobe_reg_addr : CC2500's internal address for the strobe register to access
  * @retval None
  */
void CC2500_CommandStrobe(uint8_t *status, uint8_t strobe_reg_addr){
	/* Set chip select Low at the start of the transmission */
  CC2500_CS_LOW();
	/* wait for CHIP_RDYn */
	while ( HAL_GPIO_ReadPin(CC2500_SPIx_MISO_GPIO_PORT, CC2500_SPIx_MISO_PIN) != RESET ){
	}
	/* Send strobe command */
	CC2500_SPI_strobe(status, strobe_reg_addr);
	/* Set chip select High at the end of the transmission */
  CC2500_CS_HIGH();
}

/* ------------------------- PRIVATE FUNCTIONS ------------------------- */

/**
  * @brief  Returns the most recent received data by the SPIx/I2Sx peripheral. 
  * @param  *hspi: Pointer to the SPI handle. Its member Instance can point to either SPI1, SPI2 or SPI3 
  * @retval The value of the received data.
  */
uint8_t CC2500_SPI_ReceiveData(SPI_HandleTypeDef *hspi){
	return hspi->Instance->DR;
}

/**
  * @brief  Updates the byte to be transmitted through the SPIx/I2Sx peripheral.
  * @param  *hspi: Pointer to the SPI handle. Its member Instance can point to either SPI1, SPI2 or SPI3 
  * @param  Data: Data to be transmitted.
  * @retval None
  */
void CC2500_SPI_SendData(SPI_HandleTypeDef *hspi, uint8_t Data){
	hspi->Instance->DR = Data;
}

/**
  * @brief  Reads one or many bytes from the RX FIFO.
	* @param  reg_addr: address of the destination register
	* @param 	*value: pointer to the value to write
  * @retval None
  */
void CC2500_ReadRXFIFOtoBuffer(uint8_t *buffer, uint8_t num_bytes){
	/* Set chip select Low at the start of the transmission */
  CC2500_CS_LOW();
  /* wait for CHIP_RDYn */
	while ( HAL_GPIO_ReadPin(CC2500_SPIx_MISO_GPIO_PORT, CC2500_SPIx_MISO_PIN) != RESET ){
	}

	/* set read address */
	uint8_t reg_addr;
	if (num_bytes > 0x01){
		reg_addr = CC2500_RX_FIFO_BURST;
	} else {
		reg_addr = CC2500_RX_FIFO_SINGLE;
	}
	
	/* read through SPI */
	CC2500_SPI_read(buffer, reg_addr, num_bytes);
	
	/* Set chip select High at the end of the transmission */
  CC2500_CS_HIGH();
}

/**
  * @brief  Writes one or many bytes to the TX FIFO.
	* @param  num_bytes: numbers of bytes to write to TX FIFO
	* @param 	*buffer: pointer to the value to write
  * @retval None
  */
void CC2500_WriteBufferToTXFIFO(uint8_t *buffer, uint8_t num_bytes){
	/* Set chip select Low at the start of the transmission */
  CC2500_CS_LOW();
  /* wait for CHIP_RDYn */
	while ( HAL_GPIO_ReadPin(CC2500_SPIx_MISO_GPIO_PORT, CC2500_SPIx_MISO_PIN) != RESET ){
	}

	/* set read address */
	uint8_t write_addr;
	if (num_bytes > 0x01){
		write_addr = CC2500_TX_FIFO_BURST;
	} else {
		write_addr = CC2500_TX_FIFO_SINGLE;
	}
	
	/* read through SPI */
	CC2500_SPI_write(buffer, write_addr, num_bytes);
	
	/* Set chip select High at the end of the transmission */
  CC2500_CS_HIGH();
}

/**
  * @brief  Sends a Byte through the SPI interface and return the Byte received
  *         from the SPI bus.
  * @param  Byte : Byte send.
  * @retval The received byte value
  */
static uint8_t CC2500_SPI_SendByte(uint8_t byte)
{
  /* Loop while DR register in not empty */
  CC2500_Timeout = CC2500_FLAG_TIMEOUT;
  while (__HAL_SPI_GET_FLAG(&SpiHandleCC2500, SPI_FLAG_TXE) == RESET){
    if((CC2500_Timeout--) == 0) return CC2500_Timeout_Error_Handler();
  }

	/* Set byte to be transmitted */ 
  CC2500_SPI_SendData(&SpiHandleCC2500, byte);

  /* Wait to receive a Byte */
  CC2500_Timeout = CC2500_FLAG_TIMEOUT;
  while (__HAL_SPI_GET_FLAG(&SpiHandleCC2500, SPI_FLAG_RXNE) == RESET){
    if((CC2500_Timeout--) == 0) return CC2500_Timeout_Error_Handler();
  }

  /* Return the Byte read from the SPI bus */ 
  return CC2500_SPI_ReceiveData(&SpiHandleCC2500);
}

/**
  * @brief  Sends a strobe command to the CC2500
  * @param  status : pointer to the buffer to which CC2500 chip status will be written to
  * @param  StrobeAddr : CC2500's internal address for the strobe register to access
  * @retval None
  */
void CC2500_SPI_strobe(uint8_t *status, uint8_t StrobeAddr)
{
	// set header R/W to indicate it is a strobe
  StrobeAddr |= (uint8_t)(READWRITE_CMD);
	
  /* Send the Address of the indexed register */
	*status = CC2500_SPI_SendByte(StrobeAddr);
}

/**
  * @brief  Reads a block of data from the CC2500.
  * @param  pBuffer : pointer to the buffer that receives the data read from the CC2500.
  * @param  ReadAddr : CC2500's internal address to read from.
  * @param  NumByteToRead : number of bytes to read from the CC2500.
  * @retval None
  */
void CC2500_SPI_read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	if(NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }
	
  /* Send the Address of the indexed register */
  CC2500_SPI_SendByte(ReadAddr);

  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to LIS3DSH (Slave device) */
    *pBuffer = CC2500_SPI_SendByte(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
  }
}

/**
  * @brief  Writes one byte to the CC2500.
  * @param  pBuffer : pointer to the buffer  containing the data to be written to the CC2500.
  * @param  WriteAddr : CC2500's internal address to write to.
  * @param  NumByteToWrite: Number of bytes to write.
  * @retval None
  */
void CC2500_SPI_write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{  
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }

  /* Send the Address of the indexed register */
  CC2500_SPI_SendByte(WriteAddr);
  /* Send the data that will be written into the device (MSB First) */
  while(NumByteToWrite >= 0x01)
  {
    CC2500_SPI_SendByte(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while(1)
  {
  }
}

/**
  * @brief  SPI error callbacks
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	while(1)
  {
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
int CC2500_Timeout_Error_Handler(void)
{
	return -1;
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

// ------------------- TESTING -------------------
void CC2500_testing(void){
	// test 1
	// read registers
	uint8_t VERSION = 0, PARTNUM = 0;
	CC2500_ReadRegister(&VERSION, CC2500_VERSION);
	CC2500_ReadRegister(&PARTNUM, CC2500_PARTNUM);
	
	// test 2	
	// read after write registers
	uint8_t temp[3];
	temp[0] = VAL_CC2500_FREQ0;
	temp[1] = VAL_CC2500_FREQ1;
	temp[2] = VAL_CC2500_FREQ2;
	CC2500_WriteRegister(temp, CC2500_FREQ0);
	CC2500_WriteRegister(temp+1, CC2500_FREQ1);
	CC2500_WriteRegister(temp+2, CC2500_FREQ2);
	
	CC2500_ReadRegister(temp, CC2500_FREQ0);
	CC2500_ReadRegister(temp+1, CC2500_FREQ1);
	CC2500_ReadRegister(temp+2, CC2500_FREQ2);
	
	uint8_t error = 0;
	if (temp[0] != VAL_CC2500_FREQ0 || temp[1] != VAL_CC2500_FREQ1  || temp[2] != VAL_CC2500_FREQ2 )
		error = 1;
	
	/*
	// test 3
	// burst read/write
	// write 3 bytes to registers
	
	// test 4
	// find the number of bytes left read appropriate TX and RX registers
	uint8_t status;
	while (1){
		uint8_t tempTX, tempRX, TXBYTES, RXBYTES, TX_overflow, RX_overflow;
		CC2500_ReadRegister(&tempTX, CC2500_TXBYTES);
		CC2500_ReadRegister(&tempRX, CC2500_RXBYTES);
		TXBYTES = tempTX & 0x7f;
		TX_overflow = tempTX & 0x80;
		RXBYTES = tempRX & 0x7f;
		RX_overflow = tempRX & 0x80;
	
		printf("TX bytes %d, overflow %d; RX bytes %d, overflow %d.\n", TXBYTES, TX_overflow, RXBYTES, RX_overflow);
	}
	
	// test x 
	// transmit / receive packets
	
	// test 5
	// issue strobe command
	// CC2500_CommandStrobe(&status, CC2500_SRES);
	
	// dummy operation for a breakpoint
	error = error;
	*/
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
