/**
  ******************************************************************************
  * @file    SPI/SPI_FullDuplex_ComPolling/Inc/main.h 
  * @author  MCD Application Team; Modified by: Andrei Chubarau, McGill University. 
  * @version V1.1.0
  * @date    26-June-2014
  * @brief   Header for main.c module. Modified for use for an ECSE426 project.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CC2500_H
#define CC2500_H

#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_spi.h>

/* Exported Functions --------------------------------------------------------*/
void CC2500_configure(void);

/* Public function -----------------------------------------------------------*/
void CC2500_WriteRegister(uint8_t *value, uint8_t reg_addr);
void CC2500_ReadRegister(uint8_t *value, uint8_t reg_addr);
void CC2500_CommandStrobe(uint8_t *status, uint8_t strobe_reg_addr);
void CC2500_ReceivePacket(uint8_t* receivedPacket);
void CC2500_SendPacket(uint8_t* sendPacket);

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor SPIx instance used and associated 
   resources */
/* Definition for SPIx clock resources */

/* Definition for SPIx Pins */
/* Using SPI2 pins */
#define CC2500_SPIx_AF                   GPIO_AF5_SPI2

#define CC2500_SPIx_SCK_PIN              GPIO_PIN_13
#define CC2500_SPIx_SCK_GPIO_PORT        GPIOB

#define CC2500_SPIx_MISO_PIN             GPIO_PIN_14
#define CC2500_SPIx_MISO_GPIO_PORT       GPIOB

#define CC2500_SPIx_MOSI_PIN             GPIO_PIN_15
#define CC2500_SPIx_MOSI_GPIO_PORT       GPIOB

#define CC2500_SPI_CS_PIN                GPIO_PIN_8
#define CC2500_SPI_CS_GPIO_PORT          GPIOD
#define CC2500_SPI_CS_GPIO_CLK           RCC_AHB1Periph_GPIOD

#define CC2500_CS_LOW()       					 HAL_GPIO_WritePin(CC2500_SPI_CS_GPIO_PORT, CC2500_SPI_CS_PIN, GPIO_PIN_RESET)
#define CC2500_CS_HIGH()      					 HAL_GPIO_WritePin(CC2500_SPI_CS_GPIO_PORT, CC2500_SPI_CS_PIN, GPIO_PIN_SET)

/*--------------------------------------------------------------------------
	CC2500 Register addresses
--------------------------------------------------------------------------*/

#define CC2500_IOCFG2           0x00        // GDO2 output pin configuration
#define CC2500_IOCFG1           0x01        // GDO1 output pin configuration
#define CC2500_IOCFG0           0x02        // GDO0 output pin configuration
#define CC2500_FIFOTHR          0x03        // RX FIFO and TX FIFO thresholds
#define CC2500_SYNC1            0x04        // Sync word, high byte
#define CC2500_SYNC0            0x05        // Sync word, low byte
#define CC2500_PKTLEN           0x06        // Packet length
#define CC2500_PKTCTRL1         0x07        // Packet automation control
#define CC2500_PKTCTRL0         0x08        // Packet automation control
#define CC2500_ADDR             0x09        // Device address
#define CC2500_CHANNR           0x0A        // Channel number
#define CC2500_FSCTRL1          0x0B        // Frequency synthesizer control
#define CC2500_FSCTRL0          0x0C        // Frequency synthesizer control
#define CC2500_FREQ2            0x0D        // Frequency control word, high byte
#define CC2500_FREQ1            0x0E        // Frequency control word, middle byte
#define CC2500_FREQ0            0x0F        // Frequency control word, low byte
#define CC2500_MDMCFG4          0x10        // Modem configuration
#define CC2500_MDMCFG3          0x11        // Modem configuration
#define CC2500_MDMCFG2          0x12        // Modem configuration
#define CC2500_MDMCFG1          0x13        // Modem configuration
#define CC2500_MDMCFG0          0x14        // Modem configuration
#define CC2500_DEVIATN          0x15        // Modem deviation setting
#define CC2500_MCSM2            0x16        // Main Radio Cntrl State Machine config
#define CC2500_MCSM1            0x17        // Main Radio Cntrl State Machine config
#define CC2500_MCSM0            0x18        // Main Radio Cntrl State Machine config
#define CC2500_FOCCFG           0x19        // Frequency Offset Compensation config
#define CC2500_BSCFG            0x1A        // Bit Synchronization configuration
#define CC2500_AGCCTRL2         0x1B        // AGC control
#define CC2500_AGCCTRL1         0x1C        // AGC control
#define CC2500_AGCCTRL0         0x1D        // AGC control
#define CC2500_WOREVT1          0x1E        // High byte Event 0 timeout
#define CC2500_WOREVT0          0x1F        // Low byte Event 0 timeout
#define CC2500_WORCTRL          0x20        // Wake On Radio control
#define CC2500_FREND1           0x21        // Front end RX configuration
#define CC2500_FREND0           0x22        // Front end TX configuration
#define CC2500_FSCAL3           0x23        // Frequency synthesizer calibration
#define CC2500_FSCAL2           0x24        // Frequency synthesizer calibration
#define CC2500_FSCAL1           0x25        // Frequency synthesizer calibration
#define CC2500_FSCAL0           0x26        // Frequency synthesizer calibration
#define CC2500_RCCTRL1          0x27        // RC oscillator configuration
#define CC2500_RCCTRL0          0x28        // RC oscillator configuration
#define CC2500_FSTEST           0x29        // Frequency synthesizer cal control
#define CC2500_PTEST            0x2A        // Production test
#define CC2500_AGCTEST          0x2B        // AGC test
#define CC2500_TEST2            0x2C        // Various test settings
#define CC2500_TEST1            0x2D        // Various test settings
#define CC2500_TEST0            0x2E        // Various test settings

// Status registers
#define CC2500_PARTNUM          0x30        // Part number
#define CC2500_VERSION          0x31        // Current version number
#define CC2500_FREQEST          0x32        // Frequency offset estimate
#define CC2500_LQI              0x33        // Demodulator estimate for link quality
#define CC2500_RSSI             0x34        // Received signal strength indication
#define CC2500_MARCSTATE        0x35        // Control state machine state
#define CC2500_WORTIME1         0x36        // High byte of WOR timer
#define CC2500_WORTIME0         0x37        // Low byte of WOR timer
#define CC2500_PKTSTATUS        0x38        // Current GDOx status and packet status
#define CC2500_VCO_VC_DAC       0x39        // Current setting from PLL cal module
#define CC2500_TXBYTES          0x3A        // Underflow and # of bytes in TXFIFO
#define CC2500_RXBYTES          0x3B        // Overflow and # of bytes in RXFIFO

/*----------------------------------------------------------------------------------
	Command byte bitmasks
----------------------------------------------------------------------------------*/

/* Read/Write command */
#define READWRITE_CMD              ((uint8_t)0x80)
/* Multiple byte read/write command */
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint8_t)0x00)

/*----------------------------------------------------------------------------------
  Strobe command byte presets
----------------------------------------------------------------------------------*/

#define CC2500_SRES             0x30        // Reset chip.
#define CC2500_SFSTXON          0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                            // If in RX/TX: Go to a wait state where only the synthesizer is
                                            // running (for quick RX / TX turnaround).
#define CC2500_SXOFF            0x32        // Turn off crystal oscillator.
#define CC2500_SCAL             0x33        // Calibrate frequency synthesizer and turn it off
                                            // (enables quick start).
#define CC2500_SRX              0x34        // Enable RX. Perform calibration first if coming from IDLE and
                                            // MCSM0.FS_AUTOCAL=1.
#define CC2500_STX              0x35        // In IDLE state: Enable TX. Perform calibration first if
                                            // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                            // Only go to TX if channel is clear.
#define CC2500_SIDLE            0x36        // Exit RX / TX, turn off frequency synthesizer and exit
                                            // Wake-On-Radio mode if applicable.
#define CC2500_SAFC             0x37        // Perform AFC adjustment of the frequency synthesizer
#define CC2500_SWOR             0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CC2500_SPWD             0x39        // Enter power down mode when CSn goes high.
#define CC2500_SFRX             0x3A        // Flush the RX FIFO buffer.
#define CC2500_SFTX             0x3B        // Flush the TX FIFO buffer.
#define CC2500_SWORRST          0x3C        // Reset real time clock.
#define CC2500_SNOP             0x3D        // No operation. May be used to pad strobe commands to two
                                            // bytes for simpler software.
																						
/**
  * @brief often used CC2500 Command Strobes
  */
#define CC2500_STROBE_RESET					CC2500_SRES
#define CC2500_STROBE_AUTOCAL				CC2500_SCAL
#define CC2500_STROBE_RX    				CC2500_SRX
#define CC2500_STROBE_TX    				CC2500_STX
#define CC2500_STROBE_IDLE    			CC2500_SIDLE
#define CC2500_STROBE_FLUSHRX       CC2500_SFRX
#define CC2500_STROBE_FLUSHTX       CC2500_SFTX

/*----------------------------------------------------------------------------------
	FIFO commands and constants
----------------------------------------------------------------------------------*/

#define CC2500_TX_FIFO_SINGLE 	0x3F 				// Single byte access to TX FIFO
#define CC2500_TX_FIFO_BURST 		0x7F 				// Burst access to TX FIFO
#define CC2500_RX_FIFO_SINGLE 	0xBF 				// Single byte access to RX FIFO
#define CC2500_RX_FIFO_BURST		0xFF 				// Burst access to RX FIFO)

#define CC2500_FIFO_SIZE				((uint8_t) 0x40) //64 bytes

/*----------------------------------------------------------------------------------
	Chip Status Byte
----------------------------------------------------------------------------------*/

// Bit fields in the chip status byte
#define CC2500_STATUS_CHIP_RDYn_BM             0x80
#define CC2500_STATUS_STATE_BM                 0x70
#define CC2500_STATUS_FIFO_BYTES_AVAILABLE_BM  0x0F

#define CC2500_STATE_SLEEP     	    ((uint8_t) 0x00)
#define CC2500_STATE_IDLE     	    ((uint8_t) 0x01)
#define CC2500_STATE_RX    	    		((uint8_t) 0x0D)
#define CC2500_STATE_TX		     	    ((uint8_t) 0x13)
#define CC2500_STATE_RXOVERFLOW     ((uint8_t) 0x11)
#define CC2500_STATE_TXUNDERFLOW    ((uint8_t) 0x16)

//----------------------------------------------------------------------------------
// Other register bit fields
//----------------------------------------------------------------------------------
#define CC2500_LQI_CRC_OK_BM                   0x80
#define CC2500_LQI_EST_BM                      0x7F

//----------------------------------------------------------------------------------
// Constant defintions
//----------------------------------------------------------------------------------

#define CC2500_FLAG_TIMEOUT 									 1000

//----------------------------------------------------------------------------------
// Other macros
//----------------------------------------------------------------------------------

/* Size of buffer */
#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

#endif /* CC2500_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
