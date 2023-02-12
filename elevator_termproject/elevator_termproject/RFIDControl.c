/*
 * RFIDControl.c
 *
 *  Author: lbbor
 */ 
#ifndef F_CPU
/* prevent compiler error by supplying a default */
# warning "F_CPU not defined for "RFIDControl.h"
# define F_CPU 14745600UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "UARTControl.h"
#include "RFIDControl.h"


Uid uid;								// Used by PICC_ReadCardSerial().

/* pin layout used:
 * --------------------------------------
 *             MFRC522      ATMEGA128      
 * Signal      Pin          Pin          
 * --------------------------------------
 * RST/Reset   RST          PORTC0            
 * SPI SS      SDA(SS)      PORTB0           
 * SPI MOSI    MOSI         PORTB2  
 * SPI MISO    MISO         PORTB3  
 * SPI SCK     SCK          PORTB1  
 */


void SPI_Master_Init(void)
{
    // I/O를 master로 동작으로 설정 (MOSI, SS, SCK 출력, MISO 입력)
    DDRB = (1<<SPI_SCK) | (1<<SPI_MOSI) | (1<<SPI_MISO);
    DDRB &= ~(1<<SPI_MISO);
    // SPCR을 SPI 및 마스터로 설정하고
    // MSB First, 모드 0, 속도 fosc/64
    SPCR = (0<<CPHA) | (0<<CPOL) | (1<<SPE) | (1<<MSTR)  | (0<<SPR1) | (0<<SPR0);
    SPSR=0x00;        // SPIF 클리어, SPI2X=0
}



// polling 모드로 전송
// 입력 인자 : 전송 데이터, 출력 인자 수신 데이터
unsigned char SPI_Master_Send(unsigned char data)
{
    SPDR = data;	// 데이터 전송
    while(!(SPSR & (1<<SPIF)));
    // 0x80 - Data가 송신이 끝날 때 까지 대기
    return SPDR;
    // 데이터 송신과 동시에 슬레이브로 부터 데이터 수신됨.
    // 무의미한 값이므로 읽어서 버림.
}

void RFID_Init(void)
{
    sb(RFID_NRST_PORT,RFID_NRST_PIN);
    sb(RFID_NSS_PORT,RFID_NSS_PIN);
    
    sb(RFID_NRST_DDR,RFID_NRST_PIN);
    sb(RFID_NSS_DDR,RFID_NSS_PIN);
    
    cb(RFID_NRST_DDR,RFID_NRST_PIN);
    _delay_us(100);
    sb(RFID_NRST_DDR,RFID_NRST_PIN);    
    
    _delay_ms(100);
    
    PCD_Init();    
}


/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_WriteRegister(	PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
					uint8_t value			///< The value to write.
) {    
    RFID_SS_EN();
    SPI_Master_Send(reg);					// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    SPI_Master_Send(value);					// Read the value back. Send 0 to stop reading.
    RFID_SS_DIS();
    return value;
} // End PCD_WriteRegister()

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_WriteRegister_n(	PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
                            uint8_t count,		///< The number of bytes to write to the register
                            uint8_t *values		///< The values to write. Byte array.
) {
    
    RFID_SS_EN();
    SPI_Master_Send(reg);					// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
    for (uint8_t index = 0; index < count; index++) {
        SPI_Master_Send(values[index]);
    }
    RFID_SS_DIS();
} // End PCD_WriteRegister_n()


/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
uint8_t PCD_ReadRegister(	PCD_Register reg	///< The register to read from. One of the PCD_Register enums.
								) {
    uint8_t value;
    RFID_SS_EN();
    SPI_Master_Send(0x80 | reg);					// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    value = SPI_Master_Send(0);					// Read the value back. Send 0 to stop reading.
    RFID_SS_DIS();
    return value;
} // End PCD_ReadRegister()


/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_ReadRegister_n(	PCD_Register reg,	///< The register to read from. One of the PCD_Register enums.
						uint8_t count,			///< The number of bytes to read
						uint8_t *values,		///< Byte array to store the values in.
						uint8_t rxAlign		///< Only bit positions rxAlign..7 in values[0] are updated.
) {
	if (count == 0) {
		return;
	}
	uint8_t address = 0x80 | reg;				// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    int i=0;
    RFID_SS_EN();
    
    count--;								// One read is performed outside of the loop
    SPI_Master_Send( address );
    
    if (rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
        // Create bit mask for bit positions rxAlign..7
        uint8_t mask = (0xFF << rxAlign) & 0xFF;
        // Read value and tell that we want to read the same address again.
        uint8_t value = SPI_Master_Send(address);
        // Apply mask to both current value of values[0] and the new data in value.
        values[0] = (values[0] & ~mask) | (value & mask);
        i++;
    }
    while (i < count) {
        values[i] = SPI_Master_Send(address);	// Read value and tell that we want to read the same address again.
        i++;
    }
    values[i] = SPI_Master_Send(0);			// Read the final byte. Send 0 to stop reading.
        
    RFID_SS_DIS();
}    
    
/**
 * Sets the bits given in mask in register reg.
 */
void PCD_SetRegisterBitMask(	PCD_Register reg,	///< The register to update. One of the PCD_Register enums.
							uint8_t mask			///< The bits to set.
									) { 
	uint8_t tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp | mask);			// set bit mask
} // End PCD_SetRegisterBitMask()

/**
 * Clears the bits given in mask from register reg.
 */
void PCD_ClearRegisterBitMask(	PCD_Register reg,	///< The register to update. One of the PCD_Register enums.
							uint8_t mask			///< The bits to clear.
									  ) {
	uint8_t tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp & (~mask));		// clear bit mask
} // End PCD_ClearRegisterBitMask()


/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_CalculateCRC(	uint8_t *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
								uint8_t length,	///< In: The number of bytes to transfer.
								uint8_t *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
) {
    int timeCnt = 0;                         
	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop any active command.
	PCD_WriteRegister(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	PCD_WriteRegister(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister_n(FIFODataReg, length, data);	// Write data to the FIFO
	PCD_WriteRegister(CommandReg, PCD_CalcCRC);		// Start the calculation
	
	// Wait for the CRC calculation to complete. Check for the register to
	// indicate that the CRC calculation is complete in a loop. If the
	// calculation is not indicated as complete in ~90ms, then time out
	// the operation.

	do {
		// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		uint8_t n = PCD_ReadRegister(DivIrqReg);
		if (n & 0x04) {									// CRCIRq bit set - calculation done
			PCD_WriteRegister(CommandReg, PCD_Idle);	// Stop calculating CRC for new content in the FIFO.
			// Transfer the result from the registers to the result buffer
			result[0] = PCD_ReadRegister(CRCResultRegL);
			result[1] = PCD_ReadRegister(CRCResultRegH);
			return STATUS_OK;
		}
        _delay_ms(1);
        timeCnt++;
	}
	while (timeCnt < 90);

	// 89ms passed and nothing happened. Communication with the MFRC522 might be down.
	return STATUS_TIMEOUT;
} // End PCD_CalculateCRC()




/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the MFRC522 chip.
 */
void PCD_Init() {

	// Set the chipSelectPin as digital output, do not select the slave yet
	sb(RFID_NSS_DDR, RFID_NSS_PIN);
	sb(RFID_NSS_PORT, RFID_NSS_PIN);
    
    sb(RFID_NRST_DDR, RFID_NRST_PIN);
    cb(RFID_NRST_PORT, RFID_NRST_PIN);
    _delay_us(2);
    sb(RFID_NRST_PORT, RFID_NRST_PIN);
        
	// Reset baud rates
	PCD_WriteRegister(TxModeReg, 0x00);
	PCD_WriteRegister(RxModeReg, 0x00);
	// Reset ModWidthReg
	PCD_WriteRegister(ModWidthReg, 0x26);

	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	PCD_WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
	PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteRegister(TReloadRegL, 0xE8);
	
	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End PCD_Init()


/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void PCD_Reset() {
	PCD_WriteRegister(CommandReg, PCD_SoftReset);	// Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
	uint8_t count = 0;
	do {
		// Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
		_delay_ms(50);
	} while ((PCD_ReadRegister(CommandReg) & (1 << 4)) && (++count) < 3);
} // End PCD_Reset()

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void PCD_AntennaOn() {
	uint8_t value = PCD_ReadRegister(TxControlReg);
	if ((value & 0x03) != 0x03) {
		PCD_WriteRegister(TxControlReg, value | 0x03);
	}
} // End PCD_AntennaOn()

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void PCD_AntennaOff() {
	PCD_ClearRegisterBitMask(TxControlReg, 0x03);
} // End PCD_AntennaOff()

/**
 * Get the current MFRC522 Receiver Gain (RxGain[2:0]) value.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 * 
 * @return Value of the RxGain, scrubbed to the 3 bits used.
 */
uint8_t PCD_GetAntennaGain() {
	return PCD_ReadRegister(RFCfgReg) & (0x07<<4);
} // End PCD_GetAntennaGain()

/**
 * Set the MFRC522 Receiver Gain (RxGain) to value specified by given mask.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Given mask is scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 */
void PCD_SetAntennaGain(uint8_t mask) {
	if (PCD_GetAntennaGain() != mask) {						// only bother if there is a change
		PCD_ClearRegisterBitMask(RFCfgReg, (0x07<<4));		// clear needed to allow 000 pattern
		PCD_SetRegisterBitMask(RFCfgReg, mask & (0x07<<4));	// only set RxGain[2:0] bits
	}
} // End PCD_SetAntennaGain()




/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_TransceiveData(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
								uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
								uint8_t *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
								uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
								uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
								uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
								uint8_t checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
								 ) {
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()

/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_CommunicateWithPICC(	uint8_t command,		///< The command to execute. One of the PCD_Command enums.
									uint8_t waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
									uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
									uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
									uint8_t *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
									uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
									uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
									uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
									uint8_t checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
) {
    int timeCnt = 0;
	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	PCD_WriteRegister(FIFOLevelReg, 0x80);				// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister_n(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
	PCD_WriteRegister(CommandReg, command);				// Execute the command
	if (command == PCD_Transceive) {
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}
	
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer
	// automatically starts when the PCD stops transmitting.
	//
	// Wait here for the command to complete. The bits specified in the
	// `waitIRq` parameter define what bits constitute a completed command.
	// When they are set in the ComIrqReg register, then the command is
	// considered complete. If the command is not indicated as complete in
	// ~36ms, then consider the command as timed out.
	uint8_t completed = false;

	do {
		uint8_t n = PCD_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			completed = true;
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
        timeCnt++;
        _delay_us(100);
	}
	while (timeCnt < 400);

	// 36ms and nothing happened. Communication with the MFRC522 might be down.
	if (!completed) {
		return STATUS_TIMEOUT;
	}
	
	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}
  
	uint8_t _validBits = 0;
	
	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		uint8_t n = PCD_ReadRegister(FIFOLevelReg);	// Number of bytes in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;											// Number of bytes returned
		PCD_ReadRegister_n(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = PCD_ReadRegister(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}
	
	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_COLLISION;
	}
	
	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		uint8_t controlBuffer[2];
		StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}
	
	return STATUS_OK;
} // End PCD_CommunicateWithPICC()


/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_RequestA(	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
						    uint8_t *bufferSize	    ///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
) {
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_WakeupA(	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
							uint8_t *bufferSize	    ///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
) {
	return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
StatusCode PICC_REQA_or_WUPA(	uint8_t command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
								uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
								uint8_t *bufferSize	    ///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
											) {
	uint8_t validBits;
	StatusCode status;
	
	if (bufferATQA == nullptr || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits,0,false);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End PICC_REQA_or_WUPA()


/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
uint8_t PICC_Select(	Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
						uint8_t validBits	///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
) {
	uint8_t uidComplete;
	uint8_t selectDone;
	uint8_t useCascadeTag;
	uint8_t cascadeLevel = 1;
	StatusCode result;
	uint8_t count;
	uint8_t checkBit;
	uint8_t index;
	uint8_t uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	uint8_t bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	uint8_t *responseBuffer;
	uint8_t responseLength;
	
	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9
	
	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}
	
	// Prepare MFRC522
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;
			
			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;
			
			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;
			
			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}
		
		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}
		
		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}
			
			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			
			// Transmit the buffer and receive the response.
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, false);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				uint8_t valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision 
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)
		
		// We do not check the CBB - it was constructed by us above.
		
		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}
		
		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)
	
	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
} // End PICC_Select()


/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
StatusCode PICC_HaltA() {
	StatusCode result;
	uint8_t buffer[4];
	
	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}
	
	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result = PCD_TransceiveData(buffer, sizeof(buffer), nullptr, 0,0,0,0);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
} // End PICC_HaltA()


/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 * 
 * @return bool
 */
uint8_t PICC_IsNewCardPresent() {
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);

	// Reset baud rates
	PCD_WriteRegister(TxModeReg, 0x00);
	PCD_WriteRegister(RxModeReg, 0x00);
	// Reset ModWidthReg
	PCD_WriteRegister(ModWidthReg, 0x26);

	StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);
	return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 * 
 * @return bool
 */
uint8_t PICC_ReadCardSerial() {
	StatusCode result = PICC_Select(&uid,0);
	return (result == STATUS_OK);
} // End 

void PICC_DumpToSerial(Uid *uid	///< Pointer to Uid struct returned from a successful PICC_Select().
) {
    MIFARE_Key key;
    
    // Dump UID, SAK and Type
    PICC_DumpDetailsToSerial(uid);
#if 0
    // Dump contents
    PICC_Type piccType = PICC_GetType(uid->sak);
    switch (piccType) {
        case PICC_TYPE_MIFARE_MINI:
        case PICC_TYPE_MIFARE_1K:
        case PICC_TYPE_MIFARE_4K:
        // All keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
        for (uint8_t i = 0; i < 6; i++) {
            key.keyByte[i] = 0xFF;
        }
        PICC_DumpMifareClassicToSerial(uid, piccType, &key);
        break;
        
        case PICC_TYPE_MIFARE_UL:
        PICC_DumpMifareUltralightToSerial();
        break;
        
        case PICC_TYPE_ISO_14443_4:
        case PICC_TYPE_MIFARE_DESFIRE:
        case PICC_TYPE_ISO_18092:
        case PICC_TYPE_MIFARE_PLUS:
        case PICC_TYPE_TNP3XXX:
        Serial.println(F("Dumping memory contents not implemented for that PICC type."));
        break;
        
        case PICC_TYPE_UNKNOWN:
        case PICC_TYPE_NOT_COMPLETE:
        default:
        break; // No memory dump here
    }
    
    Serial.println();
#endif
    
    PICC_HaltA(); // Already done if it was a MIFARE Classic PICC.
} // End PICC_DumpToSerial()

/**
 * Translates the SAK (Select Acknowledge) to a PICC type.
 * 
 * @return PICC_Type
 */
PICC_Type PICC_GetType(uint8_t sak		///< The SAK byte returned from PICC_Select().
										) {
	// http://www.nxp.com/documents/application_note/AN10833.pdf 
	// 3.2 Coding of Select Acknowledge (SAK)
	// ignore 8-bit (iso14443 starts with LSBit = bit 1)
	// fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
	sak &= 0x7F;
	switch (sak) {
		case 0x04:	return PICC_TYPE_NOT_COMPLETE;	// UID not complete
		case 0x09:	return PICC_TYPE_MIFARE_MINI;
		case 0x08:	return PICC_TYPE_MIFARE_1K;
		case 0x18:	return PICC_TYPE_MIFARE_4K;
		case 0x00:	return PICC_TYPE_MIFARE_UL;
		case 0x10:
		case 0x11:	return PICC_TYPE_MIFARE_PLUS;
		case 0x01:	return PICC_TYPE_TNP3XXX;
		case 0x20:	return PICC_TYPE_ISO_14443_4;
		case 0x40:	return PICC_TYPE_ISO_18092;
		default:	return PICC_TYPE_UNKNOWN;
	}
} // End PICC_GetType()

/**
 * Returns a __FlashStringHelper pointer to the PICC type name.
 * 
 * @return const __FlashStringHelper *
 */
const char* PICC_GetTypeName(PICC_Type piccType	///< One of the PICC_Type enums.
													) {
	switch (piccType) {
		case PICC_TYPE_ISO_14443_4:		return ("PICC compliant with ISO/IEC 14443-4");
		case PICC_TYPE_ISO_18092:		return ("PICC compliant with ISO/IEC 18092 (NFC)");
		case PICC_TYPE_MIFARE_MINI:		return ("MIFARE Mini, 320 bytes");
		case PICC_TYPE_MIFARE_1K:		return ("MIFARE 1KB");
		case PICC_TYPE_MIFARE_4K:		return ("MIFARE 4KB");
		case PICC_TYPE_MIFARE_UL:		return ("MIFARE Ultralight or Ultralight C");
		case PICC_TYPE_MIFARE_PLUS:		return ("MIFARE Plus");
		case PICC_TYPE_MIFARE_DESFIRE:	return ("MIFARE DESFire");
		case PICC_TYPE_TNP3XXX:			return ("MIFARE TNP3XXX");
		case PICC_TYPE_NOT_COMPLETE:	return ("SAK indicates UID is not complete.");
		case PICC_TYPE_UNKNOWN:
		default:						return ("Unknown type");
	}
} // End PICC_GetTypeName()

/**
 * Dumps debug info about the connected PCD to Serial.
 * Shows all known firmware versions
 */
int8_t PCD_DumpVersionToSerial() {
	// Get the MFRC522 firmware version
	uint8_t v = PCD_ReadRegister(VersionReg);
	LOG_str("Firmware Version: 0x");
    sprintf(strDedug,"%02x",v);
    LOG_str(strDedug);
    
	// Lookup which version
	switch(v) {
		case 0x88: LOG_str(" = (clone)\r\n");  break;
		case 0x90: LOG_str(" = v0.0\r\n");     break;
		case 0x91: LOG_str(" = v1.0\r\n");     break;
		case 0x92: LOG_str(" = v2.0\r\n");     break;
		case 0x12: LOG_str(" = counterfeit chip\r\n");     break;
		default:   LOG_str(" = (unknown)\r\n");
	}
	// When 0x00 or 0xFF is returned, communication probably failed
	if ((v == 0x00) || (v == 0xFF))
		LOG_str("WARNING: Communication failure, is the MFRC522 properly connected?\r\n");
    return v;
} // End PCD_DumpVersionToSerial()


/**
 * Dumps card info (UID,SAK,Type) about the selected PICC to Serial.
 */
void PICC_DumpDetailsToSerial(Uid *uid	///< Pointer to Uid struct returned from a successful PICC_Select().
									) {
	// UID
	LOG_str("Card UID:");
	for (uint8_t i = 0; i < uid->size; i++) {
        sprintf(strDedug," %02x",uid->uidByte[i]);
        LOG_str(strDedug);
	} 
    LOG_str("\r\n");
	
	// SAK
	LOG_str("Card SAK: ");
	if(uid->sak < 0x10)
		LOG_str("0");
        
    sprintf(strDedug,"%02x\r\n",uid->sak);
    LOG_str(strDedug);
	
	// (suggested) PICC type
	PICC_Type piccType = PICC_GetType(uid->sak);
	LOG_str("PICC type: ");
	LOG_str(PICC_GetTypeName(piccType));
} // End PICC_DumpDetailsToSerial()



int8_t RFID_GetFirmwareVersion(){
    return PCD_ReadRegister(VersionReg);
}
