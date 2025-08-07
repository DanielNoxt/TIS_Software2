/*
 * AD9106.h
 *
 *  Created on: Apr 29, 2024
 *      Author: D.Claassen
 */

#ifndef INC_AD9106_H_
#define INC_AD9106_H_

#include "main.h"
#include "AppErrorHandling.h"

/* Benötigte Funktionen
 * - Wavefom auf Sinus setzen	https://github.com/luk6xff/DevLibs/blob/master/AD9106/AD9106.c#L476 Zeile 500
 * - Frequenz setzen
 * - Gain pro Channel
 * - Delay pro Channel
 */
#define AD9106_F_CLK 156250000U		//Clock Input
#define AD9106_SPI_TIMEOUT 100

//TODO: Prescaler fpr CAL prüfen. 000 = 512 Teiler?

typedef struct __AD9106_HandleTypeDef
{
	/* SPI Interface */
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef* CS_Port;
	uint16_t CS_Pin;
	GPIO_TypeDef* Reset_Port;
	uint16_t Reset_Pin;
	GPIO_TypeDef* Trigger_Port;
	uint16_t Trigger_Pin;
} AD9106_HandleTypeDef;

void AD9106_Init(AD9106_HandleTypeDef *hAD9106);
void AD9106_set_DACx_digital_gain_float(AD9106_HandleTypeDef *hAD9106, float* fDACx_Dig_Gain);
void AD9106_set_DACx_digital_gain(AD9106_HandleTypeDef *hAD9106, int16_t* iDACx_Dig_Gain);
void AD9106_set_DSS_Freq_in_Hz(AD9106_HandleTypeDef *hAD9106, float fDSS_Freq);
void AD9106_set_DSS_Freq(AD9106_HandleTypeDef *hAD9106, uint32_t uDSS_Freq);
void AD9106_set_DSS_Phase_in_degrees(AD9106_HandleTypeDef *hAD9106, float* fDSS_Phase);
void AD9106_set_DSS_Phase(AD9106_HandleTypeDef *hAD9106, uint16_t* uDSS_Phase);
void AD9106_set_waveform_sinus(AD9106_HandleTypeDef *hAD9106);
void AD9106_Reset_IOUTFSx_calibration(AD9106_HandleTypeDef *hAD9106); //ungetestet
void AD9106_IOUTFSx_calibration(AD9106_HandleTypeDef *hAD9106); //ungetestet
void AD9106_RAM_Update(AD9106_HandleTypeDef *hAD9106);

typedef enum {
	SPICONFIG = 0x00,
	POWERCONFIG,
	CLOCKCONFIG,
	REFADJ,
	DAC4AGAIN,
	DAC3AGAIN,
	DAC2AGAIN,
	DAC1AGAIN,
	DACxRANGE,
	DAC4RSET,
	DAC3RSET,
	DAC2RSET,
	DAC1RSET,
	CALCONFIG,
	COMPOFFSET = 0x000E,
	RAMUPDATE = 0x001D,
	PAT_STATUS,
	PAT_TYPE,
	PATTERN_DLY = 0x0020,
	DAC4DOF = 0x0022,
	DAC3DOF,
	DAC2DOF,
	DAC1DOF,
	WAV4_3CONFIG,
	WAV2_1CONFIG,
	PAT_TIMEBASE,
	PAT_PERIOD = 0x0029,
	DAC4_3PATx,
	DAC2_1PATx,
	DOUT_START_DLY,
	DOUT_CONFIG,
	DAC4_CST,
	DAC3_CST,
	DAC2_CST,
	DAC1_CST,
	DAC4_DGAIN,
	DAC3_DGAIN,
	DAC2_DGAIN,
	DAC1_DGAIN,
	SAW4_3CONFIG,
	SAW2_1CONFIG,
	DDS_TW32 = 0x3E,
	DDS_TW1,
	DDS4_PW,
	DDS3_PW,
	DDS2_PW,
	DDS1_PW,
	TRIG_TW_SEL,
	DDSx_CONFIG,
	TW_RAM_CONFIG = 0x47,
	START_DLY4 = 0x50,
	START_ADDR4,
	STOP_ADDR4,
	DDS_CYC4,
	START_DLY3,
	START_ADDR3,
	STOP_ADDR3,
	DDS_CYC3,
	START_DLY2,
	START_ADDR2,
	STOP_ADDR2,
	DDS_CYC2,
	START_DLY1,
	START_ADDR1,
	STOP_ADDR1,
	DDS_CYC1,
	CFG_ERROR,
	SRAMDATA = 0x6000,   //0x6000 to 0x6FFF
} AD9106_RegAddress;


typedef enum {
	LSBFIRST = 0x8000, //LSB first selection. {{ 0- MSB first per SPI standard (default). 1- LSB first per SPI standard.
	SPI3WIRE = 0x4000, //Selects if SPI is using 3-wire or 4-wire interface. {{0 - 4-wire SPI.  1- 3-wire SPI.
	RESET_SOFT = 0x2000, //Executes software reset of SPI and controllers, reloads default register values, except for Register 0x00.
	DOUBLESPI = 0x1000, //Double SPI data line.
	SPI_DRV = 0x0800,   //Double-drive ability for SPI output.
	DOUT_EN = 0x0400,   //Enables DOUT signal on SDO/SDI2/DOUT pin.
	DOUT_ENM = 0x0020,  //Enable DOUT signal on SDO/SDI2/DOUT pin.
	SPI_DRVM = 0x0010,  //Double-drive ability for SPI output.
	DOUBLESPIM = 0x0008,  //Double SPI data line.
	RESETM = 0x0004, //Executes software reset of SPI and controllers, reloads default register values, except for Register 0x00.
	SPI3WIREM = 0x0002, //Selects if SPI is using 3-wire or 4-wire interface.
	LSBFIRSTM = 0x0001  //LSB first selection.
} AD9106_SPICONFIG_REG;
/*NOTE!!!SPICONFIG[10:15] should always be set to the mirror of SPICONFIG[5:0] to allow easy recovery of the SPI operation when
 * the LSBFIRST bit is set incorrectly. Bit[15] = Bit[0], Bit[14] = Bit[1], Bit[13] = Bit[2], Bit[12] = Bit[3], Bit[11] = Bit[4] and Bit[10] = Bit[5].*/

typedef enum {
    LSBFIRST_POS = 15,
    SPI3WIRE_POS = 14,
    RESET_SOFT_POS = 13,
    DOUBLESPI_POS = 12,
    SPI_DRV_POS = 11,
    DOUT_EN_POS = 10,
    DOUT_ENM_POS = 5,
    SPI_DRVM_POS = 4,
    DOUBLESPIM_POS = 3,
    RESETM_POS = 2,
    SPI3WIREM_POS = 1,
    LSBFIRSTM_POS = 0
} AD9106_SPICONFIG_POS;

typedef enum {
	CLK_LDO_STAT = 0x0800,  //Read only flag indicating CLKVDD_1P8 LDO is on.
	DIG1_LDO_STAT = 0x0400, //Read only flag indicating DVDD1 LDO is on.
	DIG2_LDO_STAT = 0x0200, //Read only flag indicating DVDD2 LDO is on.
	PDN_LDO_CLK = 0x0100, //Disables the CLKVDD_1P8 LDO. An external supply is required.
	PDN_LDO_DIG1 = 0x0080, //Disables the DVDD1 LDO. An external supply is required.
	PDN_LDO_DIG2 = 0x0040, //Disables the DVDD2 LDO. An external supply is required.
	REF_PDN = 0x0020, //Disables 10 kOm resistor that creates REFIO voltage. User can drive with external voltage or provide external BG resistor.
	REF_EXT = 0x0010,		//Power down main BG reference including DAC bias.
	DAC1_SLEEP = 0x0008,	//Disables DAC1 output current.
	DAC2_SLEEP = 0x0004,	//Disables DAC2 output current.
	DAC3_SLEEP = 0x0002,	//Disables DAC3 output current.
	DAC4_SLEEP = 0x0001		//Disables DAC4 output current.
} AD9106_POWERCONFIG_REG;

typedef enum {
    CLK_LDO_STAT_POS = 11,
    DIG1_LDO_STAT_POS = 10,
    DIG2_LDO_STAT_POS = 9,
    PDN_LDO_CLK_POS = 8,
    PDN_LDO_DIG1_POS = 7,
    PDN_LDO_DIG2_POS = 6,
    REF_PDN_POS = 5,
    REF_EXT_POS = 4,
    DAC1_SLEEP_POS = 3,
    DAC2_SLEEP_POS = 2,
    DAC3_SLEEP_POS = 1,
    DAC4_SLEEP_POS = 0
} AD9106_POWERCONFIG_POS;

typedef enum {
	DIS_CLK1 = 0x0800,
	DIS_CLK2 = 0x0400,
	DIS_CLK3 = 0x0200,
	DIS_CLK4 = 0x0100,
	DIS_DCLK = 0x0080,
	CLK_SLEEP = 0x0040,
	CLK_PDN = 0x0020,
	EPS = 0x0010,
	DAC1_INV_CLK = 0x0008,
	DAC2_INV_CLK = 0x0004,
	DAC3_INV_CLK = 0x0002,
	DAC4_INV_CLK = 0x0001
} AD9106_CLOCKCONFIG_REG;

typedef enum {
    DIS_CLK1_POS = 11,
    DIS_CLK2_POS = 10,
    DIS_CLK3_POS = 9,
    DIS_CLK4_POS = 8,
    DIS_DCLK_POS = 7,
    CLK_SLEEP_POS = 6,
    CLK_PDN_POS = 5,
    EPS_POS = 4,
    DAC1_INV_CLK_POS = 3,
    DAC2_INV_CLK_POS = 2,
    DAC3_INV_CLK_POS = 1,
    DAC4_INV_CLK_POS = 0
} AD9106_CLOCKCONFIG_POS;

typedef enum {
	BGDR = 0x003F
} AD9106_REFADJ_REG;

typedef enum {
    BGDR_POS = 0  // Uses the full range of 6 bits from 0 to 5
} AD9106_REFADJ_POS;

typedef enum {
	DACx_GAIN_CAL = 0x7F00,  //Read only
	DACx_GAIN = 0x007F
} AD9106_DACxAGAIN_REG;

typedef enum {
    DACx_GAIN_CAL_POS = 8,
    DACx_GAIN_POS = 0
} AD9106_DACxAGAIN_POS;

typedef enum {
	DAC4_GAIN_RNG = 0xC0,
	DAC3_GAIN_RNG = 0x30,
	DAC2_GAIN_RNG = 0x0C,
	DAC1_GAIN_RNG = 0x03
} AD9106_DACxRANGE_REG;

typedef enum {
    DAC4_GAIN_RNG_POS = 6,
    DAC3_GAIN_RNG_POS = 4,
    DAC2_GAIN_RNG_POS = 2,
    DAC1_GAIN_RNG_POS = 0
} AD9106_DACxRANGE_POS;

typedef enum {
	DACx_RSET_EN = 0x8000,
	DACx_RSET_CAL = 0x1F00,
	DACx_RSET = 0x1F
} AD9106_DACxRSET_REG;

typedef enum {
    DACx_RSET_EN_POS = 15,
    DACx_RSET_CAL_POS = 8,
    DACx_RSET_POS = 0
} AD9106_DACxRSET_POS;

typedef enum {
	COMP_OFFSET_OF = 0x4000,   //Read only
	COMP_OFFSET_UF = 0x2000,   //Read only
	RSET_CAL_OF = 0x1000,      //Read only
	RSET_CAL_UF = 0x0800,      //Read only
	GAIN_CAL_OF = 0x0400,      //Read only
	GAIN_CAL_UF = 0x0200,      //Read only
	CAL_RESET = 0x0100,
	CAL_MODE = 0x0080,         //Read only
	CAL_MODE_EN = 0x0040,
	COMP_CAL_RNG = 0x0030,
	CAL_CLK_EN = 0x0008,
	CAL_CLK_DIV = 0x0007
} AD9106_CALCONFIG_REG;

typedef enum {
    COMP_OFFSET_OF_POS = 14,
    COMP_OFFSET_UF_POS = 13,
    RSET_CAL_OF_POS = 12,
    RSET_CAL_UF_POS = 11,
    GAIN_CAL_OF_POS = 10,
    GAIN_CAL_UF_POS = 9,
    CAL_RESET_POS = 8,
    CAL_MODE_POS = 7,
    CAL_MODE_EN_POS = 6,
    COMP_CAL_RNG_POS = 4,
    CAL_CLK_EN_POS = 3,
    CAL_CLK_DIV_POS = 0
} AD9106_CALCONFIG_POS;

typedef enum {
	COMP_OFFSET_CAL = 0x7F00, //Read only
	CAL_FIN = 0x0002,         //Read only
	START_CAL = 0x0001
} AD9106_COMPOFFSET_REG;

typedef enum {
    COMP_OFFSET_CAL_POS = 8,
    CAL_FIN_POS = 1,
    START_CAL_POS = 0
} AD9106_COMPOFFSET_POS;

typedef enum {
	RAMPUPDATE = 0x0001
} AD9106_RAMUPDATE_REG;

typedef enum {
    RAMPUPDATE_POS = 0
} AD9106_RAMUPDATE_POS;

typedef enum {
	BUF_READ = 0x0008,
	MEM_ACCESS = 0x0004,
	PATTERN = 0x0002,        //Read only
	RUN = 0x0001
} AD9106_PAT_STATUS_REG;

typedef enum {
    BUF_READ_POS = 3,
    MEM_ACCESS_POS = 2,
    PATTERN_POS = 1,
    RUN_POS = 0
} AD9106_PAT_STATUS_POS;

typedef enum {
	PATTERN_RPT = 0x0001
} AD9106_PAT_TYPE_REG;

typedef enum {
    PATTERN_RPT_POS = 0
} AD9106_PAT_TYPE_POS;


typedef enum {
	PATTERN_DELAY = 0xFFFF
} AD9106_PATTERN_DLY_REG;

typedef enum {
    PATTERN_DELAY_POS = 0  // Uses the full range of 16 bits
} AD9106_PATTERN_DLY_POS;

typedef enum {
	DACx_DIG_OFFSET = 0xFFF0
} AD9106_DACxDOF_REG;

typedef enum {
    DACx_DIG_OFFSET_POS = 4
} AD9106_DACxDOF_POS;

typedef enum {
	PRESTORE_SEL4 = 0x3000,
	WAVE_SEL4 = 0x0300,
	PRESTORE_SEL3 = 0x30,
	WAVE_SEL3 = 0x03,
} AD9106_WAV4_3CONFIG_REG;

typedef enum {
    PRESTORE_SEL4_POS = 12,
    WAVE_SEL4_POS = 8,
    PRESTORE_SEL3_POS = 4,
    WAVE_SEL3_POS = 0
} AD9106_WAV4_3CONFIG_POS;

typedef enum {
	PRESTORE_SEL2 = 0x3000,
	MASK_DAC4 = 0x0800,
	CH2_ADD = 0x0400,
	WAVE_SEL2 = 0x0300,
	PRESTORE_SEL1 = 0x30,
	MASK_DAC3 = 0x08,
	CH1_ADD = 0x04,
	WAVE_SEL1 = 0x03,
} AD9106_WAV2_1CONFIG_REG;

typedef enum {
    PRESTORE_SEL2_POS = 12,
    MASK_DAC4_POS = 11,
    CH2_ADD_POS = 10,
    WAVE_SEL2_POS = 8,
    PRESTORE_SEL1_POS = 4,
    MASK_DAC3_POS = 3,
    CH1_ADD_POS = 2,
    WAVE_SEL1_POS = 0
} AD9106_WAV2_1CONFIG_POS;

typedef enum {
	HOLD = 0x0F00,
	PAT_PERIOD_BASE = 0xF0,
	START_DELAY_BASE = 0x0F
} AD9106_PAT_TIMEBASE_REG;

typedef enum {
    HOLD_POS = 8,
    PAT_PERIOD_BASE_POS = 4,
    START_DELAY_BASE_POS = 0
} AD9106_PAT_TIMEBASE_POS;

typedef enum {
	PATTERN_PERIOD = 0xFFFF
} AD9106_PAT_PERIOD_REG;

typedef enum {
    PATTERN_PERIOD_POS = 0  // Uses the full range of 16 bits
} AD9106_PAT_PERIOD_POS;

typedef enum {
	DAC4_REPEAT_CYCLE = 0xFF00,
	DAC3_REPEAT_CYCLE = 0xFF
} AD9106_DAC4_3PATx_REG;

typedef enum {
    DAC4_REPEAT_CYCLE_POS = 8,
    DAC3_REPEAT_CYCLE_POS = 0
} AD9106_DAC4_3PATx_POS;

typedef enum {
	DAC2_REPEAT_CYCLE = 0xFF00,
	DAC1_REPEAT_CYCLE = 0xFF
} AD9106_DAC2_1PATx_REG;

typedef enum {
    DAC2_REPEAT_CYCLE_POS = 8,
    DAC1_REPEAT_CYCLE_POS = 0
} AD9106_DAC2_1PATx_POS;

typedef enum {
	DOUT_START = 0xFFFF,
} AD9106_DOUT_START_DLY_REG;

typedef enum {
    DOUT_START_POS = 0  // Uses the full range of 16 bits
} AD9106_DOUT_START_DLY_POS;

typedef enum {
	DOUT_VAL = 0x20,
	DOUT_MODE = 0x10,
	DOUT_STOP = 0x0F
} AD9106_DOUT_CONFIG_REG;

typedef enum {
    DOUT_VAL_POS = 5,
    DOUT_MODE_POS = 4,
    DOUT_STOP_POS = 0
} AD9106_DOUT_CONFIG_POS;

typedef enum {
	DACx_CONST = 0xFFF0
} AD9106_DACx_CST_REG;

typedef enum {
    DACx_CONST_POS = 4
} AD9106_DACx_CST_POS;

typedef enum {
	DACx_DIG_GAIN = 0xFFF0
} AD9106_DACx_DGAIN_REG;

typedef enum {
    DACx_DIG_GAIN_POS = 4
} AD9106_DACx_DGAIN_POS;

typedef enum {
	SAW_STEP4 = 0xFC00,  //Number of samples per step for DAC4.
	SAW_TYPE4 = 0x0300, //0- Ramp up saw wave. 1- Ramp down saw wave. 2- Triangle saw wave. 3- No wave, zero.
	SAW_STEP3 = 0x00FC,
	SAW_TYPE3 = 0x0003
} AD9106_SAW4_3CONFIG_REG;

typedef enum {
    SAW_STEP4_POS = 10,
    SAW_TYPE4_POS = 8,
    SAW_STEP3_POS = 2,
    SAW_TYPE3_POS = 0
} AD9106_SAW4_3CONFIG_POS;

typedef enum {
	SAW_STEP2 = 0xFC00,  //Number of samples per step for DAC2.
	SAW_TYPE2 = 0x0300, //0- Ramp up saw wave. 1- Ramp down saw wave. 2- Triangle saw wave. 3- No wave, zero.
	SAW_STEP1 = 0x00FC,
	SAW_TYPE1 = 0x0003
} AD9106_SAW2_1CONFIG_REG;

typedef enum {
    SAW_STEP2_POS = 10,
    SAW_TYPE2_POS = 8,
    SAW_STEP1_POS = 2,
    SAW_TYPE1_POS = 0
} AD9106_SAW2_1CONFIG_POS;

///////////////
typedef enum {
	DDSTW_MSB = 0xFFFF  //DDS tuning word MSB.
} AD9106_DDS_TW32_REG;

typedef enum {
    DDSTW_MSB_POS = 0  // Uses the full range of 16 bits
} AD9106_DDS_TW32_POS;

typedef enum {
	DDSTW_LSB = 0xFF00  //DDS tuning word LSB.
} AD9106_DDS_TW1_REG;

typedef enum {
    DDSTW_LSB_POS = 8
} AD9106_DDS_TW1_POS;

typedef enum {
	DDSx_PHASE = 0xFFFF //DDSx phase offset.
} AD9106_DDSx_PW_REG;

typedef enum {
    DDSx_PHASE_POS = 0  // Uses the full range of 16 bits
} AD9106_DDSx_PW_POS;

typedef enum {
	TRIG_DELAY_EN = 0x0002 //Enable start delay as trigger delay for all four channels.
	/* Settings
	 0 Delay repeats for all patterns.
	 1 Delay is only at the start of first pattern.
	 */
} AD9106_TRIG_TW_SEL_REG;

typedef enum {
    TRIG_DELAY_EN_POS = 1
} AD9106_TRIG_TW_SEL_POS;

typedef enum {
	DDS_COS_EN4 = 0x8000, //Enable DDS4 cosine output of DDS instead of sine wave.
	DDS_MSB_EN4 = 0x4000, //Enable the clock for the RAM address. Increment is coming from the DDS4 MSB. Default is coming from DAC clock.
	DDS_COS_EN3 = 0x0800, //Enable DDS3 cosine output of DDS instead of sine wave.
	DDS_MSB_EN3 = 0x0400, //Enable the clock for the RAM address. Increment is coming from the DDS3 MSB. Default is coming from DAC clock.
	PHASE_MEM_EN3 = 0x0200, //Enable DDS3 phase offset input coming from RAM reading START_ADDR3. Since phase word is 8 bits and RAM data is 14 bits, only 8 MSB of RAM are taken into account. Default is coming from SPI map, DDS3_PHASE.
	DDS_COS_EN2 = 0x0080, //Enable DDS2 cosine output of DDS instead of sine wave.
	DDS_MSB_EN2 = 0x0040, //Enable the clock for the RAM address. Increment is coming from the DDS2 MSB. Default is coming from DAC clock.
	DDS_MSB_EN1 = 0x0004, //Enable the clock for the RAM address. Increment is coming from the DDS1 MSB. Default is coming from DAC clock.
	TW_MEM_EN = 0x0001 //Enable DDS tuning word input coming from RAM reading using START_ADDR1. Since tuning word is 24 bits and RAM data is 14 bits, 10 bits are set to 0s depending on the value of the TW_MEM_SHIFT bits in the TW_RAM_CONFIG REGister. Default is coming from SPI map, DDSTW.
} AD9106_DDSx_CONFIG_REG;

typedef enum {
    DDS_COS_EN4_POS = 15,
    DDS_MSB_EN4_POS = 14,
    DDS_COS_EN3_POS = 11,
    DDS_MSB_EN3_POS = 10,
    PHASE_MEM_EN3_POS = 9,
    DDS_COS_EN2_POS = 7,
    DDS_MSB_EN2_POS = 6,
    DDS_MSB_EN1_POS = 2,
    TW_MEM_EN_POS = 0
} AD9106_DDSx_CONFIG_POS;

typedef enum {
	TW_MEM_SHIFT = 0x000F
//settings NOTE! TW_MEM_EN1 must be set = 1 to use this bit field.
/*
 0x00   DDS1TW = {RAM[11:0],12'b0}
 0x01   DDS1TW = {DDS1TW[23],RAM[11:0],11'b0}
 0x02	DDS1TW = {DDS1TW[23:22],RAM[11:0],10'b0}
 0x03	DDS1TW = {DDS1TW[23:21],RAM[11:0],9'b0}
 0x04	DDS1TW = {DDS1TW[23:20],RAM[11:0],8'b0}
 0x05	DDS1TW = {DDS1TW[23:19],RAM[11:0],7'b0}
 0x06	DDS1TW = {DDS1TW[23:18],RAM[11:0],6'b0}
 0x07	DDS1TW = {DDS1TW[23:17],RAM[11:0],5'b0}
 0x08	DDS1TW = {DDS1TW[23:16],RAM[11:0],3'b0}
 0x09	DDS1TW = {DDS1TW[23:15],RAM[11:0],4'b0}
 0x0A	DDS1TW = {DDS1TW[23:14],RAM[11:0],2b0}
 0x0B	DDS1TW = {DDS1TW[23:13],RAM[11:0],1b0}
 0x0C	DDS1TW = {DDS1TW[23:12],RAM[11:0]}
 0x0D	DDS1TW = {DDS1TW[23:11],RAM[11:1]}
 0x0E	DDS1TW = {DDS1TW[23:10],RAM[11:2]}
 0x0F	DDS1TW = {DDS1TW[23:9],RAM[11:3]}
 0x10	DDS1TW = {DDS1TW[23:8],RAM[11:4]}
 x Reserved
 */
} AD9106_TW_RAM_CONFIG_REG;

typedef enum {
    TW_MEM_SHIFT_POS = 0  // Uses the full range of 4 bits
} AD9106_TW_RAM_CONFIG_POS;

typedef enum {
	START_DELAYx = 0xFFFF,  //Start delay of DACx.
} AD9106_START_DLYx_REG;

typedef enum {
    START_DELAYx_POS = 0  // Uses the full range of 16 bits
} AD9106_START_DLYx_POS;

typedef enum {
	START_ADDRx = 0xFFF0,  //RAM address where DACx starts to read waveform.
} AD9106_START_ADDRx_REG;

typedef enum {
    START_ADDRx_POS = 4
} AD9106_START_ADDRx_POS;

typedef enum {
	STOP_ADDRx = 0xFFF0,  //RAM address where DACx stops to read waveform.
} AD9106_STOP_ADDRx_REG;

typedef enum {
    STOP_ADDRx_POS = 4
} AD9106_STOP_ADDRx_POS;

typedef enum {
	DDS_CYCx = 0xFFFF // Number of sine wave cycles when DDS prestored waveform with start and stop delays is selected for DACx output.
} AD9106_DDS_CYCx_REG;

typedef enum {
    DDS_CYCx_POS = 0  // Uses the full range of 16 bits
} AD9106_DDS_CYCx_POS;

typedef enum {
	ERROR_CLEAR = 0x8000,  		  //Writing this bit clears all errors.
	CFG_ERROR_ = 0x7FC0,
	DOUT_START_LG_ERR = 0x0020, //When DOUT_START is larger than pattern delay, this error is toggled.  /R only
	PAT_DLY_SHORT_ERR = 0x0010, //When pattern delay value is smaller than default value, this error is toggled. /R only
	DOUT_START_SHORT_ERR = 0x0008, //When DOUT_START value is smaller than default value, this error is toggled. /R only
	PERIOD_SHORT_ERR = 0x0004, //When period register setting value is smaller than pattern play cycle, this error is toggled. /R only
	ODD_ADDR_ERR = 0x0002, //When memory pattern play is not even in length in trigger delay mode, this error flag is toggled. /R only
	MEM_READ_ERR = 0x0001 //When there is a memory read conflict, this error flag is toggled. /R only
} AD9106_CFG_ERROR_REG;

typedef enum {
    ERROR_CLEAR_POS = 15,
    CFG_ERROR_POS = 6,
    DOUT_START_LG_ERR_POS = 5,
    PAT_DLY_SHORT_ERR_POS = 4,
    DOUT_START_SHORT_ERR_POS = 3,
    PERIOD_SHORT_ERR_POS = 2,
    ODD_ADDR_ERR_POS = 1,
    MEM_READ_ERR_POS = 0
} AD9106_CFG_ERROR_POS;

//"Interne Funktionen
void AD9106_read_n_Registers(AD9106_HandleTypeDef *hAD9106, AD9106_RegAddress reg_addr, uint16_t reg_data_size, uint16_t* reg_data);
void AD9106_write_n_Registers(AD9106_HandleTypeDef *hAD9106, AD9106_RegAddress reg_addr, uint16_t reg_data_size, uint16_t* reg_data);
uint16_t AD9106_readRegister(AD9106_HandleTypeDef *hAD9106, AD9106_RegAddress reg_addr);
void AD9106_writeRegister(AD9106_HandleTypeDef *hAD9106, AD9106_RegAddress reg_addr, uint16_t reg_data);


#endif /* INC_AD9106_H_ */
