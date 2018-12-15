/**
 * \file
 *         GetTimeOnAir.c
 * \description
 *         Calculate the time on air by: Payload-Length, BR-On-Air;
 * \author
 *         JiangJun
 * \date
 *         2016-11-29 14:16
 * \copyright
 *         (c) 2016-2020 RimeLink (www.rimelink.com) All Rights Reserved.
 */


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>
#include <assert.h>

/* Private typedef -----------------------------------------------------------*/
#define ASSERT(cond)    assert(cond)

typedef enum {FALSE = 0, TRUE = !FALSE} bool;


/**
* @brief  sort of systme.
*/
typedef enum
{
    RF_SPEED_INVALID_MIN = (uint8_t)0,
    RF_SPEED_LOW,
    RF_SPEED_MID,
    RF_SPEED_HIGH,    
    RF_SPEED_INVALID_MAX,
} RF_SPEED_Typedef;

/**
* @brief  sort of systme.
*/
typedef enum
{
    SYST_INVALID_MIN = (uint8_t)0,
    SYST_TDMA,
    SYST_TDMA_WAKE,
    SYST_INVALID_MAX,
} SYST_SORT_Typedef;

/**
 * @brief  Spreading Factor of LoRa settings
 */
typedef enum
{
    RF_SF_6 = (uint8_t)6,
    RF_SF_7,
    RF_SF_8,
    RF_SF_9,
    RF_SF_10,
    RF_SF_11,
    RF_SF_12,
} RadioSF_t;

/**
 * @brief  Forward Error Correction of LoRa settings
 */
typedef enum
{
    RF_FEC_4_5 = (uint8_t)1,
    RF_FEC_4_6,
    RF_FEC_4_7,
    RF_FEC_4_8,
} RadioFEC_t;

/**
 * @brief  Bandwidth of LoRa settings
 */
typedef enum
{
    RF_BW_7800 = (uint8_t)0,
    RF_BW_10400,
    RF_BW_15600,    
    RF_BW_20800,    
    RF_BW_31250,    
    RF_BW_41700,    
    RF_BW_62500,    
    RF_BW_125000,    
    RF_BW_250000,    
    RF_BW_500000,    
} RadioBW_t;

/**
* @brief  Radio LoRa modem parameters
*/
typedef struct
{
    uint16_t    wPreambleLen;
    RadioBW_t    tBW;
    RadioSF_t    tSF;
    RadioFEC_t    tFEC;
    bool    bLowDatarateOptimize;
    bool    bFixLen;
    bool    bCrcOn;
} RadioLoRaSettings_t;

typedef struct
{
    RadioBW_t    eBW;
    RadioSF_t    eSF;
    RadioFEC_t    eFEC;
} BW_SF_FEC;


/* Private macro -------------------------------------------------------------*/
#define MIN_PREAMBLE_TIME    20 /* 20ms */
#define MIN_PREAMBLE_SIZE    8


/* Private variables ---------------------------------------------------------*/
static RadioLoRaSettings_t    s_stLoRaSettings = 
{
    .wPreambleLen = 6,
    .bLowDatarateOptimize = FALSE,
    .bFixLen = FALSE,
    .bCrcOn = TRUE,
};

/* Private function prototypes -----------------------------------------------*/

/* Private Constants ---------------------------------------------------------*/
/**
* @brief  Table for mapping speed and BW+SF_FEC.
*/
const static BW_SF_FEC    s_astBwSfFec[] =
{
    {RF_BW_31250, RF_SF_12, RF_FEC_4_5}, /* BR_ON_AIR_66 */
    {RF_BW_62500, RF_SF_12, RF_FEC_4_5}, /* BR_ON_AIR_132 */
    {RF_BW_62500, RF_SF_11, RF_FEC_4_5}, /* BR_ON_AIR_243 */
    {RF_BW_62500, RF_SF_10, RF_FEC_4_5}, /* BR_ON_AIR_443 */
    {RF_BW_125000, RF_SF_10, RF_FEC_4_5}, /* BR_ON_AIR_887 */
    {RF_BW_125000, RF_SF_9, RF_FEC_4_5}, /* BR_ON_AIR_1602 */
    {RF_BW_125000, RF_SF_8, RF_FEC_4_5}, /* BR_ON_AIR_2876 */
    {RF_BW_125000, RF_SF_7, RF_FEC_4_5}, /* BR_ON_AIR_5084 */
    {RF_BW_250000, RF_SF_7, RF_FEC_4_5}, /* BR_ON_AIR_10168 */
    {RF_BW_500000, RF_SF_7, RF_FEC_4_5}, /* BR_ON_AIR_20334 */
};

/*!
 * BandWidth used to calculate time of packet and preamble on air.
 */
const static double    s_adBW[] =    \
    {78e2, 104e2, 156e2, 208e2, 312e2, 414e2, 625e2, 125e3, 250e3, 500e3};

static const uint16_t    s_wAckDelay[10] = 
{
    287, 156, 83, 50, 31, 22, 17, 16, 15, 15
};


/**
  * @brief  Calculate time of a packet that TX or RX by radio.
  * @param  byPayload: size of this packet, valid value=[1, 255].
  * @param  bIsFast: TRUE=Fast, FALSE=Normal.
  * @retval  Time of a packet TX by radio that unit is millisecond.
  */
uint16_t GetTimeOnAir(uint8_t byPayload, bool bIsFast)
{
    double    dTS, dPayload;

    /* Get time of symbol rate */
    dTS =  (double)(1 << s_stLoRaSettings.tSF) / s_adBW[s_stLoRaSettings.tBW - RF_BW_7800];

    /* Symbol length of payload */
    dPayload = (double)( 8 * byPayload - 4 * s_stLoRaSettings.tSF + 28 +    \
                                  (s_stLoRaSettings.bCrcOn ? 16 : 0) - (s_stLoRaSettings.bFixLen ? 20 : 0) );
    dPayload /= (double)(4 * s_stLoRaSettings.tSF - (s_stLoRaSettings.bLowDatarateOptimize ? 8 : 0));
    dPayload = ceil(dPayload) * (double)(s_stLoRaSettings.tFEC + 4);
    dPayload = 8.0 + ((dPayload > 0) ? dPayload : 0.0);

    dPayload = dTS * (dPayload + s_stLoRaSettings.wPreambleLen + 4.25) * 1000.0; /* Multi 1000 for Sed=>MS */
    if (!bIsFast)
    {
        dPayload = dPayload * 32 / 26;
    }

    return (uint16_t)ceil(dPayload); /* Return ms secs */
}

/**
  * @brief  Calculate the preamble number according to time.
  * @param  int16_t wMs    millisecond
  * @param  bIsFast: TRUE=Fast, FALSE=Normal
  * @retval  preamble number.
  * @note  the number of preamble may LESS than zero.
  */
int16_t RFCalcPreambleNum(uint16_t wMs, bool bIsFast)
{
    double    dTemp;

    dTemp = s_adBW[s_stLoRaSettings.tBW - RF_BW_7800] / (double)(1 << s_stLoRaSettings.tSF);
    if (bIsFast)
    {
        dTemp = dTemp * wMs / 1000.0 - 4.25;
    }
    else
    {
        dTemp = dTemp * wMs * 26 / 32 / 1000.0 - 4.25;
    }

    return (int16_t)ceil(dTemp);
}


/**
  * @brief  Calculate the MIN slot length.
  * @param  byPayload: size of this packet, valid value=[1, 247].
  * @param  tRFSpeed: LOW, MID, HIGH.
  * @param  bIsFast: TRUE=Fast, FALSE=Normal.
  * @retval  Time of a packet that TX by radio, the unit is millisecond.
  */
uint16_t GetMinSlotLen(uint8_t byPayload, RF_SPEED_Typedef tRFSpeed, bool bIsFast)
{
    /* Add 20ms to separte the late node and early node. */
    #define SEPARATE_UPLINK_LATE_EARLY    20

    int8_t    chPreambleSize;
    uint8_t    byBROnAir;
    uint16_t    wTxData, wRxAck, wMinSlot, wDelay;

    switch (tRFSpeed)
    {
        case RF_SPEED_LOW:
            byBROnAir = 4;
            break;
        case RF_SPEED_MID:
            byBROnAir = 7;
            break;
        case RF_SPEED_HIGH:
            byBROnAir = 10;
            break;
        default:
            ASSERT(!"Bad RF speed.\r\n");
            break;
    }

    /* Convert BROnAir to BW+SF+FEC, subtract 1 because of index start from 0. */
    s_stLoRaSettings.tBW = s_astBwSfFec[byBROnAir - 1].eBW;
    s_stLoRaSettings.tSF = s_astBwSfFec[byBROnAir - 1].eSF;
    s_stLoRaSettings.tFEC = s_astBwSfFec[byBROnAir - 1].eFEC;

    /* Set the appropriate size of preamble */
    chPreambleSize = RFCalcPreambleNum(MIN_PREAMBLE_TIME, bIsFast);
    if (chPreambleSize < MIN_PREAMBLE_SIZE)
    {
        chPreambleSize = MIN_PREAMBLE_SIZE;
    }
    s_stLoRaSettings.wPreambleLen = chPreambleSize;

    wTxData = GetTimeOnAir(4 + byPayload, bIsFast); /* Add 3 for: Flag+NetVer+NodeAddr */
    wRxAck = GetTimeOnAir(26, bIsFast); /* 26=length of AckDiff frame. */

    wDelay = s_wAckDelay[byBROnAir - 1];
    wMinSlot = wTxData + wRxAck + wDelay + SEPARATE_UPLINK_LATE_EARLY;

    return wMinSlot;
}


/**
  * @brief  Calculate the time of wake exchange.
  * @param  byWakeDataSize: size of wake data, valid value=[1, 247].
  * @param  byWakeAckSize: size of wake ack, valid value=[1, 247].
  * @param  tRFSpeed: LOW, MID, HIGH.
  * @param  bIsFast: TRUE=Fast, FALSE=Normal.
  * @retval  Time of wake exchange, the unit is millisecond.
  */  
uint16_t GetTime4WakeExchange(uint8_t byWakeDataSize, uint8_t byWakeAckSize, RF_SPEED_Typedef tRFSpeed, bool bIsFast)
{
    /* Separate time for 2 RF communication as: UPLINK<->WAKE, WAKE<->WAKE. */
    #define SEPARATE_TIME    100 /* 100ms */

    int8_t    chPreambleSize;
    uint8_t    byBROnAir;
    uint8_t    byWakeDataFrame;
    uint8_t    byWakeAckFrame;
    uint16_t    wTime;
    uint16_t    wMin;

    switch (tRFSpeed)
    {
        case RF_SPEED_LOW:
            byBROnAir = 4;
            break;
        case RF_SPEED_MID:
            byBROnAir = 7;
            break;
        case RF_SPEED_HIGH:
            byBROnAir = 10;
            break;
        default:
            ASSERT(!"Bad RF speed.\r\n");
            break;
    }

    /* Convert BROnAir to BW+SF+FEC, subtract 1 because of index start from 0. */
    s_stLoRaSettings.tBW = s_astBwSfFec[byBROnAir - 1].eBW;
    s_stLoRaSettings.tSF = s_astBwSfFec[byBROnAir - 1].eSF;
    s_stLoRaSettings.tFEC = s_astBwSfFec[byBROnAir - 1].eFEC;

    /* Set the appropriate size of preamble */
    chPreambleSize = RFCalcPreambleNum(MIN_PREAMBLE_TIME, bIsFast);
    if (chPreambleSize < MIN_PREAMBLE_SIZE)
    {
        chPreambleSize = MIN_PREAMBLE_SIZE;
    }
    s_stLoRaSettings.wPreambleLen = chPreambleSize;

    byWakeDataFrame = byWakeDataSize + 1;
    byWakeAckFrame = byWakeAckSize + 3;

    wTime = SEPARATE_TIME + s_wAckDelay[byBROnAir - 1];
    wTime += GetTimeOnAir(3, bIsFast);
    wTime += GetTimeOnAir(byWakeDataFrame, bIsFast);
    wTime += GetTimeOnAir(byWakeAckFrame, bIsFast);

    wMin = SEPARATE_TIME + GetTimeOnAir(3, bIsFast) + GetTimeOnAir(20, bIsFast);
    if (wTime < wMin)
    {
        wTime = wMin;
    }

    return wTime;
}


void main(void)
{
    uint16_t    wMs;

    /* Normal Mode */
    wMs = GetMinSlotLen(100, RF_SPEED_LOW, FALSE);
    wMs = GetTime4WakeExchange(20, 0, RF_SPEED_LOW, FALSE);

    wMs = GetMinSlotLen(247, RF_SPEED_LOW, FALSE);
    wMs = GetMinSlotLen(247, RF_SPEED_MID, FALSE);
    wMs = GetMinSlotLen(247, RF_SPEED_HIGH, FALSE);


    /* Fast Mode */
    wMs = GetMinSlotLen(100, RF_SPEED_LOW, TRUE);
    wMs = GetTime4WakeExchange(20, 0, RF_SPEED_LOW, TRUE);

    wMs = GetMinSlotLen(247, RF_SPEED_LOW, TRUE);
    wMs = GetMinSlotLen(247, RF_SPEED_MID, TRUE);
    wMs = GetMinSlotLen(247, RF_SPEED_HIGH, TRUE);
}


/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0     (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

