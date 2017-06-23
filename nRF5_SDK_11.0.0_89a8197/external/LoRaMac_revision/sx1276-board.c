/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: SX1276 driver specific target board functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
//#include "board.h"
#include <stdint.h>
#include "radio.h"
#include "sx1276/sx1276.h"
#include "sx1276-board.h"
#include "app_util_platform.h"

/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby,
    SX1276SetRx,
    SX1276StartCad,
    SX1276SetTxContinuousWave,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength,
    SX1276SetPublicNetwork
};

/*!
 * Antenna switch GPIO pins objects
 */
//Gpio_t AntSwitchLf;
//Gpio_t AntSwitchHf;

nrf_drv_gpiote_out_config_t AntSwitchLf;
nrf_drv_gpiote_out_config_t AntSwitchHf;

void SX1276IoInit( void )
{
    //GpioInit( &SX1276.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    
		nrf_drv_spi_config_t initTmp = NRF_DRV_SPI_DEFAULT_CONFIG(0);
		SX1276.Spi  = initTmp;
	  
		SX1276.Spi.ss_pin = RADIO_NSS;
	  SX1276.Spi.frequency = NRF_DRV_SPI_FREQ_1M;
    SX1276.Spi.mode      = NRF_DRV_SPI_MODE_3;
    SX1276.Spi.bit_order = NRF_DRV_SPI_BIT_ORDER_LSB_FIRST;
    nrf_drv_spi_t initTmpIns = NRF_DRV_SPI_INSTANCE(0);
	  SX1276.Spi_master = initTmpIns;
    nrf_drv_spi_init(&(SX1276.Spi_master), &(SX1276.Spi), NULL);

    //GpioInit( &SX1276.DIO0, RADIO_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    //GpioInit( &SX1276.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    //GpioInit( &SX1276.DIO2, RADIO_DIO_2, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    //GpioInit( &SX1276.DIO3, RADIO_DIO_3, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    //GpioInit( &SX1276.DIO4, RADIO_DIO_4, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    //GpioInit( &SX1276.DIO5, RADIO_DIO_5, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
		nrf_drv_gpiote_in_config_t cfgTmp = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    SX1276.DIO0 =   cfgTmp;
    SX1276.DIO1 =   cfgTmp;
    SX1276.DIO2 =   cfgTmp;
    SX1276.DIO3 =   cfgTmp;
    SX1276.DIO4 =   cfgTmp;
    SX1276.DIO5 =   cfgTmp;

    SX1276.DIO0.pull = NRF_GPIO_PIN_PULLUP;
    SX1276.DIO1.pull = NRF_GPIO_PIN_PULLUP;
    SX1276.DIO2.pull = NRF_GPIO_PIN_PULLUP;
    SX1276.DIO3.pull = NRF_GPIO_PIN_PULLUP;
    SX1276.DIO4.pull = NRF_GPIO_PIN_PULLUP;
    SX1276.DIO5.pull = NRF_GPIO_PIN_PULLUP;

    uint32_t err_code;
    if(!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
    }
}

void SX1276IoIrqInit( nrf_drv_gpiote_evt_handler_t *irqHandlers )
{
    //GpioSetInterrupt( &SX1276.DIO0, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[0] );
    //GpioSetInterrupt( &SX1276.DIO1, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[1] );
    //GpioSetInterrupt( &SX1276.DIO2, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[2] );
    //GpioSetInterrupt( &SX1276.DIO3, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[3] );
    //GpioSetInterrupt( &SX1276.DIO4, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[4] );
    //GpioSetInterrupt( &SX1276.DIO5, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[5] );
    nrf_drv_gpiote_in_init(RADIO_DIO_0,&(SX1276.DIO0),irqHandlers[0]);
    nrf_drv_gpiote_in_init(RADIO_DIO_1,&(SX1276.DIO1),irqHandlers[1]);
    nrf_drv_gpiote_in_init(RADIO_DIO_2,&(SX1276.DIO2),irqHandlers[2]);
    nrf_drv_gpiote_in_init(RADIO_DIO_3,&(SX1276.DIO3),irqHandlers[3]);
    nrf_drv_gpiote_in_init(RADIO_DIO_4,&(SX1276.DIO4),irqHandlers[4]);
    nrf_drv_gpiote_in_init(RADIO_DIO_5,&(SX1276.DIO5),irqHandlers[5]);
}

void SX1276IoDeInit( void )
{
    //GpioInit( &SX1276.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    nrf_drv_spi_uninit(&(SX1276.Spi_master));

    // GpioInit( &SX1276.DIO0, RADIO_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    // GpioInit( &SX1276.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    // GpioInit( &SX1276.DIO2, RADIO_DIO_2, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    // GpioInit( &SX1276.DIO3, RADIO_DIO_3, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    // GpioInit( &SX1276.DIO4, RADIO_DIO_4, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    // GpioInit( &SX1276.DIO5, RADIO_DIO_5, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    nrf_drv_gpiote_in_uninit(RADIO_DIO_0);
    nrf_drv_gpiote_in_uninit(RADIO_DIO_1);
    nrf_drv_gpiote_in_uninit(RADIO_DIO_2);
    nrf_drv_gpiote_in_uninit(RADIO_DIO_3);
    nrf_drv_gpiote_in_uninit(RADIO_DIO_4);
    nrf_drv_gpiote_in_uninit(RADIO_DIO_5);
}

void SX1276SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1276Read( REG_PACONFIG );
    paDac = SX1276Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( SX1276.Settings.Channel );
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );
}

uint8_t SX1276GetPaSelect( uint32_t channel )
{
    if( channel < RF_MID_BAND_THRESH )
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

void SX1276SetAntSwLowPower( bool status )
{
    if( RadioIsActive != status )
    {
        RadioIsActive = status;

        if( status == false )
        {
            SX1276AntSwInit( );
        }
        else
        {
            SX1276AntSwDeInit( );
        }
    }
}

void SX1276AntSwInit( void )
{
    //GpioInit( &AntSwitchLf, RADIO_ANT_SWITCH_LF, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    //GpioInit( &AntSwitchHf, RADIO_ANT_SWITCH_HF, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
		nrf_drv_gpiote_out_config_t cfgTmpHi = GPIOTE_CONFIG_OUT_SIMPLE(true);
		nrf_drv_gpiote_out_config_t cfgTmpLo = GPIOTE_CONFIG_OUT_SIMPLE(false);
    AntSwitchLf = 	cfgTmpHi;
    AntSwitchHf = 	cfgTmpLo;
    nrf_drv_gpiote_out_init(RADIO_ANT_SWITCH_LF,&AntSwitchLf);
    nrf_drv_gpiote_out_init(RADIO_ANT_SWITCH_HF,&AntSwitchHf);
}

void SX1276AntSwDeInit( void )
{
    //GpioInit( &AntSwitchLf, RADIO_ANT_SWITCH_LF, PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 );
    //GpioInit( &AntSwitchHf, RADIO_ANT_SWITCH_HF, PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 );
    nrf_drv_gpiote_out_uninit(RADIO_ANT_SWITCH_LF);
    nrf_drv_gpiote_out_uninit(RADIO_ANT_SWITCH_HF);
}

void SX1276SetAntSw( uint8_t opMode )
{
				nrf_drv_gpiote_out_config_t cfgTmpHi = GPIOTE_CONFIG_OUT_SIMPLE(true);
				nrf_drv_gpiote_out_config_t cfgTmpLo = GPIOTE_CONFIG_OUT_SIMPLE(false);
    switch( opMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
        //GpioWrite( &AntSwitchLf, 0 );
        nrf_drv_gpiote_out_uninit(RADIO_ANT_SWITCH_LF);
        nrf_drv_gpiote_out_uninit(RADIO_ANT_SWITCH_HF);
				
        AntSwitchLf = 	cfgTmpLo;
        AntSwitchHf = 	cfgTmpHi;
        nrf_drv_gpiote_out_init(RADIO_ANT_SWITCH_LF,&AntSwitchLf);
        nrf_drv_gpiote_out_init(RADIO_ANT_SWITCH_HF,&AntSwitchHf);
        //GpioWrite( &AntSwitchHf, 1 );

        break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
    default:
        //GpioWrite( &AntSwitchLf, 1 );
        //GpioWrite( &AntSwitchHf, 0 );
        nrf_drv_gpiote_out_uninit(RADIO_ANT_SWITCH_LF);
        nrf_drv_gpiote_out_uninit(RADIO_ANT_SWITCH_HF);
        
        AntSwitchLf = 	cfgTmpHi;
        AntSwitchHf = 	cfgTmpLo;
        nrf_drv_gpiote_out_init(RADIO_ANT_SWITCH_LF,&AntSwitchLf);
        nrf_drv_gpiote_out_init(RADIO_ANT_SWITCH_HF,&AntSwitchHf);
        break;
    }
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}