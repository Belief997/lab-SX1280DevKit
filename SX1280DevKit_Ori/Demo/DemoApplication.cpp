/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: PingPong, PER and Ranging demo implementation.

Maintainer: Gregory Cristian & Gilbert Menth
*/

#include "mbed.h"
#include <math.h>
#include "radio.h"
#include "sx1280-hal.h"
#include "Eeprom.h"
#include "DemoApplication.h"
#include "FreqLUT.h"
#include "RangingCorrection.h"

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
   
double t0 =       -0.016432807883697;                         // X0
double t1 =       0.323147003165358;                          // X1
double t2 =       0.014922061351196;                          // X1^2
double t3 =       0.000137832006285;                          // X1^3
double t4 =       0.536873856625399;                          // X2
double t5 =       0.040890089178579;                          // X2^2
double t6 =       -0.001074801048732;                         // X2^3
double t7 =       0.000009240142234;                          // X2^4


double p[8] = { 0,
                -4.1e-9,
                1.03e-7,
                1.971e-5,
                -0.00107,
                0.018757,
                0.869171,
                3.072450 };

/*!
 * \brief Defines the local payload buffer size
 */
#define BUFFER_SIZE                     255

/*!
 * \brief Defines the size of the token defining message type in the payload
 *        cf. above.
 */
#define PINGPONG_SIZE                   4
#define PER_SIZE                        3

/*!
 * \brief Define time used in PingPong demo to synch with cycle
 * RX_TX_INTER_PACKET_DELAY is the free time between each cycle (time reserve)
 */
#define RX_TX_INTER_PACKET_DELAY        150  // ms
#define RX_TX_TRANSITION_WAIT           15   // ms

/*!
 * \brief Size of ticks (used for Tx and Rx timeout)
 */
#define RX_TIMEOUT_TICK_SIZE            RADIO_TICK_SIZE_1000_US

#define RNG_TIMER_MS                    384 // ms
#define RNG_COM_TIMEOUT                 100 // ms

/*!
 * \brief Ranging raw factors
 *                                  SF5     SF6     SF7     SF8     SF9     SF10
 */
const uint16_t RNG_CALIB_0400[] = { 10299,  10271,  10244,  10242,  10230,  10246  };
const uint16_t RNG_CALIB_0800[] = { 11486,  11474,  11453,  11426,  11417,  11401  };
const uint16_t RNG_CALIB_1600[] = { 13308,  13493,  13528,  13515,  13430,  13376  };
const double   RNG_FGRAD_0400[] = { -0.148, -0.214, -0.419, -0.853, -1.686, -3.423 };
const double   RNG_FGRAD_0800[] = { -0.041, -0.811, -0.218, -0.429, -0.853, -1.737 };
const double   RNG_FGRAD_1600[] = { 0.103,  -0.041, -0.101, -0.211, -0.424, -0.87  };

/*!
 * \brief Define the possible message type for the Ping-Pong and PER apps
 */
const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";
const uint8_t PerMsg[]  = "PER";

/*!
 * \brief Buffer and its size
 */
uint8_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

static uint8_t CurrentChannel;
static uint16_t MeasuredChannels;
int RngResultIndex;
double RawRngResults[DEMO_RNG_CHANNELS_COUNT_MAX];
double RssiRng[DEMO_RNG_CHANNELS_COUNT_MAX];


/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( void );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( IrqErrorCode_t );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRangingDone( IrqRangingCode_t );

/*!
 * \brief All the callbacks are stored in a structure
 */
RadioCallbacks_t Callbacks =
{
    &OnTxDone,        // txDone
    &OnRxDone,        // rxDone
    NULL,             // syncWordDone
    NULL,             // headerDone
    &OnTxTimeout,     // txTimeout
    &OnRxTimeout,     // rxTimeout
    &OnRxError,       // rxError
    &OnRangingDone,   // rangingDone
    NULL,             // cadDone
};

/*!
 * \brief Define IO and callbacks for radio
 * mosi, miso, sclk, nss, busy, dio1, dio2, dio3, rst, callbacks
 */
SX1280Hal Radio( D11, D12, D13, D7, D3, D5, NC, NC, A0, &Callbacks );

/*!
 * \brief Control the Antenna Diversity switch
 */
DigitalOut ANT_SW( A3 );

/*!
 * \brief Tx LED toggling on transmition success
 */
DigitalOut TX_LED( A4 );

/*!
 * \brief Rx LED toggling on reception success
 */
DigitalOut RX_LED( A5 );

/*!
 * \brief Mask of IRQs
 */
uint16_t IrqMask = 0x0000;

/*!
 * \brief Locals parameters and status for radio API
 * NEED TO BE OPTIMIZED, COPY OF STUCTURE ALREADY EXISTING
 */
PacketParams_t PacketParams;
PacketStatus_t PacketStatus;
ModulationParams_t ModulationParams;

/*!
 * \brief Flag to indicate if the demo is already running
 */
static bool DemoRunning = false;

/*!
 * \brief Flag holding the current internal state of the demo application
 */
static uint8_t DemoInternalState = APP_IDLE;

/*!
 * \brief Ticker for master to synch Tx frames. Flags for PER and PingPong demo
 * for Synch TX in cycle.
 */
Ticker SendNextPacket;
static bool SendNext = false;

/*!
 * \brief Ticker for slave to synch Tx frames. Flags for PER and PingPong demo
 * for Synch RX in cycle.
 */
Ticker ReceiveNextPacket;
static bool ReceiveNext = false;

/*!
 * \brief Hold last Rx packet number to compute PER in PER and PingPong demo
 */
static uint32_t PacketRxSequence = 0;
static uint32_t PacketRxSequencePrev = 0;


void SetAntennaSwitch( void );
void LedBlink( void );
void InitializeDemoParameters( uint8_t modulation );
uint16_t GetTimeOnAir( uint8_t modulation );
void SendNextPacketEvent( void );
void ReceiveNextPacketEvent( void );
uint8_t CheckDistance( void );

// **************************     RF Test Demo    ******************************
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *****************************************************************************

uint8_t RunDemoSleepMode( void )
{
    SleepParams_t SleepParam;

    if( Eeprom.EepromData.DemoSettings.HoldDemo == true )
    {
        return 0;
    }
    if( DemoRunning == false )
    {
        DemoRunning = true;
        InitializeDemoParameters( PACKET_TYPE_LORA );
        TX_LED = 0;
        RX_LED = 0;
        SleepParam.WakeUpRTC = 0;                    //!< Get out of sleep mode if wakeup signal received from RTC
        SleepParam.InstructionRamRetention = 0;      //!< InstructionRam is conserved during sleep
        SleepParam.DataBufferRetention = 0;          //!< Data buffer is conserved during sleep
        SleepParam.DataRamRetention = 0;             //!< Data ram is conserved during sleep
        Radio.SetSleep( SleepParam );
    }
    else
    {
        LedBlink( );
    }
    return 0;
}

uint8_t RunDemoStandbyRcMode( void )
{
    if( Eeprom.EepromData.DemoSettings.HoldDemo == true )
    {
        return 0;
    }
    if( DemoRunning == false )
    {
        DemoRunning = true;
        InitializeDemoParameters( PACKET_TYPE_LORA );
        TX_LED = 0;
        RX_LED = 0;
        Radio.SetRegulatorMode( ( RadioRegulatorModes_t )Eeprom.EepromData.DemoSettings.RadioPowerMode );
        Radio.SetStandby( STDBY_RC );
        DemoRunning = true;
    }
    else
    {
        LedBlink( );
    }
    return 0;
}

uint8_t RunDemoStandbyXoscMode( void )
{
    if( Eeprom.EepromData.DemoSettings.HoldDemo == true )
    {
        return 0;
    }
    if( DemoRunning == false )
    {
        DemoRunning = true;
        InitializeDemoParameters( PACKET_TYPE_LORA );
        TX_LED = 0;
        RX_LED = 0;
        Radio.SetRegulatorMode( ( RadioRegulatorModes_t )Eeprom.EepromData.DemoSettings.RadioPowerMode );
        Radio.SetStandby( STDBY_XOSC );
        DemoRunning = true;
    }
    else
    {
        LedBlink( );
    }
    return 0;
}

uint8_t RunDemoTxCw( void )
{
    if( Eeprom.EepromData.DemoSettings.HoldDemo == true )
    {
        return 0;
    }
    if( DemoRunning == false )
    {
        DemoRunning = true;
        InitializeDemoParameters( PACKET_TYPE_LORA );
        TX_LED = 0;
        RX_LED = 0;
        SetAntennaSwitch( );
        Radio.SetStandby( STDBY_RC );
        Radio.SetRegulatorMode( ( RadioRegulatorModes_t )Eeprom.EepromData.DemoSettings.RadioPowerMode );
        Radio.SetRfFrequency( Eeprom.EepromData.DemoSettings.Frequency );
        Radio.SetTxParams( Eeprom.EepromData.DemoSettings.TxPower, RADIO_RAMP_20_US );
        Radio.SetTxContinuousWave( );
        DemoRunning = true;
    }
    else
    {
        LedBlink( );
    }
    return 0;
}

uint8_t RunDemoTxContinuousModulation( void )
{
    uint8_t localPayloadSize = 250;
    uint8_t i = 0;

    if( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_RANGING )
    {
        Eeprom.EepromData.DemoSettings.ModulationType = PACKET_TYPE_LORA;
    }
    if( Eeprom.EepromData.DemoSettings.HoldDemo == true )
    {
        return 0;
    }

    if( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_FLRC )
    {
        localPayloadSize = 120; // Encoded in 4/8 so 240 bytes in total
    }

    Radio.ProcessIrqs( );

    if( DemoRunning == false )
    {
        DemoRunning = true;
        InitializeDemoParameters( Eeprom.EepromData.DemoSettings.ModulationType );
        TX_LED = 0;
        RX_LED = 0;
        IrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
        Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
        for( i = 0; i < localPayloadSize; i++ )
        {
            Buffer[i] = ( uint8_t )rand( );
        }
//        Radio.SetAutoFS( true ); // no need to relock the PLL between packets
        Radio.SendPayload( Buffer, localPayloadSize, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, 10000 } );
        DemoInternalState = APP_IDLE;
    }
    else
    {
        switch( DemoInternalState )
        {
            case APP_RX:
                break;

            case APP_TX:
                DemoInternalState = APP_IDLE;
                LedBlink( );
                // Send the next frame
                IrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
                Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                for( i = 0; i < localPayloadSize; i++ )
                {
                    Buffer[i] = ( uint8_t )rand( );
                }
//                Radio.SetAutoFS( true ); // no need to relock the PLL between packets
                Radio.SendPayload( Buffer, localPayloadSize, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, 0xFFFF } );
                break;

            case APP_RX_TIMEOUT:
                DemoInternalState = APP_IDLE;
                break;

            case APP_RX_ERROR:
                DemoInternalState = APP_IDLE;
                break;

            case APP_TX_TIMEOUT:
                DemoInternalState = APP_IDLE; 
                break;

            case APP_IDLE:
                break;

            default:
                break;
        }
    }
    return 0;
}

// *************************       PER Demo       ******************************
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *****************************************************************************
uint8_t RunDemoApplicationPer( void )
{
    uint8_t i = 0;
    uint8_t refreshDisplay = 0;
    static uint8_t CurrentChannel;

    if( Eeprom.EepromData.DemoSettings.HoldDemo == true )
    {
        return 0;
    }

    if( DemoRunning == false )
    {
        DemoRunning = true;
        CurrentChannel = 0;

#ifdef PRINT_DEBUG
        printf( "Start RunDemoApplicationPer\n\r" );
#endif

        TX_LED = 0;
        RX_LED = 0;
        SetAntennaSwitch( );
        Eeprom.EepromData.DemoSettings.CntPacketTx    = 0;
        Eeprom.EepromData.DemoSettings.CntPacketRxOK  = 0;
        Eeprom.EepromData.DemoSettings.CntPacketRxKO  = 0;
        Eeprom.EepromData.DemoSettings.RxTimeOutCount = 0;

        InitializeDemoParameters( Eeprom.EepromData.DemoSettings.ModulationType );

        UpdateRadioFrequency( Channels[CurrentChannel] );
        Radio.SetRfFrequency( Channels[CurrentChannel] );

        Eeprom.EepromData.DemoSettings.TimeOnAir = GetTimeOnAir( Eeprom.EepromData.DemoSettings.ModulationType );

        if( Eeprom.EepromData.DemoSettings.Entity == MASTER )
        {
            SendNextPacket.attach_us( &SendNextPacketEvent, ( ( uint32_t )( Eeprom.EepromData.DemoSettings.TimeOnAir + RX_TX_INTER_PACKET_DELAY ) ) * 1000 );
            DemoInternalState = APP_TX;
        }
        else
        {
            IrqMask = IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;
            Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
            // Rx Single without timeout for the start
            Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, 0x0000 } );
            DemoInternalState = APP_IDLE;
        }
    }

    Radio.ProcessIrqs( );

    if( Eeprom.EepromData.DemoSettings.MaxNumPacket > 0 ) // != Infinite
    {
        if( ( Eeprom.EepromData.DemoSettings.CntPacketRxOK + \
              Eeprom.EepromData.DemoSettings.CntPacketRxKO + \
              Eeprom.EepromData.DemoSettings.RxTimeOutCount) >= \
            Eeprom.EepromData.DemoSettings.MaxNumPacket )
        {
            RX_LED = 0;
            TX_LED = 0;
            DemoInternalState = APP_IDLE;
            Radio.SetStandby( STDBY_RC );
            SendNextPacket.detach( );
            ReceiveNextPacket.detach( );
            Eeprom.EepromData.DemoSettings.HoldDemo = true;
            refreshDisplay = 1;
        }
    }

    switch( DemoInternalState )
    {
        case PER_TX_START:
            Eeprom.EepromData.DemoSettings.CntPacketTx++;
            DemoInternalState = APP_IDLE;

            Buffer[0] = ( Eeprom.EepromData.DemoSettings.CntPacketTx >> 24 ) & 0xFF;
            Buffer[1] = ( Eeprom.EepromData.DemoSettings.CntPacketTx >> 16 ) & 0xFF;
            Buffer[2] = ( Eeprom.EepromData.DemoSettings.CntPacketTx >> 8 )  & 0xFF;
            Buffer[3] = Eeprom.EepromData.DemoSettings.CntPacketTx & 0xFF;
            Buffer[4] = PerMsg[0];
            Buffer[5] = PerMsg[1];
            Buffer[6] = PerMsg[2];
            for( i = 7; i < Eeprom.EepromData.DemoSettings.PayloadLength; i++ )
            {
                Buffer[i] = i;
            }
            TX_LED = !TX_LED;
            IrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;

            UpdateRadioFrequency( Channels[CurrentChannel] );
            Radio.SetRfFrequency( Channels[CurrentChannel] );

            Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
            Radio.SendPayload( Buffer, Eeprom.EepromData.DemoSettings.PayloadLength, 
                               ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, Eeprom.EepromData.DemoSettings.TimeOnAir << 1 } );
            if( CurrentChannel < ( CHANNELS - 1 ) )
            {
                CurrentChannel++;
            }
            else
            {
                CurrentChannel = 0;
            }
            break;

        case PER_RX_START:
            UpdateRadioFrequency( Channels[CurrentChannel] );
            Radio.SetRfFrequency( Channels[CurrentChannel] );
            IrqMask = IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;
            Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
            Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, 0xFFFF } );
            DemoInternalState = APP_IDLE;
            break;

        case APP_TX:
            if( SendNext == true )
            {
                SendNext = false;
                if( ( Eeprom.EepromData.DemoSettings.MaxNumPacket == 0 ) || 
                    ( Eeprom.EepromData.DemoSettings.CntPacketTx < Eeprom.EepromData.DemoSettings.MaxNumPacket ) )
                {
                    DemoInternalState = PER_TX_START;  // send next
                    refreshDisplay = 1;
                }
                else    // MaxNumPacket sent -> end of demo
                {
                    RX_LED = 0;
                    TX_LED = 0;
                    DemoInternalState = APP_IDLE;
                    Radio.SetStandby( STDBY_RC );
                    SendNextPacket.detach( );
                    Eeprom.EepromData.DemoSettings.HoldDemo = true;
                    refreshDisplay = 1;
                }
            }
            break;

        case APP_RX:
            RX_LED = !RX_LED;
            Radio.SetStandby( STDBY_RC );
            ReceiveNext = false;
            ReceiveNextPacket.detach( );
            ReceiveNextPacket.attach_us( &ReceiveNextPacketEvent, ( ( uint32_t )( Eeprom.EepromData.DemoSettings.TimeOnAir + RX_TX_INTER_PACKET_DELAY + RX_TX_TRANSITION_WAIT ) ) * 1000 );
            if( CurrentChannel < ( CHANNELS - 1 ) )
            {
                CurrentChannel++;
            }
            else
            {
                CurrentChannel = 0;
            }
            Radio.GetPayload( Buffer, &BufferSize, BUFFER_SIZE );
            Radio.GetPacketStatus( &PacketStatus );
            if( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_LORA )
            {
                Eeprom.EepromData.DemoSettings.RssiValue = PacketStatus.LoRa.RssiPkt;
                Eeprom.EepromData.DemoSettings.SnrValue = PacketStatus.LoRa.SnrPkt;
            }
            else if( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_FLRC )
            {
                Eeprom.EepromData.DemoSettings.RssiValue = PacketStatus.Flrc.RssiSync;
                Eeprom.EepromData.DemoSettings.SnrValue = 0;
            }
            else if( Eeprom.EepromData.DemoSettings.ModulationType == PACKET_TYPE_GFSK  )
            {
                Eeprom.EepromData.DemoSettings.RssiValue = PacketStatus.Gfsk.RssiSync;
                Eeprom.EepromData.DemoSettings.SnrValue = 0;
            }
            DemoInternalState = PER_RX_START;
            if( ( BufferSize >= PER_SIZE ) && ( strncmp( ( const char* )( Buffer + 4 ), ( const char* )PerMsg, PER_SIZE ) == 0 ) )
            {
                ComputePerPayload( Buffer, BufferSize );
                refreshDisplay = 1;
            }
            else
            {
                Eeprom.EepromData.DemoSettings.RxTimeOutCount++;
            }
            break;

        case APP_RX_ERROR:
            printf("crc\n\r");
            Radio.SetStandby( STDBY_RC );
            DemoInternalState = APP_IDLE;
            break;

        case APP_RX_TIMEOUT:
            printf("tmt\n\r");
            Radio.SetStandby( STDBY_RC );
            if( CurrentChannel < ( CHANNELS - 1 ) )
            {
                CurrentChannel++;
            }
            else
            {
                CurrentChannel = 0;
            }
            Eeprom.EepromData.DemoSettings.RxTimeOutCount++;
            DemoInternalState = PER_RX_START;
            refreshDisplay = 1;
            break;

        case APP_TX_TIMEOUT:
#ifdef PRINT_DEBUG
            printf( "Failure: timeout in Tx is shorter than the packet time on air\n\r" );
#endif
            DemoInternalState = APP_IDLE;
            Eeprom.EepromData.DemoSettings.HoldDemo = true;
            refreshDisplay = 1;
            break;

        case APP_IDLE: // do nothing
            if( ReceiveNext == true )
            {
                ReceiveNext = false;
                DemoInternalState = APP_RX_TIMEOUT;
            }
            break;

        default:
            break;
    }
    return refreshDisplay;
}

void ComputePerPayload( uint8_t *buffer, uint8_t bufferSize )
{
    uint32_t i = 0;

    Eeprom.EepromData.DemoSettings.CntPacketRxOK++;
    PacketRxSequence = ( ( uint32_t )buffer[0] << 24 ) | \
                       ( ( uint32_t )buffer[1] << 16 ) | \
                       ( ( uint32_t )buffer[2] << 8 )  | \
                                     buffer[3];

    if( ( PacketRxSequence <= PacketRxSequencePrev ) || \
        ( PacketRxSequencePrev == 0xFFFFFFFF ) )
    {
        // Sequence went back => resynchronization
        // Don't count missed packets this time
        i = 0;
    }
    else
    {
        // Determine number of missed packets
        i = PacketRxSequence - PacketRxSequencePrev - 1;
    }
    // Be ready for the next
    PacketRxSequencePrev = PacketRxSequence;
    // increment 'missed' counter for the RX session
    Eeprom.EepromData.DemoSettings.CntPacketRxKO += i;
    Eeprom.EepromData.DemoSettings.RxTimeOutCount = 0;
}

// ************************     Ping Pong Demo     *****************************
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *****************************************************************************
uint8_t RunDemoApplicationPingPong( void )
{
    uint8_t i = 0;
    uint8_t refreshDisplay = 0;

    if( Eeprom.EepromData.DemoSettings.HoldDemo == true )
    {
        return 0;   // quit without refresh display
    }

    if( DemoRunning == false )
    {
        DemoRunning = true;
        TX_LED = 0;
        RX_LED = 0;
        SetAntennaSwitch( );
        ReceiveNext = false;
        SendNext = false;
        Eeprom.EepromData.DemoSettings.CntPacketTx        = 0;
        Eeprom.EepromData.DemoSettings.CntPacketRxOK      = 0;
        Eeprom.EepromData.DemoSettings.CntPacketRxOKSlave = 0;
        Eeprom.EepromData.DemoSettings.CntPacketRxKO      = 0;
        Eeprom.EepromData.DemoSettings.CntPacketRxKOSlave = 0;
        Eeprom.EepromData.DemoSettings.RxTimeOutCount     = 0;

        InitializeDemoParameters( Eeprom.EepromData.DemoSettings.ModulationType );

        Eeprom.EepromData.DemoSettings.TimeOnAir = GetTimeOnAir( Eeprom.EepromData.DemoSettings.ModulationType );

#ifdef PRINT_DEBUG
        printf( "Start RunDemoApplicationPingPong.\n\r" );
#endif

        if( Eeprom.EepromData.DemoSettings.Entity == MASTER )
        {
            DemoInternalState = SEND_PING_MSG;
            SendNextPacket.attach_us( &SendNextPacketEvent, ( ( uint32_t )( ( Eeprom.EepromData.DemoSettings.TimeOnAir * 2 ) + RX_TX_INTER_PACKET_DELAY * 2 ) * 1000 ) );
        }
        else
        {
            IrqMask = IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;
            Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
            // Rx Single without timeout for the start
            RX_LED = !RX_LED;
            Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, 0x0000 } );
            DemoInternalState = APP_IDLE;
        }
    }

    Radio.ProcessIrqs( );

    if( Eeprom.EepromData.DemoSettings.Entity == MASTER )
    {
        switch( DemoInternalState )
        {
            case SEND_PING_MSG:
                if( ( Eeprom.EepromData.DemoSettings.MaxNumPacket != 0 ) \
                    && ( Eeprom.EepromData.DemoSettings.CntPacketTx >= Eeprom.EepromData.DemoSettings.MaxNumPacket ) )
                {
                    SendNextPacket.detach( );
                    ReceiveNextPacket.detach( );
                    ReceiveNext = false;
                    SendNext = false;
                    RX_LED = 0;
                    TX_LED = 0;
                    DemoInternalState = APP_IDLE;
                    
                    Eeprom.EepromData.DemoSettings.HoldDemo = true;
                    refreshDisplay = 1;
                }
                else
                {
                    if( SendNext == true )
                    {
                        SendNext = false;

                        DemoInternalState = APP_IDLE;
                        Eeprom.EepromData.DemoSettings.CntPacketTx++;
                        // Send the next PING frame
                        Buffer[0] = ( Eeprom.EepromData.DemoSettings.CntPacketTx >> 24 ) & 0xFF;
                        Buffer[1] = ( Eeprom.EepromData.DemoSettings.CntPacketTx >> 16 ) & 0xFF;
                        Buffer[2] = ( Eeprom.EepromData.DemoSettings.CntPacketTx >> 8 )  & 0xFF;
                        Buffer[3] = ( Eeprom.EepromData.DemoSettings.CntPacketTx & 0xFF );
                        Buffer[4] = PingMsg[0];
                        Buffer[5] = PingMsg[1];
                        Buffer[6] = PingMsg[2];
                        Buffer[7] = PingMsg[3];
                        for( i = 8; i < Eeprom.EepromData.DemoSettings.PayloadLength; i++ )
                        {
                            Buffer[i] = i;
                        }
                        TX_LED = !TX_LED;
                        IrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
                        Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                        Radio.SendPayload( Buffer, Eeprom.EepromData.DemoSettings.PayloadLength, \
                                                   ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, \
                                                   Eeprom.EepromData.DemoSettings.TimeOnAir + RX_TX_TRANSITION_WAIT } );
                    }
                }
                break;

            case APP_TX:
                DemoInternalState = SEND_PING_MSG;
                TX_LED = !TX_LED;
                RX_LED = !RX_LED;
                IrqMask = IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;
                Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, Eeprom.EepromData.DemoSettings.TimeOnAir + RX_TX_TRANSITION_WAIT } );
                break;

            case APP_RX:
                RX_LED = !RX_LED;
                Radio.GetPayload( Buffer, &BufferSize, BUFFER_SIZE );
                Radio.GetPacketStatus( &PacketStatus );
                if( Eeprom.EepromData.ModulationParams.PacketType == PACKET_TYPE_LORA )
                {
                    Eeprom.EepromData.DemoSettings.RssiValue = PacketStatus.LoRa.RssiPkt;
                    Eeprom.EepromData.DemoSettings.SnrValue = PacketStatus.LoRa.SnrPkt;
                }
                else if( Eeprom.EepromData.ModulationParams.PacketType == PACKET_TYPE_FLRC )
                {
                    Eeprom.EepromData.DemoSettings.RssiValue = PacketStatus.Flrc.RssiSync;
                    Eeprom.EepromData.DemoSettings.SnrValue = 0;
                }
                else if( Eeprom.EepromData.ModulationParams.PacketType == PACKET_TYPE_GFSK )
                {
                    Eeprom.EepromData.DemoSettings.RssiValue = PacketStatus.Gfsk.RssiSync;
                    Eeprom.EepromData.DemoSettings.SnrValue = 0;
                }
                if( ( BufferSize >= PINGPONG_SIZE ) && ( strncmp( ( const char* )( Buffer + 8 ), ( const char* )PongMsg, PINGPONG_SIZE ) == 0 ) )
                {
                    ComputePingPongPayload( Buffer, BufferSize );
                }
                else
                {
                    Eeprom.EepromData.DemoSettings.CntPacketRxKO++;
                }
                DemoInternalState = SEND_PING_MSG;
                refreshDisplay = 1;
                break;

            case APP_RX_TIMEOUT:
            case APP_RX_ERROR:
                Radio.SetStandby( STDBY_RC );
                RX_LED = !RX_LED;
                Eeprom.EepromData.DemoSettings.CntPacketRxKO++;
                DemoInternalState = SEND_PING_MSG;
                refreshDisplay = 1;
                break;

            case APP_TX_TIMEOUT:
#ifdef PRINT_DEBUG
                printf( "Failure: timeout in Tx is shorter than the packet time on air\n\r" );
#endif
                DemoInternalState = APP_IDLE;
                Eeprom.EepromData.DemoSettings.HoldDemo = true;
                refreshDisplay = 1;
                break;

            case APP_IDLE: // do nothing
                break;

            default:
                break;
        }
    }
    else // SLAVE
    {
        switch( DemoInternalState )
        {
            case SEND_PONG_MSG:
                wait_ms( 2 );

                DemoInternalState = APP_IDLE;
                // Send the next PING frame
                Buffer[0]  = ( Eeprom.EepromData.DemoSettings.CntPacketTx >> 24 ) & 0xFF;
                Buffer[1]  = ( Eeprom.EepromData.DemoSettings.CntPacketTx >> 16 ) & 0xFF;
                Buffer[2]  = ( Eeprom.EepromData.DemoSettings.CntPacketTx >>  8 ) & 0xFF;
                Buffer[3]  = ( Eeprom.EepromData.DemoSettings.CntPacketTx & 0xFF );
                Buffer[4]  = ( ( Eeprom.EepromData.DemoSettings.CntPacketRxKO + \
                                 Eeprom.EepromData.DemoSettings.RxTimeOutCount ) >> 24 ) & 0xFF;
                Buffer[5]  = ( ( Eeprom.EepromData.DemoSettings.CntPacketRxKO + \
                                 Eeprom.EepromData.DemoSettings.RxTimeOutCount ) >> 16 ) & 0xFF;
                Buffer[6]  = ( ( Eeprom.EepromData.DemoSettings.CntPacketRxKO + \
                                 Eeprom.EepromData.DemoSettings.RxTimeOutCount ) >> 8 ) & 0xFF;
                Buffer[7]  = ( ( Eeprom.EepromData.DemoSettings.CntPacketRxKO + \
                                 Eeprom.EepromData.DemoSettings.RxTimeOutCount ) & 0xFF );
                Buffer[8]  = PongMsg[0];
                Buffer[9]  = PongMsg[1];
                Buffer[10] = PongMsg[2];
                Buffer[11] = PongMsg[3];
                for( i = 12; i < Eeprom.EepromData.DemoSettings.PayloadLength; i++ )
                {
                    Buffer[i] = i;
                }
                TX_LED = !TX_LED;
                IrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
                Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                Radio.SendPayload( Buffer, Eeprom.EepromData.DemoSettings.PayloadLength, \
                                   ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, \
                                   Eeprom.EepromData.DemoSettings.TimeOnAir + RX_TX_TRANSITION_WAIT } );
                break;

            case APP_TX:
                if( ( Eeprom.EepromData.DemoSettings.MaxNumPacket != 0 ) \
                    && ( ( Eeprom.EepromData.DemoSettings.CntPacketRxOK + Eeprom.EepromData.DemoSettings.CntPacketRxKO + \
                           Eeprom.EepromData.DemoSettings.RxTimeOutCount ) >= Eeprom.EepromData.DemoSettings.MaxNumPacket ) )
                {
                    SendNextPacket.detach( ); 
                    SendNext = false;
                    RX_LED = 0;
                    TX_LED = 0;
                    DemoInternalState = APP_IDLE;
                    Radio.SetStandby( STDBY_RC );
                    Eeprom.EepromData.DemoSettings.HoldDemo = true;
                    refreshDisplay = 1;
                }
                else
                {
                    DemoInternalState = APP_IDLE;
                    TX_LED = !TX_LED;
                    RX_LED = !RX_LED;
                    IrqMask = IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;
                    Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                    Radio.SetRx( ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, 0xFFFF } );
                    refreshDisplay = 1;
                }
                break;

            case APP_RX:
                DemoInternalState = APP_IDLE;
                RX_LED = !RX_LED;

                Radio.SetStandby( STDBY_RC );
                ReceiveNext = false;
                ReceiveNextPacket.detach( );
                ReceiveNextPacket.attach_us( &ReceiveNextPacketEvent, ( ( uint32_t )( ( Eeprom.EepromData.DemoSettings.TimeOnAir << 1 ) + RX_TX_INTER_PACKET_DELAY * 2 + RX_TX_TRANSITION_WAIT ) ) * 1000 );

                Radio.GetPayload( Buffer, &BufferSize, BUFFER_SIZE );
                Radio.GetPacketStatus( &PacketStatus );
                if( Eeprom.EepromData.ModulationParams.PacketType == PACKET_TYPE_LORA )
                {
                    Eeprom.EepromData.DemoSettings.RssiValue = PacketStatus.LoRa.RssiPkt;
                    Eeprom.EepromData.DemoSettings.SnrValue = PacketStatus.LoRa.SnrPkt;
                }
                else if( Eeprom.EepromData.ModulationParams.PacketType == PACKET_TYPE_FLRC )
                {
                    Eeprom.EepromData.DemoSettings.RssiValue = PacketStatus.Flrc.RssiSync;
                    Eeprom.EepromData.DemoSettings.SnrValue = 0;
                }
                else if( Eeprom.EepromData.ModulationParams.PacketType == PACKET_TYPE_GFSK  )
                {
                    Eeprom.EepromData.DemoSettings.RssiValue = PacketStatus.Gfsk.RssiSync;
                    Eeprom.EepromData.DemoSettings.SnrValue = 0;
                }
                ComputePingPongPayload( Buffer, BufferSize );
                DemoInternalState = SEND_PONG_MSG;
                break;

            case APP_RX_TIMEOUT:
                printf("tmt\n\r");
                Radio.SetStandby( STDBY_RC );
                Eeprom.EepromData.DemoSettings.RxTimeOutCount++;
                DemoInternalState = SEND_PONG_MSG;
                refreshDisplay = 1;
                break;

            case APP_RX_ERROR:
                printf("crc\n\r");
                Radio.SetStandby( STDBY_RC );
                DemoInternalState = APP_IDLE;
                break;

            case APP_TX_TIMEOUT:
#ifdef PRINT_DEBUG
                printf( "Failure: timeout in Tx is shorter than the packet time on air\n\r" );
#endif
                DemoInternalState = APP_IDLE;
                Eeprom.EepromData.DemoSettings.HoldDemo = true;
                refreshDisplay = 1;
                break;

            case APP_IDLE: // do nothing
                if( ReceiveNext == true )
                {
                    ReceiveNext = false;
                    DemoInternalState = APP_RX_TIMEOUT;
                }
                break;

            default:
                break;
        }
    }
    return refreshDisplay;
}

void ComputePingPongPayload( uint8_t *buffer, uint8_t bufferSize )
{
    uint32_t i = 0;

    PacketRxSequence = ( ( uint32_t )buffer[0] << 24 ) | \
                       ( ( uint32_t )buffer[1] << 16 ) | \
                       ( ( uint32_t )buffer[2] << 8 )  | \
                                     buffer[3];

    if( Eeprom.EepromData.DemoSettings.Entity == MASTER )
    {
        Eeprom.EepromData.DemoSettings.CntPacketRxKOSlave = 
                       ( ( uint32_t )buffer[4] << 24 ) | \
                       ( ( uint32_t )buffer[5] << 16 ) | \
                       ( ( uint32_t )buffer[6] << 8 )  | \
                                     buffer[7];
        if( PacketRxSequence > Eeprom.EepromData.DemoSettings.CntPacketRxKOSlave )
        {
            Eeprom.EepromData.DemoSettings.CntPacketRxOKSlave = PacketRxSequence - \
                Eeprom.EepromData.DemoSettings.CntPacketRxKOSlave;
        }
        else
        {
            Eeprom.EepromData.DemoSettings.CntPacketRxOKSlave = 0;
        }

        if( PacketRxSequence == Eeprom.EepromData.DemoSettings.CntPacketTx )
        {
            Eeprom.EepromData.DemoSettings.CntPacketRxOK += 1;
        }
        else
        {
            Eeprom.EepromData.DemoSettings.CntPacketRxKO += 1;
        }
    }
    else
    {
        Eeprom.EepromData.DemoSettings.CntPacketRxOK += 1;
        if( ( PacketRxSequence <= PacketRxSequencePrev ) || \
            ( PacketRxSequencePrev == 0 ) )
        {
            // Sequence went back => resynchronization
            // Don't count missed packets this time
            i = 0;
        }
        else
        {
            // Determine number of missed packets
            i = PacketRxSequence - PacketRxSequencePrev - 1;
        }
        // Be ready for the next
        PacketRxSequencePrev = PacketRxSequence;
        Eeprom.EepromData.DemoSettings.CntPacketTx = PacketRxSequence;
        // increment 'missed' counter for the RX session
        Eeprom.EepromData.DemoSettings.CntPacketRxKO += i;
        Eeprom.EepromData.DemoSettings.RxTimeOutCount = 0;
    }
}

// ************************      Ranging Demo      *****************************
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *****************************************************************************
uint8_t RunDemoApplicationRanging( void )
{
    uint8_t refreshDisplay = 0;

    if( Eeprom.EepromData.DemoSettings.HoldDemo == true )
    {
        return 0;   // quit without refresh display
    }

    if( DemoRunning == false )
    {
        DemoRunning = true;
        TX_LED = 0;
        RX_LED = 0;
        ANT_SW = 1;

#ifdef PRINT_DEBUG
        printf( "Start RunDemoApplicationRanging\r\n" );
#endif

        Eeprom.EepromData.DemoSettings.CntPacketTx = 0;
        Eeprom.EepromData.DemoSettings.RngFei      = 0.0;
        Eeprom.EepromData.DemoSettings.RngStatus   = RNG_INIT;
        InitializeDemoParameters( Eeprom.EepromData.DemoSettings.ModulationType );

        if( Eeprom.EepromData.DemoSettings.Entity == MASTER )
        {
            Eeprom.EepromData.DemoSettings.TimeOnAir = RX_TX_INTER_PACKET_DELAY;
            Radio.SetDioIrqParams( IRQ_RX_DONE | IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_RANGING_MASTER_RESULT_VALID | IRQ_RANGING_MASTER_TIMEOUT,
                                   IRQ_RX_DONE | IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_RANGING_MASTER_RESULT_VALID | IRQ_RANGING_MASTER_TIMEOUT,
                                   IRQ_RADIO_NONE, IRQ_RADIO_NONE );
            Eeprom.EepromData.DemoSettings.RngDistance = 0.0;
            DemoInternalState = APP_RANGING_CONFIG;
        }
        else
        {
            Radio.SetDioIrqParams( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
            DemoInternalState = APP_RANGING_CONFIG;
        }
    }

    Radio.ProcessIrqs( );

    if( Eeprom.EepromData.DemoSettings.Entity == MASTER )
    {
        switch( DemoInternalState )
        {
            case APP_RANGING_CONFIG:
                if( Eeprom.EepromData.DemoSettings.HoldDemo == false )
                {
                    Eeprom.EepromData.DemoSettings.RngStatus = RNG_INIT;
                    Eeprom.EepromData.DemoSettings.CntPacketTx++;
                    ModulationParams.PacketType = PACKET_TYPE_LORA;
                    PacketParams.PacketType     = PACKET_TYPE_LORA;
                    memcpy( &( ModulationParams.Params.LoRa.SpreadingFactor ), Eeprom.Buffer + MOD_RNG_SPREADF_EEPROM_ADDR,      1 );
                    memcpy( &( ModulationParams.Params.LoRa.Bandwidth ),       Eeprom.Buffer + MOD_RNG_BW_EEPROM_ADDR,           1 );
                    memcpy( &( ModulationParams.Params.LoRa.CodingRate ),      Eeprom.Buffer + MOD_RNG_CODERATE_EEPROM_ADDR,     1 );
                    memcpy( &( PacketParams.Params.LoRa.PreambleLength ),      Eeprom.Buffer + PAK_RNG_PREAMBLE_LEN_EEPROM_ADDR, 1 );
                    memcpy( &( PacketParams.Params.LoRa.HeaderType ),          Eeprom.Buffer + PAK_RNG_HEADERTYPE_EEPROM_ADDR,   1 );
                    PacketParams.Params.LoRa.PayloadLength = 7;
                    memcpy( &( PacketParams.Params.LoRa.Crc ),                 Eeprom.Buffer + PAK_RNG_CRC_MODE_EEPROM_ADDR,     1 );
                    memcpy( &( PacketParams.Params.LoRa.InvertIQ ),            Eeprom.Buffer + PAK_RNG_IQ_INV_EEPROM_ADDR,       1 );
                    Radio.SetPacketType( ModulationParams.PacketType );
                    Radio.SetModulationParams( &ModulationParams );
                    Radio.SetPacketParams( &PacketParams );
                    Radio.SetRfFrequency( Eeprom.EepromData.DemoSettings.Frequency );
                    Eeprom.EepromData.DemoSettings.CntPacketRxOK = 0;
                    Eeprom.EepromData.DemoSettings.CntPacketRxOKSlave = 0;
                    MeasuredChannels  = 0;
                    CurrentChannel    = 0;
                    Buffer[0] = ( Eeprom.EepromData.DemoSettings.RngAddress >> 24 ) & 0xFF;
                    Buffer[1] = ( Eeprom.EepromData.DemoSettings.RngAddress >> 16 ) & 0xFF;
                    Buffer[2] = ( Eeprom.EepromData.DemoSettings.RngAddress >>  8 ) & 0xFF;
                    Buffer[3] = ( Eeprom.EepromData.DemoSettings.RngAddress & 0xFF );
                    Buffer[4] = CurrentChannel;    // set the first channel to use
                    Buffer[5] = Eeprom.EepromData.DemoSettings.RngAntenna;      // set the antenna strategy
                    Buffer[6] = Eeprom.EepromData.DemoSettings.RngRequestCount; // set the number of hops
                    TX_LED = 1;
                    Radio.SendPayload( Buffer, PacketParams.Params.LoRa.PayloadLength, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, RNG_COM_TIMEOUT } );
                    DemoInternalState = APP_IDLE;
                }
                break;

            case APP_RNG:
                if( SendNext == true )
                {
                    SendNext = false;
                    MeasuredChannels++;
                    if( MeasuredChannels <= Eeprom.EepromData.DemoSettings.RngRequestCount )
                    {
                        Radio.SetRfFrequency( Channels[CurrentChannel] );
                        TX_LED = 1;
                        switch( Eeprom.EepromData.DemoSettings.RngAntenna )
                        {
                            case DEMO_RNG_ANT_1:
                                //ANT_SW = 1; // ANT1
                                Eeprom.EepromData.DemoSettings.AntennaSwitch = 0;
                                CurrentChannel++;
                                if( CurrentChannel >= CHANNELS )
                                {
                                    CurrentChannel -= CHANNELS;
                                }
                                break;

                            case DEMO_RNG_ANT_0:
                                //ANT_SW = 0; // ANT0
                                Eeprom.EepromData.DemoSettings.AntennaSwitch = 1;
                                CurrentChannel++;
                                if( CurrentChannel >= CHANNELS )
                                {
                                    CurrentChannel -= CHANNELS;
                                }
                                break;

                            case DEMO_RNG_ANT_BOTH:
                                if( ANT_SW == 1 )
                                {
                                    //ANT_SW = 0;
                                    Eeprom.EepromData.DemoSettings.AntennaSwitch = 1;
                                }
                                else
                                {
                                    //ANT_SW = 1;
                                    Eeprom.EepromData.DemoSettings.AntennaSwitch = 0;
                                    CurrentChannel++;
                                    if( CurrentChannel >= CHANNELS )
                                    {
                                        CurrentChannel -= CHANNELS;
                                    }
                                }
                                break;
                        }
                        SetAntennaSwitch( );
                        DemoInternalState = APP_IDLE;
                        Radio.SetTx( ( TickTime_t ){ RADIO_TICK_SIZE_1000_US, 0xFFFF } );
                    }
                    else
                    {
                        Eeprom.EepromData.DemoSettings.CntPacketRxOKSlave = CheckDistance( );
                        refreshDisplay = 1;
                        SendNextPacket.detach( );
                        Eeprom.EepromData.DemoSettings.HoldDemo = true;
                        SendNext = false;
                        DemoInternalState = APP_RANGING_CONFIG;
                        Eeprom.EepromData.DemoSettings.RngStatus = RNG_INIT;
                    }
                }
                break;

            case APP_RANGING_DONE:
                TX_LED = 0;
                RawRngResults[RngResultIndex] = Radio.GetRangingResult( RANGING_RESULT_RAW );
                RawRngResults[RngResultIndex] += Sx1280RangingCorrection::GetRangingCorrectionPerSfBwGain(
                    ModulationParams.Params.LoRa.SpreadingFactor,
                    ModulationParams.Params.LoRa.Bandwidth,
                    Radio.GetRangingPowerDeltaThresholdIndicator( )
                );
                RngResultIndex++;

                Eeprom.EepromData.DemoSettings.CntPacketRxOK++;
                DemoInternalState = APP_RNG;
                break;

            case APP_RANGING_TIMEOUT:
                TX_LED = 0;
                DemoInternalState = APP_RNG;
                break;

            case APP_RX:
                RX_LED = 0;
                if( Eeprom.EepromData.DemoSettings.RngStatus == RNG_INIT )
                {
                    Radio.GetPayload( Buffer, &BufferSize, BUFFER_SIZE );
                    if( BufferSize > 0 )
                    {
                        Eeprom.EepromData.DemoSettings.RxTimeOutCount = 0;
                        Eeprom.EepromData.DemoSettings.RngStatus = RNG_PROCESS;
                        Eeprom.EepromData.DemoSettings.RngFei = ( double )( ( ( int32_t )Buffer[4] << 24 ) | \
                                                                            ( ( int32_t )Buffer[5] << 16 ) | \
                                                                            ( ( int32_t )Buffer[6] <<  8 ) | \
                                                                                         Buffer[7] );
                        Eeprom.EepromData.DemoSettings.RssiValue = Buffer[8]; // for ranging post-traitment (since V3 only)
                        ModulationParams.PacketType = PACKET_TYPE_RANGING;
                        PacketParams.PacketType     = PACKET_TYPE_RANGING;

                        memcpy( &( ModulationParams.Params.LoRa.SpreadingFactor ), Eeprom.Buffer + MOD_RNG_SPREADF_EEPROM_ADDR,      1 );
                        memcpy( &( ModulationParams.Params.LoRa.Bandwidth ),       Eeprom.Buffer + MOD_RNG_BW_EEPROM_ADDR,           1 );
                        memcpy( &( ModulationParams.Params.LoRa.CodingRate ),      Eeprom.Buffer + MOD_RNG_CODERATE_EEPROM_ADDR,     1 );
                        memcpy( &( PacketParams.Params.LoRa.PreambleLength ),      Eeprom.Buffer + PAK_RNG_PREAMBLE_LEN_EEPROM_ADDR, 1 );
                        memcpy( &( PacketParams.Params.LoRa.HeaderType ),          Eeprom.Buffer + PAK_RNG_HEADERTYPE_EEPROM_ADDR,   1 );
                        PacketParams.Params.LoRa.PayloadLength = 10;
                        memcpy( &( PacketParams.Params.LoRa.Crc ),                 Eeprom.Buffer + PAK_RNG_CRC_MODE_EEPROM_ADDR,     1 );
                        memcpy( &( PacketParams.Params.LoRa.InvertIQ ),            Eeprom.Buffer + PAK_RNG_IQ_INV_EEPROM_ADDR,       1 );

                        Radio.SetPacketType( ModulationParams.PacketType );
                        Radio.SetModulationParams( &ModulationParams );
                        Radio.SetPacketParams( &PacketParams );
                        Radio.SetRangingRequestAddress( Eeprom.EepromData.DemoSettings.RngAddress );
                        Radio.SetRangingCalibration( Eeprom.EepromData.DemoSettings.RngCalib );
                        Radio.SetTxParams( Eeprom.EepromData.DemoSettings.TxPower, RADIO_RAMP_20_US );

                        MeasuredChannels = 0;
                        RngResultIndex   = 0;
                        SendNextPacket.attach_us( &SendNextPacketEvent, Eeprom.EepromData.DemoSettings.RngReqDelay * 1000 );
                        DemoInternalState = APP_RNG;
                    }
                    else
                    {
                        DemoInternalState = APP_RANGING_CONFIG;
                    }
                }
                else
                {
                    DemoInternalState = APP_RANGING_CONFIG;
                }
                break;

            case APP_TX:
                TX_LED = 0;
                if( Eeprom.EepromData.DemoSettings.RngStatus == RNG_INIT )
                {
                    RX_LED = 1;
                    Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RNG_COM_TIMEOUT } );
                    DemoInternalState = APP_IDLE;
                }
                else
                {
                    DemoInternalState = APP_RANGING_CONFIG;
                }
                break;

            case APP_RX_TIMEOUT:
                RX_LED = 0;
                Eeprom.EepromData.DemoSettings.RngStatus = RNG_TIMEOUT;
                DemoInternalState = APP_RANGING_CONFIG;
                Eeprom.EepromData.DemoSettings.HoldDemo = true;
                refreshDisplay = 1; // display error on token color (RNG_TIMEOUT)
                break;

            case APP_RX_ERROR:
                RX_LED = 0;
                DemoInternalState = APP_RANGING_CONFIG;
                Eeprom.EepromData.DemoSettings.HoldDemo = true;
                refreshDisplay = 1; // display error on token color (RNG_TIMEOUT)
                break;

            case APP_TX_TIMEOUT:
                TX_LED = 0;
                DemoInternalState = APP_RANGING_CONFIG;
                Eeprom.EepromData.DemoSettings.HoldDemo = true;
                refreshDisplay = 1; // display error on token color (RNG_TIMEOUT)
                break;

            case APP_IDLE: // do nothing
                break;

            default:
                DemoInternalState = APP_RANGING_CONFIG;
                Eeprom.EepromData.DemoSettings.HoldDemo = true;
                break;
        }
    }
    else    // Slave
    {
        switch( DemoInternalState )
        {
            case APP_RANGING_CONFIG:
                Eeprom.EepromData.DemoSettings.RngStatus = RNG_INIT;
                ModulationParams.PacketType = PACKET_TYPE_LORA;
                PacketParams.PacketType     = PACKET_TYPE_LORA;
                memcpy( &( ModulationParams.Params.LoRa.SpreadingFactor ), Eeprom.Buffer + MOD_RNG_SPREADF_EEPROM_ADDR,      1 );
                memcpy( &( ModulationParams.Params.LoRa.Bandwidth ),       Eeprom.Buffer + MOD_RNG_BW_EEPROM_ADDR,           1 );
                memcpy( &( ModulationParams.Params.LoRa.CodingRate ),      Eeprom.Buffer + MOD_RNG_CODERATE_EEPROM_ADDR,     1 );
                memcpy( &( PacketParams.Params.LoRa.PreambleLength ),      Eeprom.Buffer + PAK_RNG_PREAMBLE_LEN_EEPROM_ADDR, 1 );
                memcpy( &( PacketParams.Params.LoRa.HeaderType ),          Eeprom.Buffer + PAK_RNG_HEADERTYPE_EEPROM_ADDR,   1 );
                PacketParams.Params.LoRa.PayloadLength = 9;
                memcpy( &( PacketParams.Params.LoRa.Crc ),                 Eeprom.Buffer + PAK_RNG_CRC_MODE_EEPROM_ADDR,     1 );
                memcpy( &( PacketParams.Params.LoRa.InvertIQ ),            Eeprom.Buffer + PAK_RNG_IQ_INV_EEPROM_ADDR,       1 );
                Radio.SetPacketType( ModulationParams.PacketType );
                Radio.SetModulationParams( &ModulationParams );
                Radio.SetPacketParams( &PacketParams );
                Radio.SetRfFrequency( Eeprom.EepromData.DemoSettings.Frequency );
                RX_LED = 1;
                // use listen mode here instead of rx continuous
                Radio.SetRx( ( TickTime_t ) { RADIO_TICK_SIZE_1000_US, 0xFFFF } );
                DemoInternalState = APP_IDLE;
                break;

            case APP_RNG:
                if( SendNext == true )
                {
                    SendNext = false;
                    MeasuredChannels++;
                    if( MeasuredChannels <= Eeprom.EepromData.DemoSettings.RngRequestCount )
                    {
                        Radio.SetRfFrequency( Channels[CurrentChannel] );
                        RX_LED = 1;
                        switch( Eeprom.EepromData.DemoSettings.RngAntenna )
                        {
                            case DEMO_RNG_ANT_1:
                                //ANT_SW = 1; // ANT1
                                Eeprom.EepromData.DemoSettings.AntennaSwitch = 0;
                                CurrentChannel++;
                                if( CurrentChannel >= CHANNELS )
                                {
                                    CurrentChannel -= CHANNELS;
                                }
                                break;

                            case DEMO_RNG_ANT_0:
                                //ANT_SW = 0; // ANT0
                                Eeprom.EepromData.DemoSettings.AntennaSwitch = 1;
                                CurrentChannel++;
                                if( CurrentChannel >= CHANNELS )
                                {
                                    CurrentChannel -= CHANNELS;
                                }
                                break;

                            case DEMO_RNG_ANT_BOTH:
                                if( ANT_SW == 1 )
                                {
                                    //ANT_SW = 0;
                                    Eeprom.EepromData.DemoSettings.AntennaSwitch = 1;
                                }
                                else
                                {
                                    //ANT_SW = 1;
                                    Eeprom.EepromData.DemoSettings.AntennaSwitch = 0;
                                    CurrentChannel++;
                                    if( CurrentChannel >= CHANNELS )
                                    {
                                        CurrentChannel -= CHANNELS;
                                    }
                                }
                                break;
                        }
                        SetAntennaSwitch( );
                        DemoInternalState = APP_IDLE;
                        Radio.SetRx( ( TickTime_t ){ RADIO_TICK_SIZE_1000_US, Eeprom.EepromData.DemoSettings.RngReqDelay } );
                    }
                    else
                    {
                        Radio.SetStandby( STDBY_RC );
                        refreshDisplay = 1;
                        SendNextPacket.detach( );
                        Eeprom.EepromData.DemoSettings.RngStatus = RNG_VALID;
                        DemoInternalState = APP_RANGING_CONFIG;
                    }
                }
                break;

            case APP_RANGING_DONE:
                RX_LED = 0;
                Eeprom.EepromData.DemoSettings.CntPacketRxOK++;
                DemoInternalState = APP_RNG;
                break;

            case APP_RANGING_TIMEOUT:
                RX_LED = 0;
                DemoInternalState = APP_RNG;
                break;

            case APP_RX:
                RX_LED = 0;
                if( Eeprom.EepromData.DemoSettings.RngStatus == RNG_INIT )
                {
                    Radio.GetPayload( Buffer, &BufferSize, BUFFER_SIZE );
                    Radio.GetPacketStatus( &PacketStatus );
                    if( ( BufferSize > 0 ) && \
                        ( Buffer[0] == ( ( Eeprom.EepromData.DemoSettings.RngAddress >> 24 ) & 0xFF ) ) && \
                        ( Buffer[1] == ( ( Eeprom.EepromData.DemoSettings.RngAddress >> 16 ) & 0xFF ) ) && \
                        ( Buffer[2] == ( ( Eeprom.EepromData.DemoSettings.RngAddress >>  8 ) & 0xFF ) ) && \
                        ( Buffer[3] == (   Eeprom.EepromData.DemoSettings.RngAddress         & 0xFF ) ) )
                    {
                        Eeprom.EepromData.DemoSettings.RngFei    = Radio.GetFrequencyError( );
                        Eeprom.EepromData.DemoSettings.RssiValue = PacketStatus.LoRa.RssiPkt;
                        Eeprom.EepromData.DemoSettings.CntPacketTx++;
                        CurrentChannel                                 = Buffer[4];
                        Eeprom.EepromData.DemoSettings.RngAntenna      = Buffer[5];
                        Eeprom.EepromData.DemoSettings.RngRequestCount = Buffer[6];
                        wait_us( 10 );
                        Buffer[4] = ( ( ( int32_t )Eeprom.EepromData.DemoSettings.RngFei ) >> 24 ) & 0xFF ;
                        Buffer[5] = ( ( ( int32_t )Eeprom.EepromData.DemoSettings.RngFei ) >> 16 ) & 0xFF ;
                        Buffer[6] = ( ( ( int32_t )Eeprom.EepromData.DemoSettings.RngFei ) >>  8 ) & 0xFF ;
                        Buffer[7] = ( ( ( int32_t )Eeprom.EepromData.DemoSettings.RngFei ) & 0xFF );
                        Buffer[8] = Eeprom.EepromData.DemoSettings.RssiValue;
                        TX_LED = 1;
                        Radio.SendPayload( Buffer, 9, ( TickTime_t ){ RADIO_TICK_SIZE_1000_US, RNG_COM_TIMEOUT } );
                        DemoInternalState = APP_IDLE;
                    }
                    else
                    {
                        DemoInternalState = APP_RANGING_CONFIG;
                    }
                }
                else
                {
                    DemoInternalState = APP_RANGING_CONFIG;
                }
                break;

            case APP_TX:
                TX_LED = 0;
                if( Eeprom.EepromData.DemoSettings.RngStatus == RNG_INIT )
                {
                    Eeprom.EepromData.DemoSettings.RngStatus = RNG_PROCESS;

                    ModulationParams.PacketType = PACKET_TYPE_RANGING;
                    PacketParams.PacketType     = PACKET_TYPE_RANGING;

                    memcpy( &( ModulationParams.Params.LoRa.SpreadingFactor ), Eeprom.Buffer + MOD_RNG_SPREADF_EEPROM_ADDR,      1 );
                    memcpy( &( ModulationParams.Params.LoRa.Bandwidth ),       Eeprom.Buffer + MOD_RNG_BW_EEPROM_ADDR,           1 );
                    memcpy( &( ModulationParams.Params.LoRa.CodingRate ),      Eeprom.Buffer + MOD_RNG_CODERATE_EEPROM_ADDR,     1 );
                    memcpy( &( PacketParams.Params.LoRa.PreambleLength ),      Eeprom.Buffer + PAK_RNG_PREAMBLE_LEN_EEPROM_ADDR, 1 );
                    memcpy( &( PacketParams.Params.LoRa.HeaderType ),          Eeprom.Buffer + PAK_RNG_HEADERTYPE_EEPROM_ADDR,   1 );
                    PacketParams.Params.LoRa.PayloadLength = 10;
                    memcpy( &( PacketParams.Params.LoRa.Crc ),                 Eeprom.Buffer + PAK_RNG_CRC_MODE_EEPROM_ADDR,     1 );
                    memcpy( &( PacketParams.Params.LoRa.InvertIQ ),            Eeprom.Buffer + PAK_RNG_IQ_INV_EEPROM_ADDR,       1 );

                    Radio.SetPacketType( ModulationParams.PacketType );

                    Radio.SetModulationParams( &ModulationParams );
                    Radio.SetPacketParams( &PacketParams );
                    Radio.SetDeviceRangingAddress( Eeprom.EepromData.DemoSettings.RngAddress );
                    Radio.SetRangingCalibration( Eeprom.EepromData.DemoSettings.RngCalib );
                    Radio.SetTxParams( Eeprom.EepromData.DemoSettings.TxPower, RADIO_RAMP_20_US );
                    Eeprom.EepromData.DemoSettings.CntPacketRxOK = 0;
                    MeasuredChannels = 0;
                    Eeprom.EepromData.DemoSettings.CntPacketRxKOSlave = 0;
                    SendNextPacket.attach_us( &SendNextPacketEvent, Eeprom.EepromData.DemoSettings.RngReqDelay * 1000 );
                    DemoInternalState = APP_RNG;
                }
                else
                {
                    DemoInternalState = APP_RANGING_CONFIG;
                }
                break;

            case APP_RX_TIMEOUT:
                RX_LED = 0;
                DemoInternalState = APP_RANGING_CONFIG;
                break;

            case APP_RX_ERROR:
                RX_LED = 0;
                DemoInternalState = APP_RANGING_CONFIG;
                break;

            case APP_TX_TIMEOUT:
                TX_LED = 0;
                DemoInternalState = APP_RANGING_CONFIG;
                break;

            case APP_IDLE: // do nothing
                if( Eeprom.EepromData.DemoSettings.CntPacketRxKOSlave > DEMO_RNG_CHANNELS_COUNT_MAX )
                {
                    Eeprom.EepromData.DemoSettings.CntPacketRxKOSlave = 0;
                    refreshDisplay = 1;
                    RX_LED = 0;
                    DemoInternalState = APP_RANGING_CONFIG;
                    SendNextPacket.detach( );
                }
                break;

            default:
                DemoInternalState = APP_RANGING_CONFIG;
                SendNextPacket.detach( );
                break;
        }
    }
    return refreshDisplay;
}

// ************************        Utils            ****************************
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *****************************************************************************

void InitDemoApplication( void )
{
    RX_LED = 1;
    TX_LED = 1;

    SetAntennaSwitch( );

    wait_ms( 500 ); // wait for on board DC/DC start-up time

    Radio.Init( );

    // Can also be set in LDO mode but consume more power
    Radio.SetRegulatorMode( ( RadioRegulatorModes_t )Eeprom.EepromData.DemoSettings.RadioPowerMode );
    Radio.SetStandby( STDBY_RC );

    memset( &Buffer, 0x00, BufferSize );

    RX_LED = 0;
    TX_LED = 0;

    PacketRxSequence = 0;
    PacketRxSequencePrev = 0;
    Eeprom.EepromData.DemoSettings.CntPacketTx    = 0;
    Eeprom.EepromData.DemoSettings.CntPacketRxOK  = 0;
    Eeprom.EepromData.DemoSettings.CntPacketRxKO  = 0;
    Eeprom.EepromData.DemoSettings.RxTimeOutCount = 0;
}

void StopDemoApplication( void )
{
    if( DemoRunning == true )
    {
        __disable_irq( );    // Disable Interrupts

#ifdef PRINT_DEBUG
        printf( "StopDemoApplication\n\r" );
#endif

        if( Radio.GetOpMode( ) == MODE_SLEEP )
        {
            Radio.Wakeup( );
            InitializeDemoParameters( Eeprom.EepromData.DemoSettings.ModulationType );
        }
        RX_LED = 0;
        TX_LED = 0;
        DemoRunning = false;
        SendNext = false;
        ReceiveNext = false;
        PacketRxSequence = 0;
        PacketRxSequencePrev = 0;
        Eeprom.EepromData.DemoSettings.CntPacketTx    = 0;
        Eeprom.EepromData.DemoSettings.CntPacketRxOK  = 0;
        Eeprom.EepromData.DemoSettings.CntPacketRxKO  = 0;
        Eeprom.EepromData.DemoSettings.RxTimeOutCount = 0;

        Radio.SetAutoFs( false );
        DemoInternalState = APP_IDLE;
        Radio.SetStandby( STDBY_RC );
        Radio.ClearIrqStatus( IRQ_RADIO_ALL );
        SendNextPacket.detach( );
        ReceiveNextPacket.detach( );
        
        __enable_irq( );     // Enable Interrupts
    }
}

/*
 * Function still being implemented >>> To be completed 
 * WARNING: Computation is in float and his really slow
 * LongInterLeaving vs LegacyInterLeaving has no influence on TimeOnAir.
 */
uint16_t GetTimeOnAir( uint8_t modulation )
{
    uint16_t result = 2000;
    double tPayload = 0.0;
    
    if( modulation == PACKET_TYPE_LORA )
    {
        uint16_t bw = 0.0;
        double nPayload = 0.0;
        double ts = 0.0;

        uint8_t SF = Eeprom.EepromData.ModulationParams.Params.LoRa.SpreadingFactor >> 4;
        uint8_t crc = ( Eeprom.EepromData.PacketParams.Params.LoRa.Crc == LORA_CRC_ON ) ? 16 : 0; // 16 bit if present else 0
        uint8_t header = ( Eeprom.EepromData.PacketParams.Params.LoRa.HeaderType == LORA_PACKET_VARIABLE_LENGTH ) ? 20 : 0; // 20 if present else 0
        uint16_t payload = 8 * Eeprom.EepromData.PacketParams.Params.LoRa.PayloadLength;
        uint8_t CR = Eeprom.EepromData.ModulationParams.Params.LoRa.CodingRate;

        switch( Eeprom.EepromData.ModulationParams.Params.LoRa.Bandwidth )
        {
            case LORA_BW_0200:
                bw = 203;
                break;

            case LORA_BW_0400:
                bw = 406;
                break;

            case LORA_BW_0800:
                bw = 812;
                break;

            case LORA_BW_1600:
                bw = 1625;
                break;

            default:
                break;
        }

        if( SF < 7 )
        {
            nPayload = max( ( ( double )( payload + crc -(4 * SF) + header ) ), 0.0 );
            nPayload = nPayload / ( double )( 4 * SF );
            nPayload = ceil( nPayload );
            nPayload = nPayload * ( CR + 4 );
            nPayload = nPayload + Eeprom.EepromData.PacketParams.Params.LoRa.PreambleLength + 6.25 + 8;
        }
        else if( SF > 10 )
        {
            nPayload = max( ( ( double )( payload + crc -(4 * SF) + 8 + header ) ), 0.0 );
            nPayload = nPayload / ( double )( 4 * ( SF - 2 ) );
            nPayload = ceil( nPayload );
            nPayload = nPayload * ( CR + 4 );
            nPayload = nPayload + Eeprom.EepromData.PacketParams.Params.LoRa.PreambleLength + 4.25 + 8;
        }
        else
        {
            nPayload = max( ( ( double )( payload + crc -(4 * SF) + 8 + header ) ), 0.0 );
            nPayload = nPayload / ( double )( 4 * SF );
            nPayload = ceil( nPayload );
            nPayload = nPayload * ( CR + 4 );
            nPayload = nPayload + Eeprom.EepromData.PacketParams.Params.LoRa.PreambleLength + 4.25 + 8;
        }
        ts = ( double )( 1 << SF ) / ( double )( bw );
        tPayload = nPayload * ts;
#ifdef PRINT_DEBUG
        printf( "ToA LoRa: %f \n\r", tPayload );
#endif
        result = ceil( tPayload );   
    }
    else if( modulation == PACKET_TYPE_FLRC )
    {
        uint16_t BitCount = 0;
        uint16_t BitCountCoded = 0;

        BitCount = 4 + ( Eeprom.EepromData.PacketParams.Params.Flrc.PreambleLength >> 4 ) * 4;              // AGC preamble 
        BitCount = BitCount + 32;                                                                           // Sync Word
        BitCount = BitCount + 21;                                                                           // Preamble
        BitCount = BitCount + ( ( Eeprom.EepromData.PacketParams.Params.Flrc.HeaderType == RADIO_PACKET_VARIABLE_LENGTH ) ? 16 : 0 );

        switch( Eeprom.EepromData.ModulationParams.Params.Flrc.CodingRate )
        {
            case FLRC_CR_3_4:
                BitCountCoded =  6 + ( Eeprom.EepromData.PacketParams.Params.Flrc.CrcLength >> 4 ) * 8;
                BitCountCoded = BitCountCoded + Eeprom.EepromData.PacketParams.Params.Flrc.PayloadLength * 8;
                BitCountCoded = ( uint16_t )( ( ( double )BitCountCoded * 4.0 ) / 3.0 );
                break;

            case FLRC_CR_1_0:
                BitCountCoded =  ( Eeprom.EepromData.PacketParams.Params.Flrc.CrcLength >> 4 ) * 8;
                BitCountCoded = BitCountCoded + Eeprom.EepromData.PacketParams.Params.Flrc.PayloadLength * 8;
                break;

            default:
            case FLRC_CR_1_2:
                BitCountCoded =  6 + ( Eeprom.EepromData.PacketParams.Params.Flrc.CrcLength >> 4 ) * 8;
                BitCountCoded = BitCountCoded + Eeprom.EepromData.PacketParams.Params.Flrc.PayloadLength * 8;
                BitCountCoded = BitCountCoded << 1;
                break;
        }
        BitCount = BitCount + BitCountCoded;

        switch( Eeprom.EepromData.ModulationParams.Params.Flrc.BitrateBandwidth )
        {
            case FLRC_BR_1_300_BW_1_2:
                tPayload = ( double )BitCount / 1300.0;
                break;

            case FLRC_BR_1_040_BW_1_2:
                tPayload = ( double )BitCount / 1040.0;
                break;

            case FLRC_BR_0_650_BW_0_6:
                tPayload = ( double )BitCount / 650.0;
                break;

            case FLRC_BR_0_520_BW_0_6:
                tPayload = ( double )BitCount / 520.0;
                break;

            case FLRC_BR_0_325_BW_0_3:
                tPayload = ( double )BitCount / 325.0;
                break;

            case FLRC_BR_0_260_BW_0_3:
                tPayload = ( double )BitCount / 260.0;
                break;
            
            default:
                break;
        }

        printf( "ToA FLRC: %f \n\r", tPayload );

        result = ceil( tPayload );
    }
    else if( modulation == PACKET_TYPE_GFSK )
    {
        uint16_t BitCount = 0;
        
        BitCount = 4 + ( Eeprom.EepromData.PacketParams.Params.Gfsk.PreambleLength >> 4 ) * 4;              // preamble
        BitCount = BitCount + 8 + ( Eeprom.EepromData.PacketParams.Params.Gfsk.SyncWordLength >> 1 ) * 8;   // sync word
        BitCount = BitCount + ( ( Eeprom.EepromData.PacketParams.Params.Gfsk.HeaderType == RADIO_PACKET_VARIABLE_LENGTH ) ? 8 : 0 );
        BitCount = BitCount + Eeprom.EepromData.PacketParams.Params.Gfsk.PayloadLength * 8;
        BitCount = BitCount + ( Eeprom.EepromData.PacketParams.Params.Gfsk.CrcLength >> 4 ) * 8;
        
        switch( Eeprom.EepromData.ModulationParams.Params.Gfsk.BitrateBandwidth )
        {
            case GFSK_BLE_BR_2_000_BW_2_4:
                tPayload = ( double )BitCount / 2000.0 ;
                break;

            case GFSK_BLE_BR_1_600_BW_2_4:
                tPayload = ( double )BitCount / 1600.0 ;
                break;

            case GFSK_BLE_BR_1_000_BW_2_4:
            case GFSK_BLE_BR_1_000_BW_1_2:
                tPayload = ( double )BitCount / 1000.0;
                break;

            case GFSK_BLE_BR_0_800_BW_2_4:
            case GFSK_BLE_BR_0_800_BW_1_2:
                tPayload = ( double )BitCount / 800.0;
                break;

            case GFSK_BLE_BR_0_500_BW_1_2:
            case GFSK_BLE_BR_0_500_BW_0_6:
                tPayload = ( double )BitCount / 500.0;
                break;

            case GFSK_BLE_BR_0_400_BW_1_2:
            case GFSK_BLE_BR_0_400_BW_0_6:
                tPayload = ( double )BitCount / 400.0;
                break;

            case GFSK_BLE_BR_0_250_BW_0_6:
            case GFSK_BLE_BR_0_250_BW_0_3:
                tPayload = ( double )BitCount / 250.0;
                break;

            case GFSK_BLE_BR_0_125_BW_0_3:
                tPayload = ( double )BitCount / 125.0;
                break;

            default:
                break;
        }
#ifdef PRINT_DEBUG
        printf( "ToA GFSK: %f \n\r", tPayload );
#endif
        result = ceil( tPayload );
    }
    
    return result;
}

void InitializeDemoParameters( uint8_t modulation )
{
    Radio.SetStandby( STDBY_RC );

    Radio.SetRegulatorMode( ( RadioRegulatorModes_t )Eeprom.EepromData.DemoSettings.RadioPowerMode );

#ifdef PRINT_DEBUG
    printf("> InitializeDemoParameters\n\r");
#endif
    if( modulation == PACKET_TYPE_LORA )
    {
#ifdef PRINT_DEBUG
        printf("set param LORA for demo\n\r");
#endif
        ModulationParams.PacketType = PACKET_TYPE_LORA;
        PacketParams.PacketType     = PACKET_TYPE_LORA;

        ModulationParams.Params.LoRa.SpreadingFactor = ( RadioLoRaSpreadingFactors_t )  Eeprom.EepromData.DemoSettings.ModulationParam1;
        ModulationParams.Params.LoRa.Bandwidth       = ( RadioLoRaBandwidths_t )        Eeprom.EepromData.DemoSettings.ModulationParam2;
        ModulationParams.Params.LoRa.CodingRate      = ( RadioLoRaCodingRates_t )       Eeprom.EepromData.DemoSettings.ModulationParam3;
        PacketParams.Params.LoRa.PreambleLength      =                                  Eeprom.EepromData.DemoSettings.PacketParam1;
        PacketParams.Params.LoRa.HeaderType          = ( RadioLoRaPacketLengthsModes_t )Eeprom.EepromData.DemoSettings.PacketParam2;
        PacketParams.Params.LoRa.PayloadLength       =                                  Eeprom.EepromData.DemoSettings.PacketParam3;
        PacketParams.Params.LoRa.Crc                 = ( RadioLoRaCrcModes_t )          Eeprom.EepromData.DemoSettings.PacketParam4;
        PacketParams.Params.LoRa.InvertIQ            = ( RadioLoRaIQModes_t )           Eeprom.EepromData.DemoSettings.PacketParam5;

        Eeprom.EepromData.DemoSettings.PayloadLength = PacketParams.Params.LoRa.PayloadLength;

        Radio.SetLNAGainSetting(LNA_LOW_POWER_MODE);
    }
    else if( modulation == PACKET_TYPE_FLRC )
    {
#ifdef PRINT_DEBUG
        printf("set param FLRC for demo\n\r");
#endif
        ModulationParams.PacketType = PACKET_TYPE_FLRC;
        PacketParams.PacketType     = PACKET_TYPE_FLRC;

        ModulationParams.Params.Flrc.BitrateBandwidth  = ( RadioFlrcBitrates_t )       Eeprom.EepromData.DemoSettings.ModulationParam1;
        ModulationParams.Params.Flrc.CodingRate        = ( RadioFlrcCodingRates_t )    Eeprom.EepromData.DemoSettings.ModulationParam2;
        ModulationParams.Params.Flrc.ModulationShaping = ( RadioModShapings_t )        Eeprom.EepromData.DemoSettings.ModulationParam3;
        PacketParams.Params.Flrc.PreambleLength        = ( RadioPreambleLengths_t )    Eeprom.EepromData.DemoSettings.PacketParam1;
        PacketParams.Params.Flrc.SyncWordLength        = ( RadioFlrcSyncWordLengths_t )Eeprom.EepromData.DemoSettings.PacketParam2;
        PacketParams.Params.Flrc.SyncWordMatch         = ( RadioSyncWordRxMatchs_t )   Eeprom.EepromData.DemoSettings.PacketParam3;
        PacketParams.Params.Flrc.HeaderType            = ( RadioPacketLengthModes_t )  Eeprom.EepromData.DemoSettings.PacketParam4;
        PacketParams.Params.Flrc.PayloadLength         =                               Eeprom.EepromData.DemoSettings.PacketParam5;
        PacketParams.Params.Flrc.CrcLength             = ( RadioCrcTypes_t )           Eeprom.EepromData.DemoSettings.PacketParam6;
        PacketParams.Params.Flrc.Whitening             = ( RadioWhiteningModes_t )     Eeprom.EepromData.DemoSettings.PacketParam7;

        Eeprom.EepromData.DemoSettings.PayloadLength = PacketParams.Params.Flrc.PayloadLength;

        Radio.SetLNAGainSetting(LNA_HIGH_SENSITIVITY_MODE);
    }
    else if( modulation == PACKET_TYPE_GFSK )
    {
#ifdef PRINT_DEBUG
        printf("set param GFSK for demo\n\r");
#endif
        ModulationParams.PacketType = PACKET_TYPE_GFSK;
        PacketParams.PacketType     = PACKET_TYPE_GFSK;

        ModulationParams.Params.Gfsk.BitrateBandwidth  = ( RadioGfskBleBitrates_t )  Eeprom.EepromData.DemoSettings.ModulationParam1;
        ModulationParams.Params.Gfsk.ModulationIndex   = ( RadioGfskBleModIndexes_t )Eeprom.EepromData.DemoSettings.ModulationParam2;
        ModulationParams.Params.Gfsk.ModulationShaping = ( RadioModShapings_t )      Eeprom.EepromData.DemoSettings.ModulationParam3;
        PacketParams.Params.Gfsk.PreambleLength        = ( RadioPreambleLengths_t )  Eeprom.EepromData.DemoSettings.PacketParam1;
        PacketParams.Params.Gfsk.SyncWordLength        = ( RadioSyncWordLengths_t )  Eeprom.EepromData.DemoSettings.PacketParam2;
        PacketParams.Params.Gfsk.SyncWordMatch         = ( RadioSyncWordRxMatchs_t ) Eeprom.EepromData.DemoSettings.PacketParam3;
        PacketParams.Params.Gfsk.HeaderType            = ( RadioPacketLengthModes_t )Eeprom.EepromData.DemoSettings.PacketParam4;
        PacketParams.Params.Gfsk.PayloadLength         =                             Eeprom.EepromData.DemoSettings.PacketParam5;
        PacketParams.Params.Gfsk.CrcLength             = ( RadioCrcTypes_t )         Eeprom.EepromData.DemoSettings.PacketParam6;
        PacketParams.Params.Gfsk.Whitening             = ( RadioWhiteningModes_t )   Eeprom.EepromData.DemoSettings.PacketParam7;

        Eeprom.EepromData.DemoSettings.PayloadLength = PacketParams.Params.Gfsk.PayloadLength;

        Radio.SetLNAGainSetting(LNA_HIGH_SENSITIVITY_MODE);
    }
    if( modulation == PACKET_TYPE_RANGING )
    {
        Radio.SetBufferBaseAddresses( 0x00, 0x00 );
        Radio.SetTxParams( Eeprom.EepromData.DemoSettings.TxPower, RADIO_RAMP_20_US );
        memcpy( &( ModulationParams.Params.LoRa.SpreadingFactor ), Eeprom.Buffer + MOD_RNG_SPREADF_EEPROM_ADDR, 1 );
        memcpy( &( ModulationParams.Params.LoRa.Bandwidth ),       Eeprom.Buffer + MOD_RNG_BW_EEPROM_ADDR,      1 );
        switch( ModulationParams.Params.LoRa.Bandwidth )
        {
            case LORA_BW_0400:
                Eeprom.EepromData.DemoSettings.RngCalib     = RNG_CALIB_0400[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
                Eeprom.EepromData.DemoSettings.RngFeiFactor = ( double )RNG_FGRAD_0400[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
                Eeprom.EepromData.DemoSettings.RngReqDelay  = RNG_TIMER_MS >> ( 0 + 10 - ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) );
                break;

            case LORA_BW_0800:
                Eeprom.EepromData.DemoSettings.RngCalib     = RNG_CALIB_0800[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
                Eeprom.EepromData.DemoSettings.RngFeiFactor = ( double )RNG_FGRAD_0800[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
                Eeprom.EepromData.DemoSettings.RngReqDelay  = RNG_TIMER_MS >> ( 1 + 10 - ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) );
                break;

            case LORA_BW_1600:
                Eeprom.EepromData.DemoSettings.RngCalib     = RNG_CALIB_1600[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
                Eeprom.EepromData.DemoSettings.RngFeiFactor = ( double )RNG_FGRAD_1600[ ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - 5 ];
                Eeprom.EepromData.DemoSettings.RngReqDelay  = RNG_TIMER_MS >> ( 2 + 10 - ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) );
                break;
        }

        Radio.SetPollingMode( );
        Radio.SetLNAGainSetting(LNA_LOW_POWER_MODE);
    }
    else
    {
        Radio.SetStandby( STDBY_RC );
        Radio.SetPacketType( ModulationParams.PacketType );
        Radio.SetRfFrequency( Eeprom.EepromData.DemoSettings.Frequency );
        Radio.SetBufferBaseAddresses( 0x00, 0x00 );
        Radio.SetModulationParams( &ModulationParams );
        Radio.SetPacketParams( &PacketParams );
        // only used in GFSK, FLRC (4 bytes max) and BLE mode
        Radio.SetSyncWord( 1, ( uint8_t[] ){ 0xDD, 0xA0, 0x96, 0x69, 0xDD } );
        // only used in GFSK, FLRC
        uint8_t crcSeedLocal[2] = {0x45, 0x67};
        Radio.SetCrcSeed( crcSeedLocal );
        Radio.SetCrcPolynomial( 0x0123 );
        Radio.SetTxParams( Eeprom.EepromData.DemoSettings.TxPower, RADIO_RAMP_20_US );
        Radio.SetPollingMode( );
    }
}

/*!
 * \brief Callback of ticker PerSendNextPacket
 */
void SendNextPacketEvent( void )
{
    SendNext = true;
    if( Eeprom.EepromData.DemoSettings.RngStatus == RNG_PROCESS )
    {
        Eeprom.EepromData.DemoSettings.CntPacketRxKOSlave++;
    }
}

/*!
 * \brief Callback of ticker ReceiveNextPacket
 */
void ReceiveNextPacketEvent( void )
{
    ReceiveNext = true;
}

uint8_t CheckDistance( void )
{
    double displayRange = 0.0;

    uint16_t j = 0;
    uint16_t i;

#ifdef PRINT_DEBUG
    printf( "#id: %d", Eeprom.EepromData.DemoSettings.CntPacketTx );
#endif
    if( RngResultIndex > 0 )
    {
        for( i = 0; i < RngResultIndex; ++i )
        {
            RawRngResults[i] = RawRngResults[i] - ( Eeprom.EepromData.DemoSettings.RngFeiFactor * Eeprom.EepromData.DemoSettings.RngFei / 1000 );
        }

        for (int i = RngResultIndex - 1; i > 0; --i) 
        {
            for (int j = 0; j < i; ++j) 
            {
                if (RawRngResults[j] > RawRngResults[j+1]) 
                {
                    int temp = RawRngResults[j];
                    RawRngResults[j] = RawRngResults[j+1];
                    RawRngResults[j+1] = temp;
                }
            }
        }
        double median;
        if ((RngResultIndex % 2) == 0) 
        {
            median = (RawRngResults[RngResultIndex/2] + RawRngResults[(RngResultIndex/2) - 1])/2.0;
        }
        else
        {
            median = RawRngResults[RngResultIndex/2];
        }

        if( median < 100 )
        {
            printf("median: %f \n\r", median );
            // Apply the short range correction and RSSI short range improvement below 50 m
            displayRange = Sx1280RangingCorrection::ComputeRangingCorrectionPolynome(
                ModulationParams.Params.LoRa.SpreadingFactor,
                ModulationParams.Params.LoRa.Bandwidth,
                median
            );
            printf("Corrected range: %f \n\r", displayRange );
            //displayRange = t0 + t1 * rssi + t2 * pow(rssi,2) + t3 * pow(rssi, 3) + t4 * median + t5 * pow(median,2) + t6 * pow(median, 3) + t7 * pow(median, 4) ;
            //printf("displayRange %f \n\r", displayRange );
//            double correctedRange = 0;
//            uint8_t k = 0;
//            uint8_t order = 6;
//            for( k = 1; k <= (order+1); k++ )                    // loop though each polynomial term and sum
//            {
//                correctedRange = correctedRange + p[k] * pow( median, ( order + 1 - k ) );
//                printf("correctedRange[%d] %f \n\r", k, correctedRange );
//            }
//            printf("Final correctedRange %f \n\r", correctedRange );
//            displayRange = correctedRange - 2;
        }
        else
        {
            displayRange = median;
        }

        if( j < DEMO_RNG_CHANNELS_COUNT_MIN )
        {
            Eeprom.EepromData.DemoSettings.RngStatus = RNG_PER_ERROR;
        }
        else
        {
            Eeprom.EepromData.DemoSettings.RngStatus = RNG_VALID;
        }

        if( displayRange < 0 )
        {
            Eeprom.EepromData.DemoSettings.RngDistance = 0.0;
        }
        else
        {
            switch( Eeprom.EepromData.DemoSettings.RngUnit )
            {
                case DEMO_RNG_UNIT_SEL_M:
                    Eeprom.EepromData.DemoSettings.RngDistance = displayRange;
                    break;

                case DEMO_RNG_UNIT_SEL_YD:
                    Eeprom.EepromData.DemoSettings.RngDistance = displayRange * DEMO_RNG_UNIT_CONV_YD;
                    break;

                case DEMO_RNG_UNIT_SEL_MI:
                    Eeprom.EepromData.DemoSettings.RngDistance = displayRange * DEMO_RNG_UNIT_CONV_MI;
                    break;
            }
        }
    }
    printf( ", Rssi: %d, Zn: %3d, Zmoy: %5.1f, FEI: %d\r\n", Eeprom.EepromData.DemoSettings.RssiValue, j, displayRange, ( int32_t )Eeprom.EepromData.DemoSettings.RngFei );

    return j;
}

void LedBlink( void )
{
    if( ( TX_LED == 0 ) && ( RX_LED == 0 ) )
    {
        TX_LED = 1;
    }
    else if( ( TX_LED == 1 ) && ( RX_LED == 0 ) )
    {
        RX_LED = 1;
    }
    else if( ( TX_LED == 1 ) && ( RX_LED == 1 ) )
    {
        TX_LED = 0;
    }
    else
    {
        RX_LED = 0;
    }
}

void SetAntennaSwitch( void )
{
    if( Eeprom.EepromData.DemoSettings.AntennaSwitch == 0 )
    {
        ANT_SW = 1; // ANT1
    }
    else
    {
        ANT_SW = 0; // ANT0
    }
}

// ************************     Radio Callbacks     ****************************
// *                                                                           *
// * These functions are called through function pointer by the Radio low      *
// * level drivers                                                             *
// *                                                                           *
// *****************************************************************************
void OnTxDone( void )
{
    DemoInternalState = APP_TX;
}

void OnRxDone( void )
{
    DemoInternalState = APP_RX;
}

void OnTxTimeout( void )
{
    DemoInternalState = APP_TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    DemoInternalState = APP_RX_TIMEOUT;
}

void OnRxError( IrqErrorCode_t errorCode )
{
    DemoInternalState = APP_RX_ERROR;
}

void OnRangingDone( IrqRangingCode_t val )
{
    if( val == IRQ_RANGING_MASTER_VALID_CODE || val == IRQ_RANGING_SLAVE_VALID_CODE )
    {
        DemoInternalState = APP_RANGING_DONE;
    }
    else if( val == IRQ_RANGING_MASTER_ERROR_CODE || val == IRQ_RANGING_SLAVE_ERROR_CODE )
    {
        DemoInternalState = APP_RANGING_TIMEOUT;
    }
    else
    {
        DemoInternalState = APP_RANGING_TIMEOUT;
    }
}

void OnCadDone( bool channelActivityDetected )
{
}
