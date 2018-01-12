/*
  To do:
  * Add code header
  * Serial transmit buffering - availableForWrite()
*/

#include <EEPROM.h>

/* uFlow version */
#define VERSION "1.0"

/* Filter coefficient for smoothing pressure readings */
//#define FILTER_ALPHA    9

/* Maximum output value, dependent on number of solenoids
  2 solenoids = 2 bits = 0b11 = 3
*/
#define MASK_MAX        0.9

/* PID related */
#define NUM_LOOPS       2         // Number of PID loops
#define PID_DECIMALS    4         // Number of decimals to print
#define PID_INDEX_P     0
#define PID_INDEX_I     1
#define PID_INDEX_D     2
#define WINDUP_MAX      1         // Used to limit integrator windup

/* Pressure sensors related */
#define NUM_PSENSORS    2
#define PRES_DECIMALS   1
#define PSENSOR_REF     DEFAULT  // INTERNAL=1.1V, DEFAULT=5V
//#define PSENSOR_REF     INTERNAL  // INTERNAL=1.1V, DEFAULT=5V

/* EEPROM locations */
#define EEPROM_PZERO    0x00      // Allow 8 pressure sensors at 8 bytes each
#define EEPROM_PID      0x40

/* Pin locations */
#define P1_PIN          0
#define P2_PIN          1
#define SOL1_PIN        9
#define SOL2_PIN        10

/* Interrupt constants */
#define CLOCK_FREQ      16000000
#define INT_DIVISOR     8
#define INT_OVF         32
#define MAX_FREQ        100

/* Commands */
typedef enum
{
  MODE_NONE = 0,
  MODE_INVALID,
  MODE_MASK,
  MODE_AMPL,
  MODE_P,
  MODE_I,
  MODE_D,
  MODE_DEBUG,
  MODE_ZERO,
  MODE_RATIO,
} E_COMMS_MODE;

volatile unsigned int timer = 0;
volatile unsigned int overflow = 100;
volatile int go = 0;
int timerOffset = 0;
char freq;
float pressure_offsets[NUM_PSENSORS];

float p = 0.0;                  // Control Proportional coefficient
float i = 0.001;                // Control Integral coefficient
float d = 0.0;                  // Control Differential coefficient

const byte P_PINS[NUM_PSENSORS] = {P1_PIN, P2_PIN};
const byte SOL_PINS[NUM_LOOPS] = {SOL1_PIN, SOL2_PIN};

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

ISR(TIMER2_COMPA_vect)
{
  if ( ++timer >= overflow )
  {
    timer = 0;
    go = 1;
  }
}

void setFreq( int hz )
{
  if ( hz == 0 )
  {
    TIMSK2 &= ~_BV(OCIE2A);      // Disable Compare 2A Interrupt
    go = 0;
  }
  else
  {
    hz = constrain( hz, 1, MAX_FREQ );
    overflow = ( (long long)CLOCK_FREQ * 2 / INT_DIVISOR /
                 INT_OVF / hz + 1 ) / 2;
    TIMSK2 |= _BV(OCIE2A);      // Enable Compare 2A Interrupt
  }
  
  freq = hz;
}

float getFreq( void )
{
  return (float)CLOCK_FREQ / INT_DIVISOR / INT_OVF / overflow;
}

float getPressure( float anValue )
{
#if PSENSOR_REF == INTERNAL
  #define Vref 1.1
#else
  #define Vref 5.0
#endif
  #define Vsupply 5.0
  #define Pmax 1.0
  #define Pmin 0.0
  
  return ( ( anValue * Vref / 1024.0 - 0.1 * Vsupply ) * ( Pmax - Pmin ) /
           ( 0.8 * Vsupply ) + Pmin ) * 1000.0;
}

E_COMMS_MODE parseMode( char mode_char )
{
  E_COMMS_MODE comms_mode = MODE_NONE;
  
  switch ( mode_char )
  {
    case 'M':
    case 'm':
      comms_mode = MODE_MASK;
      break;
      
    case 'A':
    case 'a':
      comms_mode = MODE_AMPL;
      break;
      
    case 'P':
    case 'p':
      comms_mode = MODE_P;
      break;
      
    case 'I':
    case 'i':
      comms_mode = MODE_I;
      break;
      
    case 'D':
    case 'd':
      comms_mode = MODE_D;
      break;
      
    case 'X':
    case 'x':
      comms_mode = MODE_DEBUG;
      break;
      
    case 'Z':
    case 'z':
      comms_mode = MODE_ZERO;
      break;
      
    case 'R':
    case 'r':
      comms_mode = MODE_RATIO;
      break;
      
    default:
      comms_mode = MODE_INVALID;
      break;
  }
  
  return comms_mode;
}

void zero_pressure( byte sensor )
{
  #define ZERO_SAMPLES 6000
  
  char pin;
  short i;
  long sum = 0;
  
  Serial.print( String("Zeroing pressure sensor ") +
                sensor + String("...\r\n") );
  
  /* Find analogue pin for selected sensor */
  if ( sensor < NUM_PSENSORS )
    pin = P_PINS[sensor];
  else
  {
    Serial.print( "Invalid pressure sensor\r\n" );
    pin = -1;
  }
    
  if ( pin >= 0 )
  {
    /* Sum ZERO_SAMPLES pressure readings */
    for ( i=0; i<ZERO_SAMPLES; i++ )
      sum += analogRead( pin );
    
    /* Calculate average to two decimals and convert to pressure offset */
    pressure_offsets[sensor] =
      getPressure( ( (float)( sum * 100 / ZERO_SAMPLES ) ) / 100.0 );
    
    Serial.print( "Storing to EEPROM...\r\n" );
    
    /* Save sensor offset in EEPROM at correct location */
    EEPROM.put( EEPROM_PZERO + sensor * sizeof(*pressure_offsets),
                pressure_offsets[sensor] );
  }
  
  Serial.print( "Done\r\n" );
}

float load_PID( byte index, char name, float min, float max, float def )
{
  float constant;
  
  EEPROM.get( EEPROM_PID + index * sizeof(constant), constant );
  
  Serial.print( String("PID ") + String(name) );
  
  if ( constant >= min && constant <= max )
    Serial.print( String(" = ") + String(constant, PID_DECIMALS) );
  else
  {
    constant = def;
    Serial.print( String(" invalid.  Defaulting to ") +
                  String(constant, PID_DECIMALS) );
  }
  
  Serial.print( "\r\n" );
  
  return constant;
}

void save_PID( byte index, float constant )
{
  EEPROM.put( EEPROM_PID + index * sizeof(constant), constant );
}

void setup()
{
  int i;
  
  /* Initialize serial comms */
//  Serial.begin( 9600 );
  Serial.begin( 115200 );
  Serial.print( "\r\n\r\n" );
  Serial.print( "------------\r\n" );
  Serial.print( String("uFlow v") + VERSION + String("\r\n") );
  Serial.print( "Scaturio Ltd\r\n" );
  Serial.print( "------------\r\n" );
  Serial.print( "\r\n" );
  
  Serial.print( "Loading EEPROM Data...\r\n" );
  /* Load PID constants */
  p = load_PID( PID_INDEX_P, 'P', 0.0, 10.0, p );
  i = load_PID( PID_INDEX_I, 'I', 0.0, 10.0, i );
  d = load_PID( PID_INDEX_D, 'D', 0.0, 10.0, d );
  
  /* Load EEPROM data */
  for ( i=0; i<NUM_PSENSORS; i++ )
  {
    EEPROM.get( EEPROM_PZERO + i * sizeof(*pressure_offsets),
                pressure_offsets[i] );
    Serial.print( String("Pressure sensor ") + i + String(" offset = ") +
                  pressure_offsets[i] + String("mPSI\r\n") );
  }
  
  Serial.print( "\r\n" );
  
  Serial.print( "Using "
#if PSENSOR_REF == INTERNAL
    "internal 1.1V"
#else
    "default 5V"
#endif
    " ADC reference\r\n" );
  
  Serial.print( "\r\n" );
  Serial.print( "Ready\r\n\r\n" );
  
  /* Set solenoid pins as outputs */
  pinMode( SOL1_PIN, OUTPUT );
  pinMode( SOL2_PIN, OUTPUT );

  setPwmFrequency(3, INT_DIVISOR);   // Set timer0 frequency
  
#if PSENSOR_REF == INTERNAL
  analogReference( INTERNAL );
#else
  analogReference( DEFAULT );
#endif
  
  cli();
  
  /* CTC mode on timer 2 */
  TCCR2A = 0;                 // Clear timer mode
  TCCR2A |=  0b00000010;      // WGM20, WGM21 on bit 0, 1
  TCCR2B &= ~0b00001000;      // WGM22 on bit 3
  OCR2A = INT_OVF - 1;        // Sine Overflow
  TIMSK2 |= _BV(OCIE2A);      // Enable Compare 2A
  
  sei();
} 

void loop()
{
  E_COMMS_MODE comms_mode = MODE_NONE;
  char outMask[NUM_LOOPS];        // Solenoid output bit mask
  int target[NUM_LOOPS];          // Control target
  float target_ratio;             // Ratio of float 2 to float 1
  float drive[NUM_LOOPS];         // Control output
  float outFraction[NUM_LOOPS];   // Fractional drive remainder
  float outFracCarry[NUM_LOOPS];  // Fractional drive carry
  int value;                      // Raw pressure reading
  int valueMin[NUM_PSENSORS];     // Raw pressure reading min
  int valueMax[NUM_PSENSORS];     // Raw pressure reading max
  long valueSum[NUM_PSENSORS];    // Raw pressure readings summed
  long valueCount[NUM_PSENSORS];  // Raw pressure readings counted
  float error[NUM_LOOPS];         // Control error
  float last_error[NUM_LOOPS];    // Control error from last iteration
  float integral[NUM_LOOPS];      // Control integral accumulator  
  float pressure[NUM_LOOPS];      // Filtered pressure in ADC counts
  float pressure_cal[NUM_LOOPS];  // Filtered pressure in kPa
  bool control = 0;               // Control loop enabled
  bool pwm[NUM_LOOPS];            // PWM mode active
  bool debug_enabled = 1;         // Debug printing enabled
  int j;
  
  target_ratio = 0.0;
  
  for ( j=0; j<NUM_PSENSORS; j++ )
  {
    valueMin[j]         = 1024;
    valueMax[j]         = 0;
    valueSum[j]         = 0;
    valueCount[j]       = 0;
    pressure_offsets[j] = 0.0;
  }
  
  for ( j=0; j<NUM_LOOPS; j++ )
  {
    outMask[j]      = 0;
    target[j]       = 0;
    drive[j]        = 0.0;
    outFraction[j]  = 0.0;
    outFracCarry[j] = 0.0;
    last_error[j]   = 0.0;
    integral[j]     = 0.0;
    pressure[j]     = 0.0;
    pwm[j]          = 0;
  }
  
  setFreq( 1 );
  
  while ( 1 )
  {
    /* Process characters in comms buffer */
    while ( Serial.available() > 0 )
    {
      int val_int;
      float val_float;
      char mode_char;
      
      switch ( comms_mode )
      {
        case MODE_NONE:
          /* Decide which command we have received */
          mode_char = Serial.read(); 
          comms_mode = parseMode( mode_char );
          break;
        
        /* Read additional values, depending on command */
        case MODE_P:
        case MODE_I:
        case MODE_D:
        case MODE_RATIO:
          val_float = Serial.parseFloat();
          break;
        case MODE_MASK:
        case MODE_AMPL:
        case MODE_DEBUG:
        case MODE_ZERO:
          val_int = Serial.parseInt();
          break;
        default:
          break;
      }
      
      /* The command is finished when we get an end of line character */
      if ( Serial.peek() == '\r' )
      {
        Serial.read();
        
        /* Process command and associated parameters */
        switch ( comms_mode )
        {
          case MODE_MASK:
            outMask[0] = (byte)val_int;
            outMask[1] = (byte)val_int;
            control = 0;
            target[0] = 0.0;
            target[1] = 0.0;
            break;
            
          case MODE_AMPL:
            target[0] = (int)val_int;
            if ( control == 0 )
            {
              integral[0] = 0.0;
              integral[1] = 0.0;
            }
            control = 1;
            break;
            
          case MODE_P:
            p = val_float;
            save_PID( PID_INDEX_P, p );
            break;
            
          case MODE_I:
            i = val_float;
            save_PID( PID_INDEX_I, i );
            break;
            
          case MODE_D:
            d = val_float;
            save_PID( PID_INDEX_D, d );
            break;
            
          case MODE_DEBUG:
            debug_enabled = (byte)val_int;
            break;
            
          case MODE_ZERO:
            control = 0;
            zero_pressure( (byte)val_int );
            break;
            
          case MODE_RATIO:
            target_ratio = val_float;
            break;
            
          default:
            Serial.print( "Invalid" );
            break;
        }
        
        /* Calculate second target */
        target[1] = round( (float)val_int * target_ratio );
        
        Serial.print( "\r\n" );
        comms_mode = MODE_NONE;
      }
    }
    
    /* Read pressures */
    for ( j=0; j<NUM_PSENSORS; j++ )
    {
      value = analogRead( P_PINS[j] );
      if ( value < valueMin[j] )
        valueMin[j] = value;
      if ( value > valueMax[j] )
        valueMax[j] = value;
      valueSum[j] += value;
      valueCount[j]++;
    }
    
    /* Apply filter to smooth pressure reading */
//    pressure = ( FILTER_ALPHA * pressure + (float)value ) / 10.0 - 0.45; //**
    
    if ( go == 1 )
    {
      go = 0;
      
      /* Read pressure sensors and close loops */
      for ( j=0; j<NUM_PSENSORS; j++ )
      {
        pressure[j] = (float)valueSum[j] / valueCount[j];
  //      pressure = ( (float)( valueSum * 100 ) / valueCount ) / 100.0;
        
        /* Apply calibration */
        pressure_cal[j] = getPressure( pressure[j] ) - pressure_offsets[j];
        
        /* Control loop start.  Calculate error. */
        error[j] = (float)target[j] - pressure_cal[j];
        
        /* If we have set a control target, translate control output to
           solenoid output mask.
        */
        if ( control )
        {
          /* Calculate integral and apply windup limiting. */
          integral[j] += error[j] * i;
          if ( integral[j] < -(float)WINDUP_MAX )
            integral[j] = -(float)WINDUP_MAX;
          else if ( integral[j] > (float)WINDUP_MAX )
            integral[j] = (float)WINDUP_MAX;
          
          /* Calculate output, consisting of P, I and D factors. */
          drive[j] = error[j] * p + integral[j] +
                     ( error[j] - last_error[j] ) * d;
          
          /* Limit output to be within range allowed by number of solenoids */
          if ( drive[j] < 0.0 )
            drive[j] = 0.0;
          else if ( drive[j] > (float)MASK_MAX )
            drive[j] = (float)MASK_MAX;
          
          outMask[j] = floor(drive[j]);
          outFraction[j] = outFracCarry[j] + drive[j] - outMask[j];
          
          /* Calculate PWM for fraction */
          if ( outFraction[j] >= 1.0 || outFraction[j] <= 0.0 )
  //        if ( 0 )
          {
            /* If large change */
  //          Serial.print( "Large " );
            
            /* Round to nearest whole bit ------ */
            outMask[j] = round(drive[j]);
            pwm[j] = 0;
            /* or use 50% cycle ---------------- */
  //          outFraction = 0.5;
  //          pwm = 1;
            /* --------------------------------- */
            
            outFracCarry[j] = 0.0;
          }
          else
          {
            /* If small change */
  //          Serial.print( "Small " );
            pwm[j] = 1;
            
            if ( outFraction[j] > 0.9 )
            {
              outFracCarry[j] += outFraction[j] - 1.0;
              outFraction[j] = 1.0;
            }
            else if ( outFraction[j] < 0.1 )
            {
              outFracCarry[j] += outFraction[j];
              outFraction[j] = 0.0;
            }
            else
              outFracCarry[j] = 0.0;
          }
        }
        
        /* Record last value for use in next iteration */
        last_error[j] = error[j];
      }
      
      /* Print control data */
      if ( debug_enabled )
      {
        for ( j=0; j<NUM_LOOPS; j++ )
        {
          Serial.print( (int)j + 1 );
          Serial.print( " : " );
          Serial.print( "(" );
          Serial.print( (int)target[j] );
          Serial.print( " / " );
          Serial.print( (float)pressure_cal[j], PRES_DECIMALS );
          Serial.print( " / " );
          Serial.print( (float)getPressure(valueMin[j]) );
          Serial.print( "-" );
          Serial.print( (float)getPressure(valueMax[j]) );
          Serial.print( ":" );
          Serial.print( (long)valueCount[j] );
  //        Serial.print( " = " );
  //        Serial.print( (float)getPressure((float)valueSum / valueCount) );
          Serial.print( ") / (" );
  //        Serial.print( (int)value );
  //        Serial.print( " / " );
          Serial.print( (int)outMask[j] );
          Serial.print( " / " );
          Serial.print( (float)pressure[j], PRES_DECIMALS );
          Serial.print( ") / (" );
          Serial.print( (float)error[j], PRES_DECIMALS );
          Serial.print( " / " );
          Serial.print( (float)drive[j], 2 );
  //        Serial.print( " / " );
  //        Serial.print( (float)outFraction );
  //        Serial.print( " / " );
  //        Serial.print( (float)outFracCarry );
          Serial.print( ") / (" );
          Serial.print( (float)p, PID_DECIMALS );
          Serial.print( " / " );
          Serial.print( (float)i, PID_DECIMALS );
          Serial.print( " / " );
          Serial.print( (float)d, PID_DECIMALS );
          Serial.print( " / " );
          Serial.print( (float)integral[j] );
          Serial.print( ")" );
          Serial.print( "\r\n" );
        }
      }
      
      /* Clear averaging data */
      for ( j=0; j<NUM_LOOPS; j++ )
      {
        valueMin[j] = 1024;
        valueMax[j] = 0;
        valueSum[j] = 0;
        valueCount[j] = 0;
      }
      
//        Serial.print( "Timer " );
//        Serial.print( (int)timer );
//        Serial.print( "Buf " );
//        Serial.print( (int)Serial.availableForWrite() );
//        Serial.println( "" );
        
      if ( go == 1 )
        Serial.print( "Overflow\r\n" );
      
      /* Record fractional control time start */
      timerOffset = timer;
    }
    
    for ( j=0; j<NUM_LOOPS; j++ )
    {
      if ( control )
      {
        if ( pwm[j] )
        {
          bool on = ( timer - timerOffset ) < outFraction[j] * overflow;
          outMask[j] = ( outMask[j] & ~1 ) | ( on ? 1 : 0 );
  //        outMask |= ( on ? 1 : 0 );
        }
      }
      
      /* Set solenoid outputs */
      digitalWrite( SOL_PINS[j], ( outMask[j] & 1 ) ? HIGH : LOW );
    }
  }
} 

