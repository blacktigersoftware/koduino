#include <LIS3DSH.h>
#include <SPI.h>

/*
  Tilt

  Uses the built-in LIS3DSH accelerometer on the STM32F4-Discovery board to create a tilt-level sensor.
  This was tested with the koduino Arduion board support
 
 */
 

#define LED_GREEN   PD12
#define LED_ORANGE  PD13
#define LED_RED     PD14
#define LED_BLUE    PD15

// 0 if the driver started fine
bool lis3dsh_start_failed;

// tilt thresholds
int16_t ThresholdHigh = 10;
int16_t ThresholdLow = -10;

// the setup routine runs once when you press reset:
void setup() 
{                
    // initialize the LED pins as an output.
    pinMode(PD12, OUTPUT);
    pinMode(PD13, OUTPUT);
    pinMode(PD14, OUTPUT);
    pinMode(PD15, OUTPUT);

    // need to setup SPI bus
    SPI.setDataSize(8);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV16);
    SPI.begin();

    delay(1000);

    // start the LIS3DSH
    lis3dsh_start_failed = Lis3dshDrv.begin() != 0;
}


// the loop routine runs over and over again forever:
void loop() 
{
    int16_t buffer[3] = {0};
    int16_t xval, yval = 0x00;

    if (lis3dsh_start_failed)
    {
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_BLUE, HIGH);
        digitalWrite(LED_ORANGE, HIGH);
        return;
    }
  
    Lis3dshDrv.ReadACC( buffer );
  
    xval = buffer[0];
    yval = buffer[1];

    if ( abs(xval) > abs(yval) )
    {
        if (xval > ThresholdHigh)
            digitalWrite(LED_RED, HIGH);
        else if (xval < ThresholdLow)
            digitalWrite(LED_GREEN, HIGH);
    }
    else 
    {
        if (yval < ThresholdLow)
            digitalWrite(LED_BLUE, HIGH);
        else if (yval > ThresholdHigh)
            digitalWrite(LED_ORANGE, HIGH);
    }

    delay(10);

    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_ORANGE, LOW);
  
}
