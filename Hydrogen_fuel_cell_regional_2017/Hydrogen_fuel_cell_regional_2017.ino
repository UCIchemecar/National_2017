#include "DualMC33926MotorShield.h"
#include <PololuWheelEncoders.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
DualMC33926MotorShield md;
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2591
*/
/**************************************************************************/
void configureSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  //tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  tsl.setGain(TSL2591_GAIN_HIGH);
  //tsl.setGain(TSL2591_GAIN_MAX); // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         ");
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println("1x (Low)");
      break;
    case TSL2591_GAIN_MED:
      Serial.println("25x (Medium)");
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println("428x (High)");
      break;
    case TSL2591_GAIN_MAX:
      Serial.println("9876x (Max)");
      break;
  }
  Serial.print  ("Timing:       ");
  Serial.print((tsl.getTiming() + 1) * 100, DEC); 
  Serial.println(" ms");
  Serial.println("------------------------------------");
  Serial.println("");
}

/**************************************************************************/
/*
    Program entry point for the Arduino sketch
*/
/**************************************************************************/
void setup(void) 
{
Serial.begin(115200);
  
  Serial.println("Starting Adafruit TSL2591 Test!");
  
  if (tsl.begin()) 
  {
    Serial.println("Found a TSL2591 sensor");
  } 
  else 
  {
    Serial.println("No sensor found ... check your wiring?");
    while (1);
  }
    
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Configure the sensor */
  configureSensor();
    
  // Now we're ready to get readings ... move on to loop()!

  Serial.println("Dual MC33926 Motor Shield");
  md.init();
  PololuWheelEncoders::init(3,5,6,11);
}
void simpleRead(void)
{
  // Simple data read example. Just read the infrared, fullspecrtrum diode 
  // or 'visible' (difference between the two) channels.
  // This can take 100-600 milliseconds! Uncomment whichever of the following you want to read
  //uint16_t x = tsl.getLuminosity(TSL2591_VISIBLE);
  uint16_t x = tsl.getLuminosity(TSL2591_FULLSPECTRUM);
  //uint16_t x = tsl.getLuminosity(TSL2591_INFRARED);

  Serial.print("[ "); Serial.print(millis()/1000); Serial.print(" s ] ");
  Serial.print("Luminosity: ");
  Serial.println(x, DEC);
}

/**************************************************************************/
/*
    Show how to read IR and Full Spectrum at once and convert to lux
*/
/**************************************************************************/
void advancedRead(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  Serial.print("[ "); Serial.print(millis()); Serial.print(" ms ] ");
  Serial.print("IR: "); Serial.print(ir);  Serial.print("  ");
  Serial.print("Full: "); Serial.print(full); Serial.print("  ");
  Serial.print("Visible: "); Serial.print(full - ir); Serial.print("  ");
  Serial.print("Lux: "); Serial.println(tsl.calculateLux(full, ir));
}

/**************************************************************************/
/*
    Performs a read using the Adafruit Unified Sensor API.
*/
/**************************************************************************/
void unifiedSensorAPIRead(void)
{
  /* Get a new sensor event */ 
  sensors_event_t event;
  tsl.getEvent(&event);
 
  /* Display the results (light is measured in lux) */
  Serial.print("[ "); Serial.print(event.timestamp); Serial.print(" ms ] ");
  if ((event.light == 0) |
      (event.light > 4294966000.0) | 
      (event.light <-4294966000.0))
  {
    /* If event.light = 0 lux the sensor is probably saturated */
    /* and no reliable data could be generated! */
    /* if event.light is +/- 4294967040 there was a float over/underflow */
    Serial.println("Invalid data (adjust gain or timing)");
  }
  else
  {
    Serial.print(event.light); Serial.println(" lux");
  }
}

void loop()
{
 /*********Stopping code************/
 static float total2;
  static float total1;
  static int f2=0;// flag for marking the car starts running physically 
  static int m1=390;
  static int count6=0;//count the amount of times the sensor has detected 63534
  static int countd=0;//count the amount of times the solution has gone dark
  static int m2=375;
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  static float distance=0;// distance the car will travel at the given speed in meters
  static float sp=0.55; // The speed of the car in m/s
  static unsigned long t0=0; //the time that liquid was stabilized
  static unsigned long t1=0; //the time that the liquid went dark
  static unsigned long t00=0;
  static unsigned long time2=0;
  static unsigned long time1=0; //time it takes for the liquid to go from injected to dark
  static unsigned long totalRuntime=0;
  static unsigned long tRun1=0;//start of run time
  static unsigned long tRun2=0;//end of run time
  static int f1=0; // flag to indicate phase, phase 0 is the default where nothing has happened. 
  ir = lum >> 16;
  full = lum & 0xFFFF;
  unsigned int a=tsl.calculateLux(full, ir);
  
  if(a<40000 && millis()>=200 && f1==0)
  {
    f1=1;//liquid has been injected. 
    t00=millis();
  }
  if(f1==1 && a>40000)
  {
    if(count6==0)
    {
      t0=millis();
    }
    if(count6>5)
    {
       f1=2;//liquid has been stabilized
    }
     ++count6;
  }
  if(f1==1 && a<40000)
  {
    t0=0;//reset variables if the stabilization was not continuous
    count6=0;
  }
  if(f1==2 && a<40000)
  {
    //liquid has turned dark
    if(countd==0)
    {
    t1=millis();
    }
    time1=t1-t0;//time since stabilization
    time2=t1-t00;//time since injection
    if (time1>10000 && countd>5)
    {
      f1=3;
      totalRuntime=time1;
    }
    ++countd;
  }
  
  Serial.print(millis()); Serial.print("          "); 
  Serial.print(a); Serial.print("             "); 
  Serial.print(t0);  Serial.print("                                 "); 
  Serial.print(t1); Serial.print("                              "); 
  Serial.print(time1);  Serial.print("    ms        ");
  //Serial.print(time2); 
  distance=sp*time1/1000.0;
  Serial.print(distance); Serial.println("    m   ");
  
  /***********Motor code*************/
  /*
  if(f1==3)
  {
    md.setM1Speed(0);
    md.setM2Speed(0);
  }
  else
  {
    md.setM1Speed(380);
    md.setM2Speed(400);
  }
  */
  /**********Alternate motor code that used motor run time*************/
  
  
  int i=PololuWheelEncoders::getCountsAndResetM1();
  total1=total1+abs(i/3591.84);
  if(total1==0 && tRun1==0 && totalRuntime==0)
  {
    md.setM1Speed(m1);
    md.setM2Speed(m2);
    Serial.println("It has not run");
  }
  if(total1>0.1 && f2==0)
  {
    f2=1;
    tRun1=millis();
    md.setM1Speed(m1);
    md.setM2Speed(m2);
    Serial.println("It started running");
  }
  if (tRun1>0 && totalRuntime==0)
  {
    md.setM1Speed(m1);
    md.setM2Speed(m2);
    Serial.println("Stopping Reaction has not finished");
  }
  if (tRun1>0 && totalRuntime>0)
  {
    if((millis()-tRun1)>totalRuntime)
    {
    md.setM1Speed(0);
    md.setM2Speed(0);
    Serial.println("Stopping Reaction has finished run time has reached");
    }
    else if(millis()-tRun1<totalRuntime)
    {
    md.setM1Speed(m1);
    md.setM2Speed(m2);
    Serial.println("Stopping Reaction has finished but run time has not reached");
    }
    else
    {
      Serial.println("Outlier");
      md.setM1Speed(0);
    md.setM2Speed(0);
    }
  }
  Serial.print(total1); Serial.print("    rotation    ");
  Serial.println(tRun1);
}
