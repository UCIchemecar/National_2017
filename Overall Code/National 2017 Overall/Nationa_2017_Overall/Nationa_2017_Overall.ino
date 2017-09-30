/*Functions taken from Adafruit and Pololu's examples and were modified with reaction code for complete car control code*/



#include "DualMC33926MotorShield.h"//Pololu shield library, available on ChemE car github account
#include <PololuWheelEncoders.h>//Pololu encoder for the motors, available on ChemE car github account
#include <Wire.h>
#include <Adafruit_Sensor.h>//Adafruit unified sensor library
#include "Adafruit_TSL2591.h"//TSL2591 library

#define Motor_Tick_per_rotation 3591.84
#define motor1_speed 393//adjust for speed of motor 1, out of 400
#define motor2_speed 372//adjust for speed of motor 2, out of 400
#define car_speed 0.54//speed of the car in m/s

DualMC33926MotorShield md;
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
void stopIfFault()
{
        if (md.getFault())
        {
                Serial.println("fault");
                while(1) ;
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
        tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS); // shortest integration time (bright light)
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
                while (1) ;
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

        //Sensor Variables
        uint32_t lum                                    = tsl.getFullLuminosity();
        uint16_t ir, full;

        //Motor Variables
        static float distance                           =0;// distance the car will travel at the given speed in meters
        static boolean car_moved                        =false; // flag to indicate phase, 1 for liquid injected, 2 for stabilized, 3 for finished reaction.
        static float total1; //total amount of rotation for motor 1
        static unsigned long totalRuntime               =0; //amount of time the car has been running
        static unsigned long car_start_time             =0; //time when the wheel first moves

//Reaction variables
        static int count6                               =0; //count the amount of times the sensor has detected the light from the white LED after the reaction has stabilized
        static int countd                               =0; //count the amount of times the solution has gone dark
        static unsigned long t_stabilized               =0; //the time that liquid was stabilized
        static unsigned long t_dark                     =0; //the time that the liquid went dark
        static unsigned long t_injected                 =0; //mark when liquid was injected
        static unsigned long dur_injected               =0; //time since injection in milliseconds
        static unsigned long dur_injected_to_stabilized =0; //time it takes for the liquid to go from injected to stabilized in milliseconds
        static int flag_reaction                        =0; //flags for indicating the phase of the reaction

        /*Light Sensor measurement*/
        ir = lum >> 16;
        full = lum & 0xFFFF;
        unsigned int light_level=tsl.calculateLux(full, ir);

        /*********************Reaction Event Detection Code************************/

        //Phase 1: Liquid was not injected
        if(light_level<40000 && millis()>=200 && flag_reaction==0)
        {
                flag_reaction=1;//liquid has been injected.
                t_injected=millis();
        }

        //Phase 2: Injection was dectected, the code will now search for stabilization
        if(flag_reaction==1 && light_level>40000)
        {
                if(count6==0)
                {
                        t_stabilized=millis();//mark the beginning of stabilization, will be reset if unstable
                }
                if(count6>5)
                {
                        flag_reaction=2;//liquid has been stabilized
                }
                ++count6;
        }
        if(flag_reaction==1 && light_level<40000)
        {
                t_stabilized=0;//reset variables if the stabilization was not continuous
                count6=0;
        }

        //Phase 3: Stabilization reached, searching for the darkening of the solution
        if(flag_reaction==2 && light_level<40000)
        {
                //liquid has turned dark
                if(countd==0)
                {
                        t_dark=millis();
                }
                dur_injected_to_stabilized=t_dark-t_stabilized;//time since stabilization
                dur_injected=t_dark-t_injected;//time since injection
                if (dur_injected_to_stabilized>10000 && countd>5)
                {
                        flag_reaction=3;
                        totalRuntime=dur_injected_to_stabilized;
                }
                ++countd;
        }

        /*Diagnostic and Prediction Code*/
        Serial.print(millis()); Serial.print("          ");
        Serial.print(light_level); Serial.print("             ");
        Serial.print(t_stabilized);  Serial.print("                                 ");
        Serial.print(t_dark); Serial.print("                              ");
        Serial.print(dur_injected_to_stabilized);  Serial.print("    ms        ");

        distance=(car_speed*dur_injected_to_stabilized/1000.0)-0.322;//Formula for calculating run distance in meters. This is only a prediction

        Serial.print(distance); Serial.println("    m   ");


        /************************Motor Control Code***********************/

        //Encoder Measurement
        int i=PololuWheelEncoders::getCountsAndResetM1();
        total1=total1+abs(i/Motor_Tick_per_rotation);//Amount of rotation on Motor 1, used only for sensing whether the car moved.

        //Determining the phase of movement

        if(!total1 && !car_start_time && !totalRuntime)
        {
                md.setM1Speed(motor1_speed);
                md.setM2Speed(motor2_speed);
                Serial.println("It has not run");
        }

        if(total1>0.1 && !car_moved)
        {
                car_moved=!car_moved;
                car_start_time=millis();
                md.setM1Speed(motor1_speed);
                md.setM2Speed(motor2_speed);
                Serial.println("It started running");
        }

        if (car_start_time>0 && !totalRuntime)
        {
                md.setM1Speed(motor1_speed);
                md.setM2Speed(motor2_speed);
                Serial.println("Stopping Reaction has not finished");
        }

        if (car_start_time>0 && totalRuntime>0)
        {
                if((millis()-car_start_time)>totalRuntime)
                {
                        md.setM1Speed(0);
                        md.setM2Speed(0);
                        Serial.println("Stopping Reaction has finished run time has reached");
                }
                else if((millis()-car_start_time)<totalRuntime)
                {
                        md.setM1Speed(motor1_speed);
                        md.setM2Speed(motor2_speed);
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
        Serial.println(car_start_time);
}
