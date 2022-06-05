//Tutorial07 CPG Control Solution

//Calibration of Servo Limits (TGY 50090M Servo)
//Please enter our specific servo limits for servo 1,2, and 3 here
//You can find the corresponding limits in your EDMO box
int SERVOMIN[]  {108, 108, 108}; // this is the 'minimum' pulse length count (out of 4096)
int SERVOMAX[]  {504, 498, 474};// this is the 'maximum' pulse length count (out of 4096)
int poti_low[]  {141, 141, 150};
int poti_high[] {744, 744, 740};


#include <Servo.h> 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

unsigned long previousMillis = 0;
double frequency = 0.5; // oscillator frequency
double rateOfFrequency = 0;
double targetFrequency = 0.5;
unsigned long timeStep = 25; // period used to update CPG state variables and servo motor control

int pose[] = {0, 0, 0}; //temp variable for storing the servo's poti value
int poti_pins[] = {14, 15, 16}; //analog pins to which the potis' of the servos are connected (A0=14, A1=15, A2=16, ...)


double w = 0.025; // we assume that all oscillators use same coupling weight
double a = 1; // we assume that all oscillators use same adaptation rate for amplitude
double c = 0.5; // adaptation rate for frequency and offset

const unsigned int NUM_OSCILLATORS = 3; // this number has to match entries in array osc[]
int LEFTEND = -90;
int RIGHTEND = 90;
Servo myservo[NUM_OSCILLATORS];  // create servo object to control a servo 
float calib[NUM_OSCILLATORS];

typedef struct 
{
    double phase;                       // phase of the oscillation
    double amplitude;                   // amplitude of the oscillation 
    double targetAmplitude;             // amplitude to gradually change to
    double offset;                      // offset for the oscillation (in range of servo 0-180)
    double targetOffset;                // added parameter to offset smoothly
    double rateOfPhase;                 // current rate of change of the phase parameter
    double rateOfAmplitude;             // current rate of change of the amplitude parameter
    double rateOfOffset;                // current rate of change of the offset parameter
    double pos;                         // oscillator output = servos angular position
    double phaseBias[NUM_OSCILLATORS];  // controls pairwise coupling phase bias
    double coupling[NUM_OSCILLATORS];   // controls topology of the network
    uint16_t angle_motor;               // mapped motor value
} oscillator;

// initalisation with offset 90 for all motors (since servos operate in the range 0-180)
oscillator osc[NUM_OSCILLATORS] = 
{
    {0,0,0,90,90,0,0,0,0,{0,0,0},{0,1,0}},
    {0,0,0,90,90,0,0,0,0,{0,0,0},{1,0,1}},
    {0,0,0,90,90,0,0,0,0,{0,0,0},{0,1,0}}
};

// strings for reading input
String commandString, valueString, indexString;

// standard setup method
void setup() 
{
    // initialising serial communication
    Serial.begin(9600);
    
    pwm.begin();
    pwm.setOscillatorFrequency(27000000); //27MHz
    pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz update

    delay(10);
    zeroCalib();
    setCalib(0,-3);
    setCalib(1,-4);
    setCalib(2,-10);
}


void loop() 
{    
    unsigned long currentMillis = millis();
    
    if (currentMillis - previousMillis >= timeStep) 
    {
        // save the last time we calculated CPG values
        previousMillis = currentMillis;

        // check whether parameters have been changed
        readInput();
        poti_writer();

        // for frequency, offset and amplitude, the following 2 steps are applied:
        // 1. compute rate of change
        // 2. use Euler integration to adjust parameter

        // change frequency
        rateOfFrequency = c * (targetFrequency - frequency);
        frequency = frequency + rateOfFrequency * ((float) timeStep) / 1000.0;

        for (int i = 0; i < NUM_OSCILLATORS; i++) 
        {
            // change offset for oscillator i
            osc[i].rateOfOffset = c * (osc[i].targetOffset - osc[i].offset);
            osc[i].offset = osc[i].offset + osc[i].rateOfOffset * ((float) timeStep) / 1000.0;
            
            // change amplitude for oscillator i
            osc[i].rateOfAmplitude = a * (osc[i].targetAmplitude - osc[i].amplitude);
            osc[i].amplitude = osc[i].amplitude + osc[i].rateOfAmplitude * ((float) timeStep) / 1000.0;
            
            // compute new rate of phase
            double sum = 2 * PI * frequency;
            for (int j = 0; j < NUM_OSCILLATORS; j++) 
            {
                if (i != j) 
                {
                    // NOTE: here w is used as the coupling factor for all oscillators
                    //sum = sum + w * osc[i].amplitude * sin(osc[j].phase - osc[i].phase - osc[i].phaseBias[j]);
                    // line below actually accounts for the topology of the network
                    if (osc[i].coupling[j] != 0)
                    {
                    sum = sum + w  * osc[i].amplitude * sin(osc[j].phase - osc[i].phase - osc[i].phaseBias[j]);
                    }
                }
            }
            osc[i].rateOfPhase = sum;

            // compute new phase using Euler integration
            osc[i].phase = osc[i].phase + osc[i].rateOfPhase * ((float) timeStep) / 1000.0;
        
            // compute new angular position
            osc[i].pos = osc[i].amplitude * sin(osc[i].phase) + osc[i].offset;

            // set motor to new position
            //myservo[i].write(osc[i].pos);
            osc[i].pos+= calib[i];
            osc[i].angle_motor = map(osc[i].pos,0,180,SERVOMIN[i],SERVOMAX[i]);
            osc[i].angle_motor = constrain(osc[i].angle_motor,SERVOMIN[i],SERVOMAX[i]);
           
            pwm.setPWM(i, 0, osc[i].angle_motor);
            // print out position for plotting
            //Serial.print(osc[i].pos); //diabled for experience day
            //Serial.print(" "); //disabled for experience day
        }
       // Serial.println(); //disabled for experience day
    }

}

// method for receiving commands over the serial port
void readInput() 
{
    if (Serial.available()) 
    {
        commandString = Serial.readStringUntil('\n');
        if (commandString.startsWith("amp")) 
        {
            // change the target amplitude for the specified oscillator
            indexString = commandString.substring(4, 5);
            valueString = commandString.substring(6, commandString.length());
            osc[(int) indexString.toInt()].targetAmplitude = (int) valueString.toInt();
        } else if (commandString.startsWith("off")) 
        {
            // change the target offset for the specified oscillator
            indexString = commandString.substring(4, 5);
            valueString = commandString.substring(6, commandString.length());
            osc[(int) indexString.toInt()].targetOffset = (int) valueString.toInt();
        } else if (commandString.startsWith("freq")) 
        {
            // change the target frequency for all oscillators
            valueString = commandString.substring(5, commandString.length());
            targetFrequency = (float) valueString.toFloat();
        } else if (commandString.startsWith("phb")) 
        {
            // change the phase bias between the two specified oscillators
            indexString = commandString.substring(4, 5);
            int index1 = (int) indexString.toInt();
            indexString = commandString.substring(6, 7);
            int index2 = (int) indexString.toInt();
            valueString = commandString.substring(8, commandString.length());
            osc[index1].phaseBias[index2] =  ((PI * valueString.toFloat())/180);//deg2rad!!! (float)
            osc[index2].phaseBias[index1] = -osc[index1].phaseBias[index2];
        } else if (commandString.startsWith("weight")) 
        {
            // change the weight for the adaptation of the rate of change of the phase
            valueString = commandString.substring(7, commandString.length());
            w = (float) valueString.toFloat();
        } else if (commandString.startsWith("print")) 
        {
            // print information about the current state of the oscillators
            Serial.print("Frequency: ");
            Serial.println(frequency);
            for (int i = 0; i < NUM_OSCILLATORS; i++) 
            {
                Serial.print(i);
                Serial.print(": ");
                Serial.print("[");
                Serial.print(osc[i].phase);
                Serial.print(", ");
                Serial.print(osc[i].amplitude);
                Serial.print(", ");
                Serial.print(osc[i].targetAmplitude);
                Serial.print(", ");
                Serial.print(osc[i].offset);
                Serial.print(", ");
                Serial.print(osc[i].targetOffset);
                Serial.print(", ");
                Serial.print(osc[i].rateOfPhase);
                Serial.print(", ");
                Serial.print(osc[i].rateOfAmplitude);
                Serial.print(", ");
                Serial.print(osc[i].rateOfOffset);
                Serial.print(", ");
                Serial.print(osc[i].pos);
                Serial.print(", [");
                Serial.print(osc[i].phaseBias[0]);
                Serial.print(", ");
                Serial.print(osc[i].phaseBias[1]);
                Serial.print(", ");
                Serial.print(osc[i].phaseBias[2]);
                Serial.print("]]");
                Serial.println();
            }
        }
    }
}

void poti_writer()
{
  for (int i =0; i< NUM_OSCILLATORS; i++)
  {
    pose[i] = analogRead(poti_pins[i]);
    pose[i] = map(pose[i], poti_low[i], poti_high[i], LEFTEND, RIGHTEND);
    //Serial.print(angle);
    //Serial.print("\t");
    Serial.print(pose[i]);
    Serial.print(" ");
    Serial.print(osc[i].pos);
    if (i < NUM_OSCILLATORS - 1)
    {
      Serial.print(", ");
    }
  }
  Serial.println();
}

void zeroCalib()
{
    for (byte j = 0 ; j < NUM_OSCILLATORS ; j++)
      calib[j] = 0;
}

void setCalib(int motor,int val)
{
    if(motor < NUM_OSCILLATORS)
        calib[motor] = val;
    else
       Serial.println("Enter a valid motor number"); 
}