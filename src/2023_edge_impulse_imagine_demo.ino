/*
 * Project Partcile Imagine 2023 Data Gathering
 * Description:
 * Author:
 * Date:
 */

#include "Particle.h"
#include "UIDisplay.h"
#include <Debounce.h>
#include <Wire.h>

//SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler(LOG_LEVEL_INFO);

// OLED FeatherWing buttons map
#define BUTTON_A D4
#define BUTTON_B D3
#define BUTTON_C D2

#define ADC_COUNTS (1 << 12) // ADC bits

//Various gain settings when collecting data individually
#define GAIN_LAMPDEMO   254
#define GAIN_LAMPINC    212
#define GAIN_LAMP33W    212
#define GAIN_DESKFAN    254
#define GAIN_FRIDGE     251
#define GAIN_BLENDER    212
#define GAIN_TOASTER    212
#define GAIN_DEFAULT    212
#define GAIN_ALL        245

#define NUM_SAMPLES     80  //number of samples per event
#define SAMPLE_PERIOD   50   //Sampling interval for each event in milliseconds
#define GAIN_VALUE      GAIN_ALL     //212 is default gain for 15A full scale

int payloadBuffer[2][NUM_SAMPLES];
int sampleIndex = 0;
char sampleBuffer[16];
char buf[1024];
String payloadString;
String sampleString;
bool sendData = false;
bool firstSample = true;

double filteredCurrent = 0;
double offsetCurrent = 0;
float sumCurrent = 0;
double squareCurrent = 0;
float rmsCurrent = 0;
float instPower = 0;
double CURRENT_CAL = 0.027566;

UIDisplay ui;

int redLED = D3; // blinker on custom analog front end PCB
int voltagePIN = A2; // voltage is on A2
int currentPIN = A5; // current is pn A5

int32_t voltageADC = 0; //init voltage value to zero
int32_t currentADC = 0; //init current value to zero

// 1 ms timer for sample frequency 1 kHz
Timer sample(1, sampleData);

//Debounce feathering buttons
Debounce debouncer = Debounce(); 

void setGain(void)
{
    char gainRet = 0;
    Wire.beginTransmission(0x2f);
    Wire.write(0x00);
    Wire.write(GAIN_VALUE);
    Wire.endTransmission();
    delay(5);
    Wire.requestFrom(0x2f,1);
    gainRet = Wire.read();
    Wire.endTransmission(); 
    if(gainRet == GAIN_VALUE)
    {
        Log.info("SUCCESS SETTING GAIN: %d", gainRet);
    }
    else
    {
        Log.info("FAILED TO SET GAIN: %d", gainRet);
    }  
}

void sampleData()
{
    voltageADC = analogRead(voltagePIN); //read ADC values
    currentADC = analogRead(currentPIN);
    payloadBuffer[0][sampleIndex] = voltageADC;
    payloadBuffer[1][sampleIndex] = currentADC;
    sampleIndex++;

    filteredCurrent = currentADC - 2048;
    squareCurrent = filteredCurrent * filteredCurrent;
    sumCurrent += squareCurrent;

    if(sampleIndex == NUM_SAMPLES)  //we are done sampling for this event
    {
        sendData = true;
        sampleIndex = 0;
        sample.stop();  
        rmsCurrent = sqrt(sumCurrent / NUM_SAMPLES);
        rmsCurrent -= 8;
        if(rmsCurrent < 0)
        {
            rmsCurrent = 0;
        }
        rmsCurrent *= CURRENT_CAL;
        sumCurrent = 0;
    }
}

void setup() {
    pinMode(redLED, OUTPUT);
    Serial.begin(9600);
    Log.info("OLED FeatherWing test");
    debouncer.attach(BUTTON_C, INPUT_PULLUP);
    debouncer.interval(10); 
    Particle.connect();
    //ui.begin();
    delay(5);
    setGain();
    sample.start();
}

bool tx_data(String data_in) {
    bool retval = false;
    retval = Particle.publish("edge/ingest/blender", data_in, PRIVATE);  //manually set appliance label here
    return retval;
}

void loop() {
 
    static unsigned long now = millis();
    static bool state = false;
    static bool gatherData = true;
    int buttonState = 1;

    digitalWrite(redLED, LOW);
    debouncer.update();
    buttonState = debouncer.read();
    if(buttonState == 0)
    {
        gatherData = !gatherData;
    }

    if(Particle.connected())    //don't do anything if the cloud hasn't connected yet
    {
        if(firstSample)
        {
            ui.cloudIsConnected();  //indicate cloud connection on LCD
            now = millis();
            while(millis() - now < 3000); //wait until 3s have passed after first cloud connect
            firstSample = false;
        }

        if(gatherData)
        {
            if (millis() - now > SAMPLE_PERIOD) {
                sample.start();
                now = millis();
            }

            if(sendData) {
                digitalWrite(redLED, HIGH);
                memset(buf, 0, sizeof(buf));
                JSONBufferWriter writer(buf, sizeof(buf) - 1);

                for(int i=0; i<NUM_SAMPLES; i++)
                {
                    writer.beginArray();
                    writer.value(payloadBuffer[0][i]);
                    writer.value(payloadBuffer[1][i]);
                    writer.endArray();
                }

                Log.info(buf);
                bool result = tx_data(buf);
                Log.info("result: %d" ,result);
            
                sendData = false;
            }
        }
    }// end if connected
}
