/*
  TesLorean Cabin PTC Control Module
  Aug 2020
  Jeff Cooke

  Coding Todos
  - During Thermister loop checks, if temps are out of range, throttle back the duty cycle (may need a dynamic MAX)
  - DONE : The fault line for a given IGBT will set occassionally.  The reset line clears the fault and allows the IGBT to operate again
  - DONE : Recombine the PWMPin object into the IGBT object (each IGBT will have its own PWMpin)
  - DONE : Dropped the PWM library for IGBT control signals
  
  Coding Notes
  - IGBT target temp is 365K (93C), each IGBT dissipates heat of 5600W, max 448K (176C)
  - R = 10K / (1023/ADC - 1)
  - Commonly-used frequencies fro IGBT switching are in the vicinity of 30 kHz
  - Original Tesla microcontroller was STM8AF 16Mhz processor, 6 control lines, 6 reset lines, and 6 fault lines from the controller to the IGBT Driver chip
  - If the IGBT faults frequently and needs reset, the controller may keep track of the number of resets and disable (stop using the IGBT eventually)
  - The Fault line is actually NotFault, i.e. HIGH on no fault, LOW on fault
  - To clear the Fault, the Reset line is pulled LOW and Control line must also be LOW (Once the failure cause has been removed the micro controller must set the control inputs into an "Outputlow" state before applying the Reset pulse.)
  - Reset pulse width should exceed 0.1 micro-seconds
  - PWM duty cycle for IGBT control should be 10-20kHz
  - 5v PWM signal reduced to 4.5V via a voltage divider using 1kOhm and 10kOhm resistors
  - 5v Reset signal reduced to 2.5V via a voltage divider using two 4.7kOhm resistors
  - Fault signal from the IGBT driver goes through a 10kOhm resistor
  - Thermister 5v signal reduced to 1.136v using a voltage divider using 34kOhm and 10kOhm
  - If the air box is configured/modified, the IGBTs can be activated to differentially heat the driver and passenger

*/
#define DEBUG 1

// Libraries
#include <mcp_can.h>
#include <SPI.h>
#include <avr/wdt.h>    // Watchdog - autoresets
//#include <PWM.h>        // for high frequency PWM
#include "config.h"

// Lookup array of voltages to temperatures for thermister, tempsC 100,90,80,70,60,50,40,30,20,10,0,-10,-20
uint16_t tArray[15]={0,94,123,160,209,272,348,436,532,628,718,831,890,962,1023};
uint8_t cArray[15]={0,20,40,60,80,100,120,140,160,180,200,220,240,250,254};   // C in (0.5,-40) format

struct can_frame {
    uint32_t  can_id;
    uint8_t   can_dlc;
    uint8_t   data[8];
};
struct can_frame frame;

// Enums
enum thermtype {TYPE_IGBT, TYPE_PTC};
enum thermstatus {TNORMAL, TWARNING, TFAULT};
enum igbtstatus {INORMAL, IFAULT};

// Time Variables
unsigned long timesinceHeartbeat = 0;    // time of last Heatbeat message

// Predefined frames
struct can_frame PTrollcall;    // Rollcall response
struct can_frame PTheartbeat;   // Heartbeat message ~1Hz
struct can_frame PTstatus;      // Status of the PTC cabin heater
struct can_frame PTigbtfault;   // IGBT has triggered fault line

// Timers
unsigned long timeLastHeartbeat;
unsigned long timeSinceStartup;
boolean inStartupPeriod;

// Populate the CAN frame with the data specified
// When less that 8 bytes in frame, just fill out remainder with whatever...
void buildCANframe(can_frame & frame, uint16_t cid, uint8_t bcnt, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3,  uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7)
{
  frame.can_id = cid;
  frame.can_dlc = bcnt;
  frame.data[0] = b0;
  frame.data[1] = b1;
  frame.data[2] = b2;
  frame.data[3] = b3;
  frame.data[4] = b4;
  frame.data[5] = b5;
  frame.data[6] = b6;
  frame.data[7] = b7;
}

// CLASSES

//// CLASS IGBT
class IGBTswitch
{
  private:
  uint8_t outPWMpin;            // Pin for PWM control of IGBT 
  uint8_t outResetpin;          // Pin to reset IGBT
  uint8_t inFaultpin;           // Pin flagging IGBT error
  bool hasThermister;           // does the IGBT have a Thermister attached
  uint8_t locatID;              // location of the IGBT (1-6)
  igbtstatus statusIGBT;        // current state {INORMAL, IFAULT};
  bool onStatus;                // Flag if currently running
  uint8_t dutyCycle;            // duty cycle for the IGBT (0-255)
  int32_t freqPWM;              // MS duration for each duty cycle square wave
  unsigned long timeFault;      // time at which the fault was first detected
  uint16_t faultCount;          // Number of times that fault has set/reset since startup

  public:
  IGBTswitch()
  {
    // Initialize the values
    outResetpin = 0;
    inFaultpin = 0;
    hasThermister = false;
    locatID = 0;
    
    doReset();
  }

  // Set up the IGBT, reset and fault pins if 0 are ignored
  setupIGBT(uint8_t oPWMpin, uint8_t oResetpin, uint8_t iFaultpin, int32_t fPWM, bool bTherm, uint8_t locat)
  {
    // Pull in the values
    outResetpin = oResetpin;
    inFaultpin = iFaultpin;
    hasThermister = bTherm;
    locatID = locat;
    outPWMpin = oPWMpin;
    freqPWM = fPWM;

    //sets the frequency for the specified pin
    bool success;
    //success = SetPinFrequencySafe(outPWMpin, freqPWM);
    success = true;

    // Set up the pins
    if (success)
    {
      pinMode(outPWMpin, OUTPUT);
    }
    else
    {
      Serial.print("Error setting frequency for PWMpin #");
      Serial.println(outPWMpin);
    }

    // Setup the Reset and Fault pins (if not 0 = undefined)
    if (outResetpin > 0)
    {
      pinMode(outResetpin, OUTPUT);
      digitalWrite(outResetpin, HIGH);
    }
    if (inFaultpin > 0)
    {
      pinMode(inFaultpin, INPUT);
    }
    
    // All other internal variables
    doReset();
  }

  doReset()
  {
    statusIGBT = INORMAL;
    onStatus = false;
    dutyCycle = 0;
    timeFault = 0;
    faultCount = 0;
  }

  void printIGBT()
  {
    Serial.print("IGBT #");
    Serial.print(locatID);
    Serial.print(", Mode ");
    if(statusIGBT == INORMAL){Serial.print("NORMAL");}
    if(statusIGBT == IFAULT){Serial.print("FAULT");}
    Serial.print(", PWMpin #");
    Serial.print(outPWMpin);
    if(onStatus){Serial.print(", Status ON");}else{Serial.print(", Status OFF");}
    Serial.print(", Duty Cycle ");
    Serial.print(dutyCycle);
    Serial.print(", Frequency ");
    Serial.print(freqPWM);
    Serial.println();
  }

  // Sets duty cycle of IGBT between 0 and MAX.  Set duty cycle to 0 to switch IGBT off
  void setDutyCycle(uint8_t duty)
  {
    // Validate the number range of the duty cycle request
    if (duty < MIN_DUTY_CYCLE){duty = MIN_DUTY_CYCLE;}
    if (duty > MAX_DUTY_CYCLE){duty = MAX_DUTY_CYCLE;}

    // Store the duty cycle value
    dutyCycle = duty;
    
    // Switch on/off the duty cycle
    if (dutyCycle > MIN_DUTY_CYCLE)
    {
      onStatus = true;
      //pwmWrite(outPWMpin, dutyCycle);
      analogWrite(outPWMpin, dutyCycle);
    }
    else // PWM shutdown
    {
      onStatus = false;
      //pwmWrite(outPWMpin, MIN_DUTY_CYCLE);
      analogWrite(outPWMpin, MIN_DUTY_CYCLE);
    }
  }

  uint8_t getDuty()
  {
    return dutyCycle;
  }
  
  // Checks the fault line and sends CAN if first found
  checkIGBT()
  {
    // FaultPin = 0 means not defined or connected
    if (inFaultpin != 0)
    {
      // Check if a fault is waiting to be cleared
      if (statusIGBT == IFAULT)
      {
        // Has enough time passed to clear the fault?
        if (millis() > timeFault + TIME_FAULT_CLEAR)
        {
          // Reset the Fault clock
          timeFault = millis();
          
          // Only clear the fault if it has happened less than a maximum number of times
          if (faultCount < MAX_IGBT_FAULT_COUNT)
          {
            // Clearing the fault requires that PWM is off and RESET pulsed LOW (for 0.1 micro second)
            //pwmWrite(outPWMpin, 0);
            analogWrite(outPWMpin, 0);
            digitalWrite(outResetpin, LOW);
            delay(1);     // Wait 1/1000th second
            digitalWrite(outResetpin, HIGH);
            //pwmWrite(outPWMpin, dutyCycle);
            analogWrite(outPWMpin, dutyCycle);
          }
        }
      }
      
      // Record the prior status
      igbtstatus priorStatusIGBT;
      priorStatusIGBT = statusIGBT;
      
      // Test for fault, value goes LOW, HIGH is normal
      if (digitalRead(inFaultpin) == LOW && priorStatusIGBT == INORMAL)
      {
        // Record the fault status and time
        statusIGBT = IFAULT;
        faultCount++;

        // Test if too many faults were detected/cleared
        if (faultCount >= MAX_IGBT_FAULT_COUNT)
        {
          // Send message indicating fault
          PTigbtfault.data[0] = locatID;              // IGBT number
          PTigbtfault.data[1] = statusIGBT;           // Normal or Fault status
          PTigbtfault.data[2] = outPWMpin;            // Pin that the IGBT is on
          sendMessage(PTigbtfault);

          Serial.print("FAULT MAX : IGBT #");
          Serial.print(locatID);
          Serial.print(" on Pin ");
          Serial.println(outPWMpin);
        }
      }
      
      // Reset the IGBT status if the fault line has cleared, Fault line HIGH is normal
      if (digitalRead(inFaultpin) == HIGH && priorStatusIGBT != INORMAL)
      {
        statusIGBT = INORMAL;
      }
    }
  }
};  // end of class

//// CLASS THERMISTER
class Thermister
{
  private:
  uint8_t inAnalogPin;
  uint8_t tempCurrent;  // in (0.5,-40) "iC" Format
  unsigned long timeLastRead;
  unsigned long timeLastWarn;
  thermtype typeTherm;    // enum {TYPE_IGBT, TYPE_PTC};
  uint8_t locatID;        // number of the IGBT or PTC location of the thermister     
  thermstatus statusTherm;    // enum {TNORMAL, TWARNING, TFAULT};

  public:
  Thermister()
  {
    // Initialize the values
    inAnalogPin = 0;
    typeTherm = TYPE_IGBT;
    locatID = 0;
    doReset();
  }
  
  // Set up the parameters
  setupThermister(uint8_t inPin, thermtype ttype, uint8_t locID)
  {
    // Set up the Thermister
    inAnalogPin = inPin;
    typeTherm = ttype;
    locatID = locID;
    
    doReset();
  }

  // Reset the variables that are not permanent
  doReset()
  {
    tempCurrent = 10; //  Need to set to non-trigger ambient
    timeLastRead = 0; // Trigger a read soon (in most cases)
    timeLastWarn = 0;
    statusTherm = TNORMAL;
  }

  // Dumps the status of the Thermister to the Serial port for debugging
  printTherm()
  {
    Serial.print("Thermister @");
    if(typeTherm == TYPE_IGBT){Serial.print("IGBT");}
    if(typeTherm == TYPE_PTC){Serial.print("PTC");}
    Serial.print("#");
    Serial.print(locatID);
    Serial.print(" temperture ");
    Serial.print( convertItoC(tempCurrent));
    Serial.print("C, status ");
    if(statusTherm == TNORMAL){Serial.print("NORMAL");}
    if(statusTherm == TWARNING){Serial.print("WARNING");}
    if(statusTherm == TFAULT){Serial.print("FAULT");}
    Serial.print(", on Pin A");
    Serial.print(inAnalogPin);
    Serial.print(", Reading ");
    Serial.print(analogRead(inAnalogPin));
    Serial.print(" (0-1023)");
    Serial.println("");    
  }

  // Force a read of the Thermister
  int readThermC()
  {
    uint16_t tread;
    uint8_t ttemp;
    tread = analogRead(inAnalogPin);
   
    // Convert the treading to a temperature
    for (uint8_t x=0; x<14;x++)
    {
      if (tread >= tArray[x] && tread <= tArray[x+1])
      {
        // Interpolate between the two values, but pull values returned from the C listing
        ttemp =  cArray[x] + ((cArray[x+1] - cArray[x])   * ((tread  - tArray[x])/(tArray[x+1] - tArray[x]))); 
      }
    }
    // Update the stored temperature
    tempCurrent = ttemp;
    return convertItoC(tempCurrent);
  }

  uint8_t readThermI()
  {
    readThermC();
    return tempCurrent;
  }

  // In loop check of the therm, read if necessary, warn/fault as indicated
  void checkTherm()
  {
    // Has enough time passed since the last read
    if (millis() > timeLastRead + TIME_BETWEEN_THERMREAD)
    {
      int ttemp;
      ttemp = readThermC();
      timeLastRead = millis();
      thermstatus priorStatusTherm;

      // Reset the internal status
      priorStatusTherm = statusTherm;
      statusTherm = TNORMAL;

      // Check the temp limits for each type of Thermister use
      if (typeTherm == TYPE_IGBT)
      {
        // If the current temp outside of limited
        if (ttemp >= THERM_IGBT_WARN_TEMPC){statusTherm = TWARNING;}
        if (ttemp >= THERM_IGBT_FAULT_TEMPC){statusTherm = TFAULT;}
      }
      if (typeTherm == TYPE_PTC)
      {
        // If the current temp outside of limited
        if (ttemp >= THERM_PTC_WARN_TEMPC){statusTherm = TWARNING;}
        if (ttemp >= THERM_PTC_FAULT_TEMPC){statusTherm = TFAULT;}
      }
      // Send warning Frame if not in normal range
      if (statusTherm != TNORMAL)
      {
        // Send a warning/fault message to TesLorean CAN
        PTstatus.data[0] = 0; // Normal, then replaced
        if (statusTherm == TWARNING){PTstatus.data[0] = 1;}
        if (statusTherm == TFAULT){PTstatus.data[0] = 2;}
        PTstatus.data[1] = typeTherm;
        PTstatus.data[2] = statusTherm;
        PTstatus.data[3] = convertCtoI(ttemp);
        sendMessage(PTstatus);

        if(statusTherm == TWARNING){Serial.print("WARNING !!!");}
        if(statusTherm == TWARNING){Serial.print("FAULT !!!");}
        printTherm();

        timeLastWarn = millis();
      }
      else    // Now Normal
      {
        // Check if the warning or fault status is set, and has been for a while
        if ((priorStatusTherm != TNORMAL) && (millis() > timeLastWarn + TIME_CLEAR_THERMWARN))
        {
          // Clear the warning/fault status
          statusTherm = TNORMAL; 

          // TBD - Need to send a message clearing the warning or fault???
        }
      }
    }
  }
}; // end of Thermister class

//// CLASS CANCIRCULARARRAY
class CanCircularArray
{
  private:
  // Array for stored circular array CAN frames
  can_frame CANFrames[CAN_CIRCULAR_SIZE + 1];
  
  // Counters tracking the add point and read point in the circular array
  volatile uint8_t addPointFrames;
  volatile uint8_t readPointFrames;
  volatile uint16_t netAddReadCount;   // Adds increment, Reads decrement, only Read if <> 0

  public:
  can_frame cca_frame;
  bool cca_available;
  
  CanCircularArray()
  {
    // Set the pointers
    addPointFrames = 0;
    readPointFrames = 0;
    netAddReadCount = 0;

    // Initialize the public direct data
    cca_available = false;
    cca_frame.can_id = 0;
    cca_frame.can_dlc = 0;
    for(uint8_t x=0;x<8;x++){cca_frame.data[x] = 0;}   
  }

  void addCanFrame(can_frame & frame)
  {
    // Transfer the frame into the circular buffer
    CANFrames[addPointFrames].can_id = frame.can_id;
    CANFrames[addPointFrames].can_dlc = frame.can_dlc;
    for (uint8_t x = 0;x<8;x++){CANFrames[addPointFrames].data[x] = frame.data[x];}
 
    // Increment the circular array pointers
    addPointFrames = (addPointFrames + 1) % CAN_CIRCULAR_SIZE;
    netAddReadCount++;    
  }

  bool canAvailable()
  {
    // Check for received frames
    if (netAddReadCount > 0)
    {
      if (readPointFrames != addPointFrames)
      {
        return true;
      }
    }
    return false;    
  }

  // Populates the public frame cca_frame with next frame in the circular array
  // uses the public flag cca_available to signal that a next frame was available
  void getCanFrame(can_frame & frame)
  {
    // Check for received frames
    if (netAddReadCount > 0)
    {
      if (readPointFrames != addPointFrames)
      {
        // Process the frame received
        frame.can_id = CANFrames[readPointFrames].can_id;
        frame.can_dlc = CANFrames[readPointFrames].can_dlc;
        for(uint8_t x=0;x<8;x++){frame.data[x] = CANFrames[readPointFrames].data[x];}
  
        // Move on to the next frame to read
        readPointFrames = (readPointFrames + 1) % CAN_CIRCULAR_SIZE;
        if(netAddReadCount > 0){netAddReadCount--;}
      }
    }
  }
};    // end of CanCircularArray

// KEY DECLARATIONS

// Create the CAN object that links to the CAN transceiver
MCP_CAN can0(10);

// Circular array setup
CanCircularArray CanCir;

void irqCANHandler()
{
  if(!digitalRead(PIN_IN_CAN_INTERRUPT))      // interrupt pin low if data on can0
  {
    can_frame captframe;
    
    // Capture the frame
    can0.readMsgBuf(&captframe.can_dlc, captframe.data);
    captframe.can_id = can0.getCanId();

    // Add frame to the circular buffer
    CanCir.addCanFrame(captframe);   
  }
}

void sendMessage(can_frame & frame)
{
  can0.sendMsgBuf(frame.can_id, 0, frame.can_dlc, frame.data);
}

int convertItoC(uint8_t itemp)
{
  int newC;
  newC = (itemp / 2) - 40;
  return newC;
}

uint8_t convertCtoI(int ctemp)
{
  uint8_t newI;
  newI = (ctemp + 40) * 2;
}

//// SETUP

// CREATE OBJECTS
// PIMpin (uint8_t oPWMpin, int32_t fPWM)
//PWMpin PWMPin1;
// IGBTswitch(uint8_t oResetpin, uint8_t iFaultpin, bool bTherm, uint8_t locat)
IGBTswitch IGBT1;
//IGBTswitch IGBT2;
//IGBTswitch IGBT3;
//IGBTswitch IGBT4;
//IGBTswitch IGBT5;
//IGBTswitch IGBT6;
// Thermister(uint8_t inPin, thermtype ttype, uint8_t locID)
Thermister Therm1;
Thermister Therm2;
Thermister Therm3;
Thermister Therm4;

// FUNCTIONS (MORE)

// Reset all of the Systems, Controls, and Units to start-point defaults
void doFullReset()
{
  // Resets
//  PWMPin1.doReset();
  IGBT1.doReset();
  //IGBT2.doReset();
  //IGBT3.doReset();
  //IGBT4.doReset();
  //IGBT5.doReset();
  //IGBT6.doReset();
  Therm1.doReset();
  Therm2.doReset();
  Therm3.doReset();
  Therm4.doReset();
}

// Dump out the status of all Systems, Controls, Units to the Serial port
void doFullPrint()
{
  // PWMs
//  Serial.println("PWMs...");
//  PWMPin1.printPWMpin();
  // IGBTs
  Serial.println("IGBTs...");
  IGBT1.printIGBT();
  //IGBT2.printIGBT();
  //IGBT3.printIGBT();
  //IGBT4.printIGBT();
  //IGBT5.printIGBT();
  //IGBT6.printIGBT();
  // Thermisters
  Serial.println("Thermisters...");
  Therm1.printTherm();
  Therm2.printTherm();
  Therm3.printTherm();
  Therm4.printTherm();
  Serial.println();
}

void Controller_CAN_Messages()
{
  can_frame outframe;  //A structured variable according to due_can library for transmitting CAN data.

  // On 1Hz send out a heatbeat message
  if (millis() > timeLastHeartbeat + TIME_BETWEEN_HEARTBEAT)
  {
    // Include some data in the heartbeat message
    PTheartbeat.data[0] = IGBT1.getDuty();
    PTheartbeat.data[1] = MAX_DUTY_CYCLE;
    PTheartbeat.data[2] = Therm1.readThermI();
    PTheartbeat.data[3] = Therm2.readThermI();
    PTheartbeat.data[4] = Therm3.readThermI();
    PTheartbeat.data[5] = Therm4.readThermI();
        
    sendMessage(PTheartbeat);
    timeLastHeartbeat = millis();
  }
}

void setup()
{
  Serial.println("Setup...");
  
  // Set up Serial port for test input and outputs
  while (!Serial);
  Serial.begin(115200);
  Serial.println("Serial started @ 115200 baud.");

  // Start SPI
  SPI.begin();    // init the SPI communications

  // Prepare the interrupt pins
  pinMode(PIN_IN_CAN_INTERRUPT, INPUT);
  
  // Set up TESLOREAN CAN interface
  can0.begin(CAN_500KBPS);

  // Set the Startup timer
  timeSinceStartup = millis();
  inStartupPeriod = true;
  
  // CAN Interrupt
//  attachInterrupt(digitalPinToInterrupt(PIN_IN_CAN_INTERRUPT), irqCANHandler, LOW);

  // PWM control - initialize all timers except for 0, to save time keeping functions
  //InitTimersSafe();

  ////// Setup the objects
  
  // IGBTswitch(uint8_t oResetpin, uint8_t iFaultpin, bool bTherm, uint8_t locat)
  IGBT1.setupIGBT(PIN_PWM_1, PIN_RESET_1, PIN_FAULT_1,PWM_DEFAULT_FREQ, false, 1);
  //IGBT2.setupIGBT(PIN_PWM_2, PIN_RESET_2, PIN_FAULT_2,PWM_DEFAULT_FREQ, true, 2);
  //IGBT3.setupIGBT(PIN_PWM_3, PIN_RESET_3, PIN_FAULT_3,PWM_DEFAULT_FREQ, false, 3);
  //IGBT4.setupIGBT(PIN_PWM_4, PIN_RESET_4, PIN_FAULT_4,PWM_DEFAULT_FREQ, false, 4);
  //IGBT5.setupIGBT(PIN_PWM_5, PIN_RESET_5, PIN_FAULT_5,PWM_DEFAULT_FREQ, true, 5);
  //IGBT6.setupIGBT(PIN_PWM_6, PIN_RESET_6, PIN_FAULT_6,PWM_DEFAULT_FREQ, false, 6);
  
  // Thermister(uint8_t inPin, thermtype ttype, uint8_t locID)
  Therm1.setupThermister(PIN_THERM_1, TYPE_IGBT, 2);
  Therm2.setupThermister(PIN_THERM_2, TYPE_IGBT, 5);
  Therm3.setupThermister(PIN_THERM_3, TYPE_PTC, 1);
  Therm4.setupThermister(PIN_THERM_4, TYPE_PTC, 6);

  // Dump out details
  doFullPrint();

  // Pre-defined CAN frames
  buildCANframe(PTrollcall,PTCRollCallResFrameID,8,MODULE_PTC_ID,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(PTheartbeat,PTCHeartbeatFrameID,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(PTstatus,PTCStatusID,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);

  // Start the watchdog
  wdt_enable(WDTO_8S);    // Eight second reset
}

//// FUNCTIONS

// Process instructions on the TesLorean CAN bus
void canDecode(can_frame & frame)
{
  switch (frame.can_id)
  {
    // Rollcall request from the Trip Computer, respond with the pre-prepared frame
    case PTCRollCallReqFrameID:    // 0x818
    {
      sendMessage(PTrollcall);
      break;
    }

    // Request to turn PTC on/off and/or update the Duty Cycle
    case PTCDutyCycleReqFrameID:    // 0x877
    {
      if (frame.data[0] == 0)
      {
        // Switch the PTC off
        IGBT1.setDutyCycle(MIN_DUTY_CYCLE);
      }
      if (frame.data[0] == 1)
      {
        // Switch the PTC on AND update the duty cycle
        IGBT1.setDutyCycle(frame.data[1]);
      }
      break;
    }
    
    default:
    {
      // if nothing else matches, do the default
      break;
    }
  }
}

//// LOOP

void loop()
{
  // Reset the watchdog
  wdt_reset();

  // LOGIC

  // Only start checks after the startup period
  if (inStartupPeriod)
  {
    if (millis() > timeSinceStartup + STARTUP_NOFAULT_TIME){inStartupPeriod = false;}
  }
  else  // out of startup and in normal operations
  {
    // Step 1 - Check all the IGBTs, Check Thermisters
    //IGBT1.checkIGBT();
    //IGBT2.checkIGBT();
    //IGBT3.checkIGBT();
    //IGBT4.checkIGBT();
    //IGBT5.checkIGBT();
    //IGBT6.checkIGBT();
    Therm1.checkTherm();
    Therm2.checkTherm();
    Therm2.checkTherm();
    Therm2.checkTherm();
  }

  // CAN FRAME HANDLING

  // Is frame waiting for processing
  if (CanCir.canAvailable())
  {
    // Process the frame
    can_frame ccaframe;
    CanCir.getCanFrame(ccaframe);
    canDecode(ccaframe);
  }
      
  // Selectively send out status and informational messages
  Controller_CAN_Messages();

  // Check the Serial Bus for commands
  if (Serial.available())
  {
    char incomingByte;
    incomingByte = Serial.read(); // read the incoming byte:

    switch (incomingByte)
    {
      case '?'://? for instructions
      {
        Serial.println("Instructions : d(display), t(turn on), s(turn off), r(Reset)");
        break;
      }

      case 'd'://d for display
      {
        Serial.println("Controls Status :");
        doFullPrint();
        break;
      }
        
      case 'r'://r for reset
      {
        doFullReset();
        Serial.println("All IGBTs and Thermisters reset.");
        break;
      }

      case 't'://t for turn on
      {
        // Switch the PTC on AND update the duty cycle
        IGBT1.setDutyCycle(MAX_DUTY_CYCLE);
        //IGBT2.setDutyCycle(MAX_DUTY_CYCLE);
        //IGBT3.setDutyCycle(MAX_DUTY_CYCLE);
        //IGBT4.setDutyCycle(MAX_DUTY_CYCLE);
        //IGBT5.setDutyCycle(MAX_DUTY_CYCLE);
        //IGBT6.setDutyCycle(MAX_DUTY_CYCLE);

        Serial.println("All IGBTs set to 50% of Max.");
        break;
      }

      case 's'://s for turn off
      {
        // Switch the PTC on AND update the duty cycle
        IGBT1.setDutyCycle(MIN_DUTY_CYCLE);
        //IGBT2.setDutyCycle(MIN_DUTY_CYCLE);
        //IGBT3.setDutyCycle(MIN_DUTY_CYCLE);
        //IGBT4.setDutyCycle(MIN_DUTY_CYCLE);
        //IGBT5.setDutyCycle(MIN_DUTY_CYCLE);
        //IGBT6.setDutyCycle(MIN_DUTY_CYCLE);

        Serial.println("All IGBTs set to Min (off).");
        break;
      }
        
      default:
      {
        // if nothing else matches, do the default
        // default is optional
        break;
      }
    }
  }
}
