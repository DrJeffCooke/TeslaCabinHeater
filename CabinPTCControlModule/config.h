// Thermal Control Module

// Relay Statuses
#define RELAY_ON 0
#define RELAY_OFF 1

// BIT MANIPULATION a=target variable, b=bit number to act upon 0-n
#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1ULL<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1ULL<<(b))))        // '!!' to make sure this returns 0 or 1

#define PTCRollCallResFrameID 0x819    // ID to send back to the Trip Computer
#define PTCRollCallReqFrameID 0x818    // ID from Trip Computer requesting rollcall
#define PTCHeartbeatFrameID 0x0822     // Alive 1Hz message
#define PTCStatusID 0x879              // Mostly a means of flagging a fault condition, sends out max setting for Duty Cycle
#define PTCDutyCycleReqFrameID 0x877    // Switch the PTC Cabin heater on/off and set the duty cycle

#define MODULE_PTC_ID 0x017           // PTC id number (see Trip computer roll-call logic

// Heartbeat message every 1 second
#define HEARTBEAT_FREQ 1000    

// How many history values to store for temperature
#define CAN_CIRCULAR_SIZE 10

// CAN communications and interrupt pin
// (definition included here as wiring guide only)
#define PIN_COMMS_SCK 13
#define PIN_COMMS_MISO 12
#define PIN_COMMS_MOSI 11
#define PIN_IN_CAN_INTERRUPT 2
#define PIN_IN_CAN_CHIP_SELECT 10

// PTC Cabin Heater Settings
#define THERMISTERRESISTOR 10000    // 10K resistor for the voltage divider circuit on the thermister
#define MIN_DUTY_CYCLE 0            // 0-255 to limit the IGBT
#define MAX_DUTY_CYCLE 200         // 0-255 to limit the IGBT for heat issues
#define PWM_DEFAULT_FREQ 1000       // 1KHz as a starting point, uses Arduino PWM service
#define TIME_BETWEEN_THERMREAD 2000 // ms between thermister reads
#define THERM_IGBT_WARN_TEMPC 100   // temperature for IGBT warning
#define THERM_IGBT_FAULT_TEMPC 125  // temperature C for IGBT fault signal
#define THERM_PTC_WARN_TEMPC 100    // temperature for PCT warning
#define THERM_PTC_FAULT_TEMPC 125   // temperature C for PCT fault signal
#define TIME_CLEAR_THERMWARN 10000  // warning or fault can be cleared after this minimum ms
#define TIME_BETWEEN_HEARTBEAT 1025 // Roughly a second every heartbeat
#define TIME_FAULT_CLEAR 250        // ms after a fault is detected before it should be cleared
#define MAX_IGBT_FAULT_COUNT 100    // If IGBT keeps faulting out, don't clear the faults
#define STARTUP_NOFAULT_TIME 2000   // Don't pay attention to Faults in this initial period

#define PIN_PWM_1 5                                                         
#define PIN_PWM_2 3
#define PIN_PWM_3 7
#define PIN_PWM_4 8
#define PIN_PWM_5 4
#define PIN_PWM_6 6
#define PIN_RESET_1 28
#define PIN_RESET_2 26
#define PIN_RESET_3 30
#define PIN_RESET_4 31
#define PIN_RESET_5 27
#define PIN_RESET_6 29
#define PIN_FAULT_1 16
#define PIN_FAULT_2 14
#define PIN_FAULT_3 18
#define PIN_FAULT_4 19
#define PIN_FAULT_5 15
#define PIN_FAULT_6 17
#define PIN_THERM_1 0
#define PIN_THERM_2 1
#define PIN_THERM_3 2
#define PIN_THERM_4 3
