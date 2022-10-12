//#include <Controllino.h>  /* Usage of CONTROLLINO library allows you to use CONTROLLINO_xx aliases in your sketch. */
#include <OneWire.h>
#include <DallasTemperature.h>

//int const OutsideLampPin = CONTROLLINO_D1;
//int const BigLampsPin = CONTROLLINO_D3;
//int const SmallLampPin = CONTROLLINO_D2;
//int const DoorSwitchA = CONTROLLINO_A0;
//int const DoorSwitchB = CONTROLLINO_A1;
//int const DoorSwitchC = CONTROLLINO_A2;
//int const BedSwitchA = CONTROLLINO_A3;
//int const InsideTempPin = ?;
//int const OutsideTempPin = ?;

// int const OutsideLampPin = CONTROLLINO_D1;
// int const BigLampsPin = CONTROLLINO_D3;
int const SmallLampPin = LED_BUILTIN;
int const DoorSwitchA = 2;
//int const DoorSwitchB = CONTROLLINO_A1;
//int const DoorSwitchC = CONTROLLINO_A2;
//int const BedSwitchA = 12;
int const InsideTempPin = 3;
// int const OutsideTempPin = ?;


class Lamp {
public:
  Lamp(int OutputPin) : Pin(OutputPin), CurrentState(HIGH) {
    pinMode(OutputPin, OUTPUT);
    digitalWrite(OutputPin, CurrentState);
  }
  void switchState() {
    if (CurrentState == 1) {
      CurrentState = 0;
    } else {
      CurrentState = 1;
    }
    digitalWrite(Pin, CurrentState);
  }
private:
  int Pin;
  int CurrentState;
};

class LightSwitch {
public:
  LightSwitch(int InputPin, Lamp &ControlLamp) : Pin(InputPin), LastValue(digitalRead(InputPin)), UsedLamp(ControlLamp) {
    pinMode(InputPin, INPUT);
  }
  void loopFunction() {
    unsigned long CurrentTime = millis();
    if (CurrentTime < LastChangeTime) {
      LastChangeTime = 0; 
    }
    if (CurrentTime < LastChangeTime + ChangeTimeout) {
      return;
    }
    int CurrentValue = digitalRead(Pin);
    if (CurrentValue != LastValue) {
      LastValue = CurrentValue;
      LastChangeTime = CurrentTime;
      UsedLamp.switchState();
    }
  }
private:
  const unsigned long ChangeTimeout = 100;
  unsigned long LastChangeTime;
  int Pin;
  int LastValue;
  Lamp &UsedLamp;
};

class FanController {
public:
	FanController(int PowerPin, int SpeedPin) : OnPin(PowerPin), SpeedControlPin(SpeedPin) {
		pinMode(PowerPin, OUTPUT);
		digitalWrite(PowerPin, 0);
		pinMode(SpeedPin, OUTPUT);

    const word PWM_FREQ_HZ = 25000; //Adjust this value to adjust the frequency (Frequency in HZ!) (Set currently to 25kHZ)
    const word TCNT1_TOP = 16000000/(2*PWM_FREQ_HZ);

		TCCR1A = 0;
		TCCR1B = 0;
		TCNT1  = 0;
	  // Set Timer1 configuration
	  // COM1A(1:0) = 0b10   (Output A clear rising/set falling)
	  // COM1B(1:0) = 0b00   (Output B normal operation)
	  // WGM(13:10) = 0b1010 (Phase correct PWM)
	  // ICNC1      = 0b0    (Input capture noise canceler disabled)
	  // ICES1      = 0b0    (Input capture edge select disabled)
	  // CS(12:10)  = 0b001  (Input clock select = clock/1)
	  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
	  TCCR1B |= (1 << WGM13) | (1 << CS10);
	  ICR1 = TCNT1_TOP;
	}
	
	void turnOn(bool On) {}
	void setSpeed(float Speed) {}
	
	void loopFunction() {}
private:
	int OnPin;
	int SpeedControlPin;
};

class TemperatureConsumer {
public:
	virtual void setCurrentTemperature(float) = 0;
};

class TemperatureController : TemperatureConsumer {
public:
	enum class Mode {Off, Maintain, TempControl};
	TemperatureController(int HeaterPin, FanController &HeaterFan) : OutputPin(HeaterPin), HeaterFan(HeaterFan) {
		
	}
	
	void setMode(TemperatureController::Mode NewMode) {
		CMode = NewMode;
	}
	
	void setCurrentTemperature(float CurrentTemp) override {
		
	}
	
	void loopFunction() {}
private:
	Mode CMode{Mode::Maintain};
	float MaintainLow{5.0};
	float MaintainHigh{26.0};
	float TempControlTemp{24.0};
	float TempHysteresis{1.0};
	int OutputPin;
	FanController &HeaterFan;
};

class TemperatureSensor {
	public:
		TemperatureSensor(int InputPin) : Bus(InputPin), Sensors(&Bus) {
			Sensors.begin();
			Sensors.getAddress(Address, 0);
			if (Address == 0) {
				return;
			}
			Sensors.setResolution(Address, 12);
			Sensors.setWaitForConversion(false);
			Sensors.requestTemperatures();
		}
		void loopFunction() {
			if (Address == 0) {
				return;
			}
			if (Sensors.isConversionComplete()) {
				 Serial.println(Sensors.getTempC(Address));
				 Sensors.requestTemperatures();
			}
		}
		
private:
	OneWire Bus;
	DallasTemperature Sensors;
	DeviceAddress Address{0};
};

Lamp SmallLamp(SmallLampPin);
LightSwitch SmallLampSwitch1(DoorSwitchA, SmallLamp);
//LightSwitch SmallLampSwitch2(BedSwitchA, SmallLamp);
TemperatureSensor TempSensor(InsideTempPin);

void setup() {
	// Input
  pinMode(DoorSwitchA, INPUT);
  //digitalWrite(DoorSwitchA, HIGH);
  //pinMode(BedSwitchA, INPUT);
  //digitalWrite(BedSwitchA, HIGH);
	
	// Output
  pinMode(SmallLampPin, OUTPUT);
  // digitalWrite(CONTROLLINO_D0, HIGH);
  // pinMode(CONTROLLINO_D1, OUTPUT);
  // pinMode(CONTROLLINO_D3, OUTPUT);
  // pinMode(CONTROLLINO_D6, OUTPUT);
  Serial.begin(9600);
  //Serial.println("Start");
}

// the loop function runs over and over again forever
void loop() {
  //digitalWrite(CONTROLLINO_D0, digitalRead(CONTROLLINO_A0));
  // digitalWrite(CONTROLLINO_D1, digitalRead(CONTROLLINO_A0));
  // digitalWrite(CONTROLLINO_D3, digitalRead(CONTROLLINO_A0));
  
  //digitalWrite(CONTROLLINO_D2, digitalRead(CONTROLLINO_A1));
  
  // digitalWrite(CONTROLLINO_D6, digitalRead(CONTROLLINO_A2));
  SmallLampSwitch1.loopFunction();
  //SmallLampSwitch2.loopFunction();
	TempSensor.loopFunction();
}
