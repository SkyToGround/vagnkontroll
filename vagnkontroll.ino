#include <Controllino.h>  /* Usage of CONTROLLINO library allows you to use CONTROLLINO_xx aliases in your sketch. */
#include <OneWire.h>
#include <DallasTemperature.h>

int const OutsideLampPin = CONTROLLINO_D6;
int const LargeLampPin1 = CONTROLLINO_D1;
int const LargeLampPin2 = CONTROLLINO_D3;
int const SmallLampPin = CONTROLLINO_D2;
int const DoorSwitchA = CONTROLLINO_A0;
int const DoorSwitchB = CONTROLLINO_A1;
int const DoorSwitchC = CONTROLLINO_A2;
int const BedSwitchA = CONTROLLINO_A3;
int const BedSwitchB = CONTROLLINO_A4;
//int const InsideTempPin = ?;
//int const OutsideTempPin = ?;


//int const BedSwitchA = 12;
int const InsideTempPin = 3;
// int const OutsideTempPin = ?;


class Output {
  public:
  Output(int Pin) : OutputPin(Pin) {
    Serial.print("Output pin is: ");
    Serial.println(OutputPin);
    pinMode(OutputPin, OUTPUT);
    digitalWrite(OutputPin, LOW);
  }
  void setState(int NewState) {
    Serial.print("Write ");
    Serial.print(NewState);
    Serial.print(" to pin ");
    Serial.println(OutputPin);
//digitalWrite(OutputPin, NewState);
  }
private:
  int OutputPin;
};

class LampBase {
public:
  LampBase() {}
  void switchState() {
    Serial.println("Switching output");
    if (CurrentState == HIGH) {
      CurrentState = LOW;
    } else {
      CurrentState = HIGH;
    }
    setState(CurrentState);
  }
  virtual void setState(int NewState) = 0;
private:
  int CurrentState{LOW};
};

class DualLamp : public LampBase {
  public:
  DualLamp(Output &Lamp1, Output &Lamp2) : Lamp1(Lamp1), Lamp2(Lamp2) {}

  void setState(int NewState) override {
    Lamp1.setState(NewState);
    Lamp2.setState(NewState);
  }
private:
  Output &Lamp1;
  Output &Lamp2;
};

class Lamp : public LampBase {
public:
  Lamp(Output *LampOutput) : Output(Output), CurrentState(LOW) {}
  void setState(int NewState) override {
    Serial.print("Output set to ");
    Serial.println(NewState);
    Output->setState(NewState);
  }
private:
  Output *Output;
  int CurrentState;
};

class LightSwitch {
public:
  LightSwitch(int InputPin, LampBase *ControlLamp) : Pin(InputPin), LastValue(digitalRead(InputPin)), UsedLamp(ControlLamp) {
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
      Serial.println("State switched");
      LastValue = CurrentValue;
      LastChangeTime = CurrentTime;
      UsedLamp->switchState();
    }
  }
private:
  const unsigned long ChangeTimeout{100};
  unsigned long LastChangeTime{0};
  int Pin;
  int LastValue;
  LampBase *UsedLamp;
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


// Output LargeLampOutput1(LargeLampPin1);
// Output LargeLampOutput2(LargeLampPin2);
// DualLamp LargeLamp(&LargeLampOutput1, &LargeLampOutput2);
// LightSwitch LargeLampSwitch1(DoorSwitchB, &LargeLamp);
 
// Output OutsideLampOutput(OutsideLampPin);



// Lamp OutsideLamp(OutsideLampOutput);


// LightSwitch SmallLampSwitch2(BedSwitchA, SmallLamp);

// 
// LightSwitch LargeLampSwitch2(BedSwitchB, LargeLamp);

// LightSwitch OutsideLampSwitch(DoorSwitchC, OutsideLamp);

// TemperatureSensor TempSensor(InsideTempPin);

void setup() {
  Serial.begin(9600);
  Serial.println("Start");
}


Output SmallLampOutput(SmallLampPin);
Lamp SmallLamp(&SmallLampOutput);
LightSwitch SmallLampSwitch1(DoorSwitchA, &SmallLamp);


void loop() {
  digitalWrite(CONTROLLINO_D2, digitalRead(CONTROLLINO_A0));
  // SmallLampSwitch1.loopFunction();
  // SmallLampSwitch2.loopFunction();
  // LargeLampSwitch1.loopFunction();
  // LargeLampSwitch2.loopFunction();
  // OutsideLampSwitch.loopFunction();
	// TempSensor.loopFunction();
}
