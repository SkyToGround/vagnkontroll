//#include <Controllino.h>  /* Usage of CONTROLLINO library allows you to use CONTROLLINO_xx aliases in your sketch. */
int const SmallLampPin = 6; //CONTROLLINO_D2;
int const DoorSwitchA = 14; //CONTROLLINO_A0;

int const OutsideLampPin = 18; //CONTROLLINO_D6;
int const LargeLampPin1 = 5; //CONTROLLINO_D1;
int const LargeLampPin2 = 7; //CONTROLLINO_D3;
int const DoorSwitchB = 15; //CONTROLLINO_A1;
int const DoorSwitchC = 16; //CONTROLLINO_A2;
int const BedSwitchA = 17; //CONTROLLINO_A3;
int const BedSwitchB = 20; //CONTROLLINO_A4;
int const BedSwitchC = 21; //CONTROLLINO_A5;
//int const InsideTempPin = ?;
//int const OutsideTempPin = ?;


//int const BedSwitchA = 12;
int const InsideTempPin = 3;
// int const OutsideTempPin = ?;
char StrBuf[50];


class Output {
  public:
  Output(int Pin) : OutputPin(Pin) {
    pinMode(Pin, OUTPUT);
    digitalWrite(Pin, LOW);
  }
  void setState(int NewState) {
    Serial.println(StrBuf);
    digitalWrite(OutputPin, NewState);
  }
private:
  int const OutputPin;
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
  DualLamp(Output *Lamp1, Output *Lamp2) : Lamp1Ptr(Lamp1), Lamp2Ptr(Lamp2) {}

  void setState(int NewState) override {
    Lamp1Ptr->setState(NewState);
    Lamp2Ptr->setState(NewState);
  }
private:
  Output *Lamp1Ptr;
  Output *Lamp2Ptr;
};

class Lamp : public LampBase {
public:
  Lamp(Output *LampOutput) : Output(LampOutput) {}
  void setState(int NewState) override {
    Serial.print("Output set to ");
    Serial.println(NewState);
    Output->setState(NewState);
  }
private:
  Output *Output;
};

class LightSwitchBase {
  public:
  LightSwitchBase(LampBase *ControlLamp, int CurrentValue) : LastValue(CurrentValue), UsedLamp(ControlLamp) {}
  void handleInput(int CurrentValue) {
    unsigned long CurrentTime = millis();
    if (CurrentTime < LastChangeTime) {
      LastChangeTime = 0; 
    }
    if (CurrentTime < LastChangeTime + ChangeTimeout) {
      return;
    }
    if (CurrentValue != LastValue) {
      LastValue = CurrentValue;
      LastChangeTime = CurrentTime;
      UsedLamp->switchState();
    }
  }
  virtual void loopFunction() = 0;
private:
  const unsigned long ChangeTimeout{100};
  unsigned long LastChangeTime{0};
  int LastValue;
  LampBase *UsedLamp;
};

class LightSwitch : public LightSwitchBase {
public:
  LightSwitch(int InputPin, LampBase *ControlLamp) : Pin(InputPin), LightSwitchBase(ControlLamp, digitalRead(Pin)) {
    pinMode(InputPin, INPUT);
  }
  void loopFunction() override {
    handleInput(digitalRead(Pin));
  }
private:
  int Pin;
};

class LightSwitchADC : public LightSwitchBase {
public:
  LightSwitchADC(int InputPin, LampBase *ControlLamp) : Pin(InputPin), LightSwitchBase(ControlLamp, analogRead(Pin) > ADC_Threshold) {
    //pinMode(InputPin, INPUT);
  }
  void loopFunction() override {
    int av = analogRead(Pin);
    //sprintf(StrBuf, "Analog read of pin %d gives %d", Pin, av);
    //Serial.println(StrBuf);
    handleInput(av > ADC_Threshold);
  }
private:
  const int ADC_Threshold{200};
  int Pin;
};


void setup() {
  Serial.begin(9600);
  Serial.println("Start");
}


void loop() {
  static Output SmallLampOutput(SmallLampPin);
  static Lamp SmallLamp(&SmallLampOutput);
  static LightSwitch SmallLampSwitch1(DoorSwitchA, &SmallLamp);
  static LightSwitch SmallLampSwitch2(BedSwitchA, &SmallLamp);

  static Output LargeLampOutput1(LargeLampPin1);
  static Output LargeLampOutput2(LargeLampPin2);
  static DualLamp LargeLamp(&LargeLampOutput1, &LargeLampOutput2);
  static LightSwitch LargeLampSwitch1(DoorSwitchB, &LargeLamp);
  static LightSwitchADC LargeLampSwitch2(BedSwitchB, &LargeLamp);

  static Output OutsideLampOutput(OutsideLampPin);
  static Lamp OutsideLamp(&OutsideLampOutput);
  static LightSwitch OutsideLampSwitch(DoorSwitchC, &OutsideLamp);
  static LightSwitchADC OutsideLampSwitch2(BedSwitchC, &OutsideLamp);

  SmallLampSwitch1.loopFunction();
  SmallLampSwitch2.loopFunction();
  LargeLampSwitch1.loopFunction();
  LargeLampSwitch2.loopFunction();
  OutsideLampSwitch.loopFunction();
  OutsideLampSwitch2.loopFunction();
}
