
class PID {
  public:
    float Kp, Ki, Kd;
    float setpoint, lastError, integral;
    unsigned long lastTime;

    PID(float Kp, float Ki, float Kd, float setpoint):Kp{Kp}, Ki{Ki}, Kd{Kd}, setpoint{setpoint} {
      lastError = 0;
      integral = 0;
      lastTime = millis();  
    }

    float compute(float input) {
      unsigned long currentTime = millis();
      float dt = (currentTime - lastTime) / 1000.0;
      lastTime = currentTime;

      float error = setpoint - input;
      integral += (error * dt);  
      float derivative = (error - lastError) / dt;  
      lastError = error;

      return (Kp * error) + (Ki * integral) + (Kd * derivative); 
    }
};


class ExponentialSmoothing {
  private:
    float alphaValue, prevValue;

  public:
    ExponentialSmoothing(float alphaValue):alphaValue{alphaValue} {
      prevValue = 0;
    }

    float filter(float input) {
      prevValue = alphaValue * input + (1 - alphaValue) * prevValue;
      return prevValue;
    }
};


int motorPin = 9;   
float current_pos = 0.0;  
float output = 0.0;        

PID pid(1.0, 0.01, 0.1, 200);  // Kp Ki Kd Target_Point
ExponentialSmoothing filter(0.2);  // alpha 


void setup() {
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);  
}

void loop() {

  output = pid.compute(current_pos);
  
  output = filter.filter(output);
  
  int pwmOutput = map(output, 0, 255, 0, 255);
  pwmOutput = constrain(pwmOutput, 0, 255);

  analogWrite(motorPin, pwmOutput);

  Serial.print("Setpoint: ");
  Serial.print(pid.setpoint);
  Serial.print(", Current speed: ");
  Serial.print(pwmOutput);
  Serial.print(", PID Output: ");
  Serial.println(output);
  
  delay(100);
}
