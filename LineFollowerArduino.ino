// Motor Pins
#define ENA A0  // PWM Pin for Motor A
#define ENB A1  // PWM Pin for Motor B
#define IN1 12  // Motor A IN1
#define IN2 13  // Motor A IN2
#define IN3 10  // Motor B IN3
#define IN4 11  // Motor B IN4

// Sensor Pins
#define S1 7
#define S2 6
#define S3 5
#define S4 4
#define S5 3

// Speed
#define BASE_SPEED 150  // Base motor speed
#define TURN_SPEED 150  // Base motor speed
#define MAX_SPEED 255   // Maximum motor speed

enum State {
  STRAIGHT,
  TURN_LEFT,
  TURN_RIGHT,
  SLIDE_LEFT,
  SLIDE_RIGHT
};

State currentState = State::STRAIGHT;

String stateToString(State state) {
  switch (state) {
    case SLIDE_LEFT:
      return "SLIDE_LEFT";
    case SLIDE_RIGHT:
      return "SLIDE_RIGHT";
    case STRAIGHT:
      return "STRAIGHT";
    case TURN_RIGHT:
      return "TURN_RIGHT";
    case TURN_LEFT:
      return "TURN_LEFT";
    default:
      return "UNKNOWN";
  }
}

void setup() {
  // Initialize motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialize sensor pins
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  currentState = State::STRAIGHT;

  Serial.begin(9600);
}

void loop() {

  // Read sensor values
  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);
  int s5 = digitalRead(S5);

  String sensors = String(s1) + String(s2) + String(s3) + String(s4) + String(s5);
  // Serial.println(sensors);

  if (currentState == State::STRAIGHT || currentState == State::SLIDE_LEFT || currentState == State::SLIDE_RIGHT) {
    if (sensors == "00000") {
      currentState = State::STRAIGHT;
    } else if (sensors == "00100") {
      currentState = State::STRAIGHT;
    } else if (sensors == "01000" || sensors == "01100") {
      currentState = State::SLIDE_LEFT;
    } else if (sensors == "00010" || sensors == "00110") {
      currentState = State::SLIDE_RIGHT;
    } else if (sensors == "10000" || sensors == "11000" || sensors == "11100" || sensors == "10100") {
      currentState = State::TURN_LEFT;
    } else if (sensors == "00001" || sensors == "00011" || sensors == "00111" || sensors == "00101") {
      currentState = State::TURN_RIGHT;
    }else{
      currentState = State::STRAIGHT;
    }
  }

  if(currentState == State::STRAIGHT){
    moveForwardWithAcceleration(MAX_SPEED, 1);
  }

  if(currentState == State::SLIDE_LEFT || currentState == State::SLIDE_RIGHT){
    // Calculate error value (weighted position)
    int error = (-1 * s1) + (-1 * s2) + (1 * s4) + (1 * s5);
    
    // Determine motor speeds based on error
    int leftMotorSpeed = constrain(BASE_SPEED - (error * 50), 0, MAX_SPEED);
    int rightMotorSpeed = constrain(BASE_SPEED + (error * 50), 0, MAX_SPEED);

    // Set motor directions
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    // Adjust motor speeds
    analogWrite(ENA, leftMotorSpeed);
    analogWrite(ENB, rightMotorSpeed);
  }

  if (currentState == State::TURN_RIGHT) {
    moveForwardWithAcceleration(MAX_SPEED, 1);
    
    analogWrite(ENA, TURN_SPEED);
    analogWrite(ENB, TURN_SPEED);

    
    // Set motor directions
    // Roda KIRI
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    // RODA KANAN
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    if (sensors == "00100" || sensors == "10000") {
      currentState = State::STRAIGHT;
    }
  }


  if (currentState == State::TURN_LEFT) {
    moveForwardWithAcceleration(MAX_SPEED, 1);
    
    analogWrite(ENA, TURN_SPEED);
    analogWrite(ENB, TURN_SPEED);

    
    // Set motor directions
    // Roda KIRI
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    // RODA KANAN
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    if (sensors == "00100" || sensors == "00001") {
      currentState = State::STRAIGHT;
    }
  }

  Serial.println(stateToString(currentState));

  // delay(100);
  // stopEngine();
  // delay(1000);
}


// Function to move forward with acceleration
void moveForwardWithAcceleration(int targetSpeed, int duration) {
  // Set motor directions for forward motion
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Accelerate to the target speed
  accelerate(targetSpeed, duration);
}


// Function to accelerate motors gradually
void accelerate(int targetSpeed, int duration) {
  int currentSpeed = 0;                    // Start at 0
  int stepDelay = duration / targetSpeed;  // Calculate delay between each increment

  // Gradually increase speed
  while (currentSpeed <= targetSpeed) {
    analogWrite(ENA, currentSpeed);  // Set speed for Motor A
    analogWrite(ENB, currentSpeed);  // Set speed for Motor B
    delay(stepDelay);                // Wait for the step
    currentSpeed++;                  // Increment speed
  }
}

void stopEngine() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}