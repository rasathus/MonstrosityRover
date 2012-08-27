/*
 * Monstrocity Rover
 */

// used to check host is still alive, and trigger dead mans handle.
int heartbeat_max = 400 ; // The number of ticks around the loop before he's declared dead. not sure what to set this to. 
int heartbeat_tick = heartbeat_max ; // if this hits 0, all motors set to 0

// Pin Listing
// Max current and current analogue read pins.
// Max cur should the number of analogue read units corresponding with the max voltage allowable.
int max_cur = 200;
int cur_load = 0;
int front_left_cur = 0;
int rear_left_cur = 1;
int front_right_cur = 2;
int rear_right_cur = 3;

// PWM drive outputs
int front_left_drive = 3;
int rear_left_drive = 9;
int front_right_drive = 10;
int rear_right_drive = 11;

// Avoid pins 5 & 6 for PWM if pos. http://arduino.cc/en/Reference/AnalogWrite

// PWM current value, an int between 0 and 255 used by the drive loop to pulse the motor drivers.
int front_left_pwm_val = 0;
int rear_left_pwm_val = 0;
int front_right_pwm_val = 0;
int rear_right_pwm_val = 0;

// digi drive direction outputs
int front_left_direc = 4;
int rear_left_direc = 5;
int front_right_direc = 6;
int rear_right_direc = 7;

int debug_pin = 2;

const int BUFFER_SIZE = 256;
char rxbuffer[BUFFER_SIZE];
String rx_message ;

char con_buff[3];

void setup()
{
  // Open the serial connection, 9600 baud
  Serial.begin(57600);

  // Initialise our direction pins
  Serial.println("Initialising Direction Pins");
  pinMode(front_left_direc, OUTPUT);
  pinMode(rear_left_direc, OUTPUT);
  pinMode(front_right_direc, OUTPUT);
  pinMode(rear_right_direc, OUTPUT);

  // Set the direction pins to forward.
  Serial.println("Setting Direction Pins Initial State");
  digitalWrite(front_left_direc, HIGH);
  digitalWrite(rear_left_direc, LOW);
  digitalWrite(front_right_direc, HIGH);
  digitalWrite(rear_right_direc, LOW);

  pinMode(debug_pin, OUTPUT);
  digitalWrite(debug_pin, LOW);

  Serial.println("Setup Complete");
}

void serialEvent(){
  // Called whenever data is available in the serial buffer.

  //Serial.println("SerialEvent called");
  // check for heartbeat, and set flag.

  if (Serial.readBytesUntil('\n', rxbuffer, BUFFER_SIZE)) {
    rx_message = String(rxbuffer);
    Serial.println(rx_message);
    Serial.flush();
  } 
  else {
    Serial.println("I think I timed out...");
    heartbeat_tick = 0;
    checkHeartbeat();      
  }

  // Commands to implement
  // KI1100 == Kill all movement, set motors to 0 and update motor pos.
  // LT0000 == Left Track, 0 or 1 representing direction 0 forwards 1 back.  value between 0 and 255 representing pwm val.
  // RT0000 == Right Track, 0 or 1 representing direction 0 forwards 1 back.  value between 0 and 255 representing pwm val.
  // BV0000 == Battery Voltage, triggers an alagoue pin read, and serial.write of value. 
  // KA0000 == Keep Alive, the signal sent for heartbeat reset.

  if (rx_message.substring(0,2) == "KA") {
    // got a heartbeat, resetting our watchdog.
    Serial.println("Got a Herartbeat");
    if ( heartbeat_tick < 1 ) {
      // Reset our debug led
      digitalWrite(debug_pin, HIGH);
    }
    heartbeat_tick = heartbeat_max ;

  } 
  else if (rx_message.substring(0,2) == "KI") {
    heartbeat_tick = 0 ;

  }
  else if (rx_message.substring(0,2) == "LT") {
    Serial.println("Got a left track command");
    // Process the direction element
    if (rx_message.substring(2,3) == "0"){
      digitalWrite(front_left_direc, HIGH);
      digitalWrite(rear_left_direc, LOW);  
    } 
    else {
      // I know its an assumption but im feeling lazy.
      digitalWrite(front_left_direc, LOW);
      digitalWrite(rear_left_direc, HIGH);      
    } 
    // Process the speed element
    Serial.println(rx_message.substring(3,6));
    //Serial.println(rx_message.substring(3,6).toInt());
    front_left_pwm_val = rx_message.substring(3,6).toInt();
    rear_left_pwm_val = rx_message.substring(3,6).toInt();
    
  } 
  else if (rx_message.substring(0,2) == "RT") {
    Serial.println("Got a right track command");
    // Process the direction element
    if (rx_message.substring(2,3) == "0"){
      digitalWrite(front_right_direc, HIGH);
      digitalWrite(rear_right_direc, LOW);
    } 
    else {
      // I know its an assumption but im feeling lazy.
      digitalWrite(front_right_direc, LOW);
      digitalWrite(rear_right_direc, HIGH);
    } 
    // Process the speed element
    Serial.println(rx_message.substring(3,6));
    //Serial.println(rx_message.substring(3,6).toInt());
    front_right_pwm_val = rx_message.substring(3,6).toInt();
    rear_right_pwm_val = rx_message.substring(3,6).toInt();

  } 
  else if (rx_message.substring(0,2) == "BV") {
    Serial.println("Got a Battery voltage request");
    Serial.println("RESP: Command Not Implemented");

  } 
  else {
    Serial.println("Unknown command");
  }
}

int checkMotorCurrent(){
  /* 
   * Check the current readings from each of the analgoue pins, if they are 
   * greater than our max current, set the pwm value to 75% of its current value. 
   */
  cur_load = analogRead(front_left_cur);
  if (cur_load > max_cur) {
    // Motor current is currently higher than allowed, cutting pwm by 75%
    front_left_pwm_val = front_left_pwm_val * 0.75 ;
    heartbeat_tick = 0 ;
    digitalWrite(debug_pin, HIGH);
  } 
  else {
    //Serial.println("Front Left Current - OK!");
    //Serial.println(cur_load);  
  }
  cur_load = analogRead(rear_left_cur);
  if (cur_load > max_cur) {
    // Motor current is currently higher than allowed, cutting pwm by 75%
    rear_left_pwm_val = rear_left_pwm_val * 0.75 ;
    heartbeat_tick = 0 ;
    digitalWrite(debug_pin, HIGH);
  } 
  else {
    //Serial.println("Rear Left Current - OK!");
    //Serial.println(cur_load);
  }
  cur_load = analogRead(front_right_cur);
  if (cur_load > max_cur) {
    // Motor current is currently higher than allowed, cutting pwm by 75%
    front_right_pwm_val = front_right_pwm_val * 0.75 ;
    heartbeat_tick = 0 ;
    digitalWrite(debug_pin, HIGH);
  } 
  else {
    //Serial.println("Front Right Current - OK!");
    //Serial.println(cur_load);
  }
  cur_load = analogRead(rear_right_cur);
  if (cur_load > max_cur) {
    // Motor current is currently higher than allowed, cutting pwm by 75%
    rear_right_pwm_val = rear_right_pwm_val * 0.75 ;
    heartbeat_tick = 0 ;
    digitalWrite(debug_pin, HIGH);
  } 
  else {
    //Serial.println("Rear Right Current - OK!");
    //Serial.println(cur_load);
  }  
}

int setMotors(){
  analogWrite(front_left_drive, front_left_pwm_val);
  //Serial.println("Front Left Current - OK!");
  //Serial.println(front_left_pwm_val);  
  analogWrite(rear_left_drive, rear_left_pwm_val);
  //Serial.println("Rear Left Current - OK!");
  //Serial.println(rear_left_pwm_val);
  analogWrite(front_right_drive, front_right_pwm_val);
  //Serial.println("Front Right Current - OK!");
  //Serial.println(front_right_pwm_val);
  analogWrite(rear_right_drive, rear_right_pwm_val);
  //Serial.println("Rear Right Current - OK!");
  //Serial.println(rear_right_pwm_val);
}

int checkHeartbeat(){
  if ( heartbeat_tick < 1 ) {
    Serial.println("He's dead jim !  Kill the engines");
    front_left_pwm_val = 0;
    rear_left_pwm_val = 0;
    front_right_pwm_val = 0;
    rear_right_pwm_val = 0;
    setMotors();
    digitalWrite(debug_pin, HIGH);
    Serial.println("Coasting to a halt.");
  } 
  else {
    // decrement our heartbeat.
    heartbeat_tick = heartbeat_tick -1 ;
  }
}


void loop()
{
  // Read in each pair of current sensors, and ensure they aren't over their predefined load.
  checkMotorCurrent() ;
  // Check we havent lost host comms.
  checkHeartbeat() ;
  // Update our motor values.
  setMotors() ;
  delay(20);
}

