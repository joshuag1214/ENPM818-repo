String password = "1234";   // your secret password
String inputString = "";

const int ledPin = LED_BUILTIN;  // built-in LED on pin 13 (or pin 25 on UNO R4 WiFi)

bool unlocked = false;  // flag to remember if password was accepted

void setup() {
  // Start serial communication at 9600 baud
  Serial.begin(9600);

  // Wait for the serial port to connect (needed on some boards)
  while (!Serial) {
    ; // Wait for serial connection
  }
    pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
 delay(2000);
  Serial.println("Setup complete. Starting loop...");


  Serial.println("Enter password:");
}

void loop() {
  // Print a message

  if (Serial.available() > 0) {
    char incomingChar = Serial.read();

    // If user presses Enter, '\n' is sent
    if (incomingChar == '\n') {
      inputString.trim(); // Remove any whitespace or newline

      if (inputString == "1234") {
        Serial.println("Access granted!");
        unlocked = true;
      } else {
        Serial.println("Incorrect password. Try again:");
      }

      inputString = ""; // Reset input for next attempt
    } else {
      inputString += incomingChar; // Add char to string
    }
  }

if(unlocked == true){
  Serial.println("Hello from Arduino UNO R4 WiFi!");
delay(2000);

}

  
  // Wait for 2 seconds (2000 milliseconds)
  delay(2000);
}
