/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <Keypad.h>            /* Library for handling matrix keypads */
#include <EEPROM.h>            /* Library for reading and writing data to non-volatile memory */
#include <Wire.h>              /* Library for I2C communication */
#include <LiquidCrystal_I2C.h> /* Library for controlling I2C-based LCD displays */
#include <ESP32Servo.h>        /* Library for controlling servo motors with ESP32 */
#include "BluetoothSerial.h"   /* Library for Bluetooth serial communication */

/*******************************************************************************
 * MACRO
 ******************************************************************************/
/* Define the angle for the servo to open the door */
#define ANGLE_SERVO_CLOSE_DOOR_END 86

/* Define the angle for the servo to close the door */
#define ANGLE_SERVO_OPEN_DOOR_START 100

/* Define the length of the password */
#define PASSWORD_LENGTH 8

/* Define the EEPROM address to store the flag */
#define FLAG_ADDRESS PASSWORD_LENGTH

/* Define the flag value used to indicate that a password is saved in EEPROM */
#define PASSWORD_FLAG 0xAA

/* Define the pin connected to the servo */
#define SERVO_PIN 19

/* Define the PWM frequency used to control the servo (60 Hz) */
#define PWM_FREQUENCY 60

/* Define the start of the PWM pulse range (500 microseconds) */
#define PULSE_RANGE_START 500

/* Define the end of the PWM pulse range (2500 microseconds) */
#define PULSE_RANGE_END 2500

/* Define the starting angle of the servo (0 degrees) */
#define ANGLE_SERVO_CLOSE_DOOR_START 5

/* Define the angle for the end position of the servo */
#define ANGLE_START_OPEN_DOOR_END 5

/* Define the maximum number of password attempts */
#define MAX_ATTEMPTS 5

/* Define the lock time in milliseconds (10 minutes) */
#define LOCK_TIME_MS (10 * 60 * 1000)

/* Define the number of milliseconds in a minute */
#define MS_IN_A_MINUTE 60000

/* Define the number of milliseconds in a second */
#define MS_IN_A_SECOND 1000

/* Define GPIO pins for Ultrasonic Sensor 1 */
#define ULTRASONIC1_TRIGGER_PIN 18 /* Trigger pin for Ultrasonic Sensor 1 */

#define ULTRASONIC1_ECHO_PIN 23 /* Echo pin for Ultrasonic Sensor 1 */

/* Define GPIO pins for Ultrasonic Sensor 2 */
#define ULTRASONIC2_TRIGGER_PIN 4 /* Trigger pin for Ultrasonic Sensor 2 */

#define ULTRASONIC2_ECHO_PIN 2 /* Echo pin for Ultrasonic Sensor 2 */

#define DISTANCE_THRESHOLD_MAX 12 /* Maximum distance in cm to detect a person (for door to open) */

#define DISTANCE_THRESHOLD_MIN 0 /* Minimum valid distance in cm to filter out noise or invalid readings */

#define TIMEOUT_DURATION 30000 /* Timeout duration for pulseIn() in microseconds (30ms) */

#define SPEED_OF_SOUND 0.034 /* Speed of sound in cm per microsecond (used for distance calculation) */

#define DOUBLE_DISTANCE 2 /* Divider used to calculate one-way distance (round-trip divided by 2) */

#define BAUDRATE 115200 /* Baudrate transmiter */

#define ROW_LCD 2 /* Row of LCD*/

#define COLUMN_LCD 16 /* Column of LCD*/
/*******************************************************************************
 * Variables
 ******************************************************************************/
bool isOpenDoor = false; /* Flag to indicate the door state: open or closed */
static bool isManualMode = true;
char password[PASSWORD_LENGTH + 1];      /* Stores the predefined password */
char inputPassword[PASSWORD_LENGTH + 1]; /* Stores the user-entered password */
byte passwordIndex = 0;                  /* Tracks the current index for entering the password */
String inputBuffer = "";                 /* Buffer to store the input string from the keypad */

Servo myServo;            /* Object to control the servo motor */
BluetoothSerial SerialBT; /* Object for Bluetooth serial communication */

/* Configuration for the I2C LCD */
LiquidCrystal_I2C lcd(0x27, COLUMN_LCD, ROW_LCD); /* LCD with I2C address 0x27, 16 columns, and 2 rows */

const int buzzerPin = 5; /* Pin assigned to the buzzer */

/* Configuration for the matrix keypad */
const byte ROWS = 4; /* Number of rows in the keypad */
const byte COLS = 4; /* Number of columns in the keypad */

const int ledPin3 = 15; /* Pin assigned to LED 3 */

char keys[ROWS][COLS] = { /* Defines the key mappings for the keypad */
                          { '1', '2', '3', 'A' },
                          { '4', '5', '6', 'B' },
                          { '7', '8', '9', 'C' },
                          { '*', '0', '#', 'D' }
};

byte rowPins[ROWS] = { 27, 14, 12, 13 }; /* GPIO pins connected to the keypad rows */
byte colPins[COLS] = { 26, 25, 33, 32 }; /* GPIO pins connected to the keypad columns */

/* Creates a Keypad object with the defined mappings and pins */
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

/*
 * @name: setup
 * ----------------------------
 * @brief: Init System
 */

void setup() {
  SerialBT.begin("Auto Door Bluetooth");                         /* Initializes Bluetooth with the device name "Auto Door Bluetooth" */
  
  Serial.begin(BAUDRATE);                                          /* Sets up serial communication at a baud rate of 115200 */
  
  myServo.setPeriodHertz(PWM_FREQUENCY);                         /* Increases PWM frequency to 60Hz for better signal */
  myServo.attach(SERVO_PIN, PULSE_RANGE_START, PULSE_RANGE_END); /* Attaches the servo to pin 18 with pulse range 500-2500 microseconds */

  pinMode(ULTRASONIC1_TRIGGER_PIN, OUTPUT); /* Set Sensor 1 trigger pin as output */
  pinMode(ULTRASONIC1_ECHO_PIN, INPUT);     /* Set Sensor 1 echo pin as input */

  pinMode(ULTRASONIC2_TRIGGER_PIN, OUTPUT); /* Set Sensor 2 trigger pin as output */
  pinMode(ULTRASONIC2_ECHO_PIN, INPUT);     /* Set Sensor 2 echo pin as input */

  EEPROM.begin(PASSWORD_LENGTH + 1); /* Initializes EEPROM with a size sufficient to store the password */
  lcd.init();                        /* Initializes the LCD */
  lcd.backlight();                   /* Turns on the LCD backlight */

  pinMode(buzzerPin, OUTPUT); /* Sets the buzzer pin as an output */
  pinMode(ledPin3, OUTPUT);   /* Sets LED 3 pin as an output */

  digitalWrite(ledPin3, LOW); /* Turns off LED 3 */

  if (isPasswordStored()) {
    lcd.setCursor(0, 0);      /* Sets the cursor to the first row, first column */
    lcd.print("Input Pass:"); /* Prompts the user to input the password */
    checkPassword();          /* Calls the function to verify the password */
  } else {
    lcd.setCursor(0, 0);           /* Sets the cursor to the first row, first column */
    lcd.print("Input New Pass:");  /* Prompts the user to set a new password */
    lcd.setCursor(0, 1);           /* Sets the cursor to the second row, first column */
    lcd.print("                "); /* Clears the second row of the display */
  }
}

/*
 * @name: LockByApp
 * ----------------------------
 * @brief: Lock Door through App on Mobile Phone via BLE
 */

void LockByApp() {
  if (SerialBT.available()) {
    String DataBLE = ""; /* Initialize an empty string to store the data received from the Bluetooth app */
    while (SerialBT.available()) {
      char characterBLE = SerialBT.read(); /* Read each character from Bluetooth */
      DataBLE += characterBLE;             /* Append the character to the string */
    }

    Serial.println("Data from App: "); /* Print a message indicating data has been received from the app */
    Serial.println(DataBLE);           /* Print the data received from the app */

    /* Check if the received command is "*" */
    if (DataBLE.equals("*")) {
      lcd.clear();                 /* Clear the LCD screen */
      lcd.setCursor(0, 0);         /* Set the cursor to the first row, first column */
      lcd.print("App Close Door"); /* Display a message indicating the door is closing */
      CloseDoor();                 /* Call the function to close the door */
      delay(2000);                 /* Wait for 2 seconds */
      isOpenDoor = false;          /* Reset the flag indicating the door is closed */
      lcd.clear();                 /* Clear the LCD screen */
      lcd.setCursor(0, 0);         /* Set the cursor to the first row, first column */
      lcd.print("Input Pass:");    /* Prompt the user to input the password */
      checkPassword();             /* Call the function to verify the password */
    }
  }
}

/*
 * @name: readDistance
 * -------------------
 * @brief: Measures distance using an SRF04 ultrasonic sensor.
 * It triggers a 10 microsecond pulse, listens for the echo,
 * and calculates the distance based on the time of flight.
 *
 * @param triggerPin: GPIO pin used to send the trigger pulse.
 * @param echoPin: GPIO pin used to receive the echo signal.
 *
 * @return: Distance in centimeters, or -1 if no echo is detected (out of range).
 */
long readDistance(int triggerPin, int echoPin) {
  /* Ensure the trigger pin is LOW before starting */
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2); /* Wait for 2 microseconds for stability */

  /* Send a 10 microsecond HIGH pulse to trigger the sensor */
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);         /* Keep the pulse HIGH for 10 microseconds */
  digitalWrite(triggerPin, LOW); /* Set the trigger pin LOW again */

  /* Measure the duration of the HIGH pulse on the echo pin */
  long duration = pulseIn(echoPin, HIGH, TIMEOUT_DURATION); /* Timeout after 30ms if no echo */

  /* If no pulse is received, return -1 to indicate an error */
  if (duration == 0) return -1;

  /* Calculate distance in centimeters: (duration * speed of sound (cm/us)) / 2 */
  long distance = duration * SPEED_OF_SOUND / DOUBLE_DISTANCE;

  /* Return the calculated distance */
  return distance;
}

/*
 * @name: Loop
 * ----------------------------
 * @brief: A Loop run all action on program
 */

void loop() {
  LockByApp();                /* Check if a command is received from the app to lock/unlock the door */
  char key = keypad.getKey(); /* Get the key pressed on the keypad */

  /* If no password is stored yet, handle new password input */
  if (!isPasswordStored()) {
    handleNewPasswordInput(key); /* Handle the input for setting a new password */
  } else {
    if (key == 'B') {
      /* Check if a password has been stored */
      if (isPasswordStored()) {

        /* If the door is currently open, close it first */
        if (isOpenDoor) {
          lcd.clear();             /* Clear the LCD display */
          lcd.setCursor(0, 0);     /* Set cursor to the beginning of the first line */
          CloseDoor();             /* Call function to close the door */
          lcd.print("Close Door"); /* Display "Close Door" message */
          delay(1500);             /* Wait for 2 seconds */
          isOpenDoor = false;      /* Update the flag to indicate the door is now closed */
        }

        /* Switch to Manual mode */
        lcd.clear();                /* Clear the LCD display */
        lcd.setCursor(0, 0);        /* Set cursor to the beginning of the first line */
        lcd.print("Switch Manual"); /* Display message indicating mode switch */
        delay(1500);                /* Wait for 2 seconds */
        isManualMode = true;        /* Enable manual mode by setting flag */

        lcd.clear();              /* Clear the LCD display */
        lcd.setCursor(0, 0);      /* Set cursor to the beginning of the first line */
        lcd.print("Manual Mode"); /* Display "Manual Mode" message */
        delay(1500);              /* Wait for 2 seconds */

        lcd.clear();              /* Clear the LCD display */
        lcd.setCursor(0, 0);      /* Set cursor to the beginning of the first line */
        lcd.print("Input Pass:"); /* Prompt the user to input password */
        checkPassword();          /* Call function to verify password */
      }
    }

    if (isManualMode == true) {
      /* If the "#" key is pressed */
      if (key == '#') {
        if (isPasswordStored()) /* Check if a password is already stored */
        {
          if (isOpenDoor) /* If the door is open */
          {
            lcd.clear();                        /* Clear the LCD screen */
            lcd.setCursor(0, 0);                /* Set cursor to the first row, first column */
            resetPassword();                    /* Reset the stored password */
            lcd.print("RESET PASS");            /* Display the reset message */
            SerialBT.print("PASSWORD RESETED"); /* Send a message to the app about password reset */
            delay(2000);                        /* Wait for 2 seconds */
            ESP.restart();                      /* Restart the ESP32 */
          }
        }
      }

      /* If the "*" key is pressed */
      if (key == '*') {
        if (isPasswordStored()) /* Check if a password is stored */
        {
          if (isOpenDoor) /* If the door is open */
          {
            lcd.clear();              /* Clear the LCD screen */
            lcd.setCursor(0, 0);      /* Set cursor to the first row, first column */
            CloseDoor();              /* Close the door */
            lcd.print("Close Door");  /* Display the message */
            delay(1000);              /* Wait for 2 seconds */
            isOpenDoor = false;       /* Set the flag to false after closing the door */
            lcd.clear();              /* Clear the LCD screen */
            lcd.setCursor(0, 0);      /* Set cursor to the first row, first column */
            lcd.print("Input Pass:"); /* Prompt the user to input the password */
            checkPassword();          /* Verify the entered password */
          }
        }
      }

      /* If the "*" key is pressed */
      if (key == 'A') {
        if (isPasswordStored()) /* Check if a password is stored */
        {
          if (isOpenDoor) /* If the door is open */
          {
            lcd.clear();                /* Clear the LCD screen */
            lcd.setCursor(0, 0);        /* Set cursor to the first row, first column */
            CloseDoor();                /* Close the door */
            lcd.print("Switch Auto");   /* Display the message */
            delay(2000);                /* Wait for 2 seconds */
            isManualMode = false;       /* Set the flag to false after closing the door */
            lcd.clear();                /* Clear the LCD screen */
            lcd.setCursor(0, 0);        /* Set cursor to the first row, first column */
            lcd.print("AutoDoor Mode"); /* Prompt the user to input the password */
          }
        }
      }
    }

    if (isManualMode == false) {

      /* Read distance from Sensor 1 */
      long distance1 = readDistance(ULTRASONIC1_TRIGGER_PIN, ULTRASONIC1_ECHO_PIN);

      /* Read distance from Sensor 2 */
      long distance2 = readDistance(ULTRASONIC2_TRIGGER_PIN, ULTRASONIC2_ECHO_PIN);

      /* Print Sensor 1 value to Serial Monitor */
      Serial.print("Sensor 1: ");
      if (distance1 == -1)
        Serial.print("Out of range"); /* If no echo received, sensor is out of range */
      else
        Serial.print(distance1); /* Print measured distance */

      /* Print Sensor 2 value to Serial Monitor */
      Serial.print(" cm | Sensor 2: ");
      if (distance2 == -1)
        Serial.println("Out of range"); /* If no echo received, sensor is out of range */
      else
        Serial.println(distance2); /* Print measured distance */

      /* Condition to open door: person detected within DISTANCE_THRESHOLD_MAX cm from any sensor */
      if ((distance1 > DISTANCE_THRESHOLD_MIN && distance1 < DISTANCE_THRESHOLD_MAX) || (distance2 > DISTANCE_THRESHOLD_MIN && distance2 < DISTANCE_THRESHOLD_MAX)) {
        if (!isOpenDoor) { /* Only open the door if it is currently closed */
          OpenDoor();      /* Call function to open the door */
        }
      }
      /* Condition to close door: no person detected within 12 cm of both sensors */
      else {
        if (isOpenDoor) { /* Only close the door if it is currently open */
          CloseDoor();    /* Call function to close the door */
        }
      }
      delay(500); /* Delay for sensor stability and to avoid rapid switching */
    }
  }
}

/*
 * @name: OpenDoor
 * ----------------------------
 * @brief: This function opens the door using a servo motor.
 *         It updates the LED status, sets a flag, rotates the servo to the open position,
 *         and sends a message via Bluetooth to indicate the door is opened.
 */
void OpenDoor() {
  int angle;         /* Declare a variable to hold the current servo angle */
  isOpenDoor = true; /* Set the flag to indicate that the door is open */

  digitalWrite(ledPin3, HIGH); /* Turn on LED3 to indicate the door is open */

  /* Loop from start open door angle to end open door angle */
  for (angle = ANGLE_SERVO_OPEN_DOOR_START; angle >= ANGLE_START_OPEN_DOOR_END; angle--) {
    myServo.write(angle); /* Rotate the servo to the specified angle */
    delay(5);             /* Delay to allow smooth servo movement */
  }

  SerialBT.print("OPENED DOOR"); /* Send message via Bluetooth to indicate door is opened */
}

/*
 * @name: CloseDoor
 * ----------------------------
 * @brief: This function closes the door using a servo motor.
 *         It updates the LED status, resets the open flag, rotates the servo to the closed position,
 *         and sends a message via Bluetooth to indicate the door is closed.
 */
void CloseDoor() {
  int angle;          /* Declare a variable to hold the current servo angle */
  isOpenDoor = false; /* Set the flag to indicate that the door is closed */

  digitalWrite(ledPin3, LOW); /* Turn off LED3 (open indicator) */

  /* Loop from start close door angle to end close door angle */
  for (angle = ANGLE_SERVO_CLOSE_DOOR_START; angle <= ANGLE_SERVO_CLOSE_DOOR_END; angle++) { /* Loop from start angle to close angle */
    myServo.write(angle);                                                                    /* Rotate the servo to the specified angle */
    delay(5);                                                                                /* Delay to allow smooth servo movement */
  }

  SerialBT.print("CLOSED DOOR"); /* Send message via Bluetooth to indicate door is closed */
}

/*
 * @name: handleNewPasswordInput
 * ----------------------------
 * @brief: This function handles the input of a new password. It checks if the key pressed is valid (a number or letter),
 * appends it to the password array, and displays it on the LCD screen. The entered password is saved to EEPROM once
 * the defined length is reached. If an invalid key is pressed, it displays an error message on the LCD and blinks an LED.
 */
void handleNewPasswordInput(char key) {
  /* Check if no key was pressed */
  if (key == NO_KEY) {
    return; /* Exit the function if no key is pressed */
  }

  /* Check if the pressed key is valid (number or letter) */
  if ((key >= '0' && key <= '9') || (key >= 'A' && key <= 'Z') || (key >= 'a' && key <= 'z')) {
    if (passwordIndex < PASSWORD_LENGTH) {
      /* Turn on LED to indicate password entry is in progress */
      digitalWrite(ledPin3, HIGH);

      password[passwordIndex] = key;

      /* Display the entered character on the LCD screen */
      lcd.setCursor(passwordIndex, 1);
      lcd.print(key);
      delay(500);

      /* Replace the displayed character with '*' */
      lcd.setCursor(passwordIndex, 1);
      lcd.print('*');

      passwordIndex++; /* Move to the next character position */
    }

    /* If the password length is reached, save it to EEPROM */
    if (passwordIndex == PASSWORD_LENGTH) {
      password[PASSWORD_LENGTH] = '\0'; /* Null-terminate the password string */
      savePasswordToEEPROM(password);   /* Save the password to EEPROM */

      lcd.setCursor(0, 1);
      lcd.print("Saved EEPROM");
      delay(2000);

      /* Clear the display and show a success message */
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Password Saved!");
      delay(2000);
      ESP.restart(); /* Restart the system to apply the new password */
    }
  } else {
    /* Display an error message if the entered key is invalid */
    lcd.setCursor(0, 1);
    lcd.print("Invalid Key!");
    digitalWrite(ledPin3, HIGH); /* Blink the LED to indicate error */
    delay(100);                  /* Wait 1 second */
    digitalWrite(ledPin3, LOW);  /* Turn off the LED */
    delay(100);                  /* Wait 1 second */
    delay(500);
    lcd.clear();
  }
}

/*
 * @name: savePasswordToEEPROM
 * ----------------------------
 * @brief: This function saves the entered password to the EEPROM memory. It loops through the password characters
 * and writes each one to the EEPROM. After saving the password, it stores a flag to indicate that a password has been set.
 */
void savePasswordToEEPROM(char *password) {
  /* Loop through each character in the password and save it to EEPROM */
  for (byte i = 0; i < PASSWORD_LENGTH; i++) {
    EEPROM.write(i, password[i]);
  }

  /* Store a flag to indicate that a password is saved */
  EEPROM.write(FLAG_ADDRESS, PASSWORD_FLAG);
  EEPROM.commit(); /* Commit the changes to EEPROM */
}

/*
 * @name: isPasswordStored
 * ----------------------------
 * @brief: This function checks if a password has been stored in EEPROM. It reads the flag from the EEPROM and compares it
 * with the predefined PASSWORD_FLAG. If the flag matches, it returns true, indicating that a password has been saved.
 */
bool isPasswordStored() {
  /* Read the flag from EEPROM to check if a password is stored */
  byte flag = EEPROM.read(FLAG_ADDRESS);

  /* Return true if the flag matches PASSWORD_FLAG, indicating the presence of a stored password */
  return (flag == PASSWORD_FLAG);
}

/*
 * @name: resetPassword
 * ----------------------------
 * @brief: This function resets the stored password by clearing the password data from EEPROM. It sets the password flag to 0xFF
 * to indicate that no password is stored. After clearing the data, it commits the changes to the EEPROM.
 */
void resetPassword() {
  /* Reset the password index */
  passwordIndex = 0;

  /* Clear the stored password data from EEPROM */
  for (byte i = 0; i < PASSWORD_LENGTH; i++) {
    EEPROM.write(i, 0xFF);
  }

  /* Reset the flag indicating no password is stored */
  EEPROM.write(FLAG_ADDRESS, 0xFF);

  /* Commit the changes to EEPROM */
  EEPROM.commit();
}
/*
 * Function to check the entered password
 * ----------------------------
 * @brief: This function verifies if the entered password is correct. The user has a limited number of attempts (5 attempts).
 * If the user exceeds the allowed number of attempts, the system will be locked for 10 minutes.
 * Password can be entered via both keypad and Bluetooth. If the correct password is entered, the door will open.
 */
void checkPassword() {
  /* Define the maximum number of attempts and lock time in milliseconds */
  const byte maxAttempts = MAX_ATTEMPTS;
  const unsigned long lockTimeMs = LOCK_TIME_MS; /* Lock time of 10 minutes in milliseconds */
  byte attempts = MAX_ATTEMPTS;
  byte inputIndex = 0;

  /* Infinite loop to keep checking password input */
  while (true) {
    /* Continue attempting to enter the password if there are remaining attempts */
    while (attempts > 0) {
      char key = '\0';

      /* Check data from the keypad */
      char keypadKey = keypad.getKey();
      if (keypadKey) {
        key = keypadKey;
      }

      /* Check data from Bluetooth */
      if (SerialBT.available()) {
        String bluetoothData = "";
        while (SerialBT.available()) {
          char bluetoothChar = SerialBT.read();
          bluetoothData += bluetoothChar;
        }

        /* Process Bluetooth password if fully received */
        if (bluetoothData.length() == PASSWORD_LENGTH) {
          bluetoothData.toCharArray(inputPassword, PASSWORD_LENGTH + 1);
          if (comparePasswords(inputPassword)) /* Check if the entered password is correct */
          {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Pass Correct!");
            buzzerOn(); /* Activate buzzer */
            OpenDoor(); /* Open the door */
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Open Door!");
            return;
          } else {
            /* Decrease attempts left and update display */
            attempts--;
            lcd.setCursor(0, 1);
            lcd.print("Wrong! Left: ");
            lcd.print(attempts);
            SerialBT.print("Wrong! Left: ");
            SerialBT.print(attempts);
            buzzerOn(); /* Activate buzzer */
            inputIndex = 0;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Input Pass:");
          }
          continue; /* Skip the keypad processing if Bluetooth input is already processed */
        }
      }

      /* Process key input from the keypad */
      if (key) {
        /* Add key to input password */
        if (inputIndex < PASSWORD_LENGTH) {
          inputPassword[inputIndex] = key;
          lcd.setCursor(inputIndex, 1);
          lcd.print(key);
          delay(350);

          lcd.setCursor(inputIndex, 1);
          lcd.print('*'); /* Display '*' instead of actual character */
          inputIndex++;
        }

        /* Check if the input password is fully entered */
        if (inputIndex == PASSWORD_LENGTH) {
          inputPassword[PASSWORD_LENGTH] = '\0'; /* Null-terminate the password string */

          if (comparePasswords(inputPassword)) /* Check if password is correct */
          {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Pass Correct!");
            buzzerOn(); /* Activate buzzer */
            OpenDoor(); /* Open the door */
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Open Door!");
            return;
          } else {
            /* Decrease attempts left and update display */
            attempts--;
            lcd.setCursor(0, 1);
            lcd.print("Wrong! Left: ");
            lcd.print(attempts);
            buzzerOn(); /* Activate buzzer */
            inputIndex = 0;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Input Pass:");
          }
        }
      }
    }

    /* Lock the system for 10 minutes after exceeding the maximum number of attempts */
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Locked 10 mins!");
    SerialBT.print("Locked 10 mins!");

    unsigned long lockStart = millis(); /* Record the time when the lock starts */

    /* Wait for the lock period to pass (10 minutes) */
    while (millis() - lockStart < LOCK_TIME_MS) {
      unsigned long remainingTime = LOCK_TIME_MS - (millis() - lockStart); /* Calculate remaining time */
      int minutes = remainingTime / MS_IN_A_MINUTE;                        /* Convert to minutes */
      int seconds = (remainingTime % MS_IN_A_MINUTE) / MS_IN_A_SECOND;     /* Convert to seconds */

      lcd.setCursor(0, 1);
      lcd.print("Time Left: ");
      lcd.print(minutes);
      lcd.print(":");
      if (seconds < 10)
        lcd.print("0"); /* Display seconds with leading zero */
      lcd.print(seconds);
      lcd.print(" ");

      /* Flash LEDs to indicate locked status */
      digitalWrite(ledPin3, HIGH); /* Turn on LED 3 */
      delay(100);                  /* Wait for 100 ms */
      digitalWrite(ledPin3, LOW);  /* Turn off LED 1 3*/
      delay(200);                  /* Wait for 200 ms */
    }

    /* Reset attempts and input index after lock period */
    attempts = MAX_ATTEMPTS; /* Reset maximum attempts */
    inputIndex = 0;          /* Reset input index */
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Input Pass:");
  }
}

/*
 * Function to compare the input password with the stored password in EEPROM
 * ----------------------------
 * @brief: This function checks if the entered password matches the password stored in EEPROM.
 * If all characters match, the function returns true. Otherwise, it returns false.
 */
bool comparePasswords(char *input) {
  /* Loop through each character of the input password and compare it with the stored password in EEPROM */
  for (byte i = 0; i < PASSWORD_LENGTH; i++) {
    /* If any character doesn't match, return false */
    if (EEPROM.read(i) != input[i]) {
      return false;
    }
  }

  /* If all characters match, return true */
  return true;
}

/*
 * Function to turn the buzzer on for a short duration
 * ----------------------------
 * @brief: This function activates the buzzer for 3 short beeps.
 * It turns the buzzer on and off with a delay of 100ms for each beep.
 */
void buzzerOn() {
  /* Repeat 3 times to create 3 beeps */
  for (int i = 0; i < 3; i++) {
    /* Turn the buzzer on */
    digitalWrite(buzzerPin, HIGH);
    delay(100); /* Wait for 100ms */

    /* Turn the buzzer off */
    digitalWrite(buzzerPin, LOW);
    delay(100); /* Wait for 100ms */
  }
}

/*
 * Function to turn off the buzzer
 * ----------------------------
 * @brief: This function turns off the buzzer by setting its pin to LOW.
 */
void buzzerOff() {
  /* Turn off the buzzer */
  digitalWrite(buzzerPin, LOW);
}
/*******************************************************************************
 * EOF
 ******************************************************************************/