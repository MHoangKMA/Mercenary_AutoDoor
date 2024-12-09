#include <Keypad.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Cấu hình LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int buzzerPin = 5;  // Chân cho buzzer

// Cấu hình bàn phím ma trận
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {27, 14, 12, 13};
byte colPins[COLS] = {26, 25, 33, 32};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

#define PASSWORD_LENGTH 8
#define FLAG_ADDRESS PASSWORD_LENGTH  // Địa chỉ EEPROM để lưu flag
#define PASSWORD_FLAG 0xAA            // Giá trị flag thể hiện mật khẩu đã lưu
char password[PASSWORD_LENGTH + 1];
char inputPassword[PASSWORD_LENGTH + 1];
byte passwordIndex = 0;

void setup() {
  EEPROM.begin(PASSWORD_LENGTH + 1);  // Khởi tạo EEPROM với kích thước phù hợp
  lcd.init();
  lcd.backlight();
  pinMode(buzzerPin, OUTPUT);  // Khởi tạo buzzer

  if (isPasswordStored()) {
    lcd.setCursor(0, 0);
    lcd.print("Input Pass:");
    checkPassword(); // Gọi hàm kiểm tra mật khẩu
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Input New Pass:");
    lcd.setCursor(0, 1);
    lcd.print("                ");
  }
}

void loop() {
  char key = keypad.getKey();

  if (key) {
    if (key == '#') {
      if (isPasswordStored()) {
        deletePasswordWithVerification();  // Gọi hàm xóa mật khẩu với xác thực
      } else {
        lcd.setCursor(0, 1);
        lcd.print("Chua co mat khau");
        delay(2000);
        lcd.clear();
      }
    } else if (!isPasswordStored()) {
      handleNewPasswordInput(key);
    }
  }
}

// Hàm xử lý khi nhập mật khẩu mới
void handleNewPasswordInput(char key) {
  if (passwordIndex < PASSWORD_LENGTH) {
    password[passwordIndex] = key;

    // Hiển thị số vừa nhập
    lcd.setCursor(passwordIndex, 1);
    lcd.print(key);
    delay(500);

    // Thay thế bằng dấu *
    lcd.setCursor(passwordIndex, 1);
    lcd.print('*');

    passwordIndex++;
  }

  if (passwordIndex == PASSWORD_LENGTH) {
    password[PASSWORD_LENGTH] = '\0';
    savePasswordToEEPROM(password);

    lcd.setCursor(0, 1);
    lcd.print("Saved EEPROM");
    delay(2000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Password Saved!");
    delay(2000);
    lcd.clear();
  }
}

// Hàm lưu mật khẩu vào EEPROM
void savePasswordToEEPROM(char *password) {
  for (byte i = 0; i < PASSWORD_LENGTH; i++) {
    EEPROM.write(i, password[i]);
  }
  EEPROM.write(FLAG_ADDRESS, PASSWORD_FLAG);  // Lưu flag
  EEPROM.commit();
}

// Kiểm tra mật khẩu đã lưu chưa
bool isPasswordStored() {
  byte flag = EEPROM.read(FLAG_ADDRESS);
  return (flag == PASSWORD_FLAG);
}

// Hàm xóa mật khẩu
void resetPassword() {
  passwordIndex = 0;
  for (byte i = 0; i < PASSWORD_LENGTH; i++) {
    EEPROM.write(i, 0xFF);
  }
  EEPROM.write(FLAG_ADDRESS, 0xFF);
  EEPROM.commit();
}

// Kiểm tra mật khẩu
void checkPassword() {
  byte attempts = 5;
  byte inputIndex = 0;

  while (attempts > 0) {
    char key = keypad.getKey();

    if (key) {
      if (inputIndex < PASSWORD_LENGTH) {
        inputPassword[inputIndex] = key;
        lcd.setCursor(inputIndex, 1);
        lcd.print(key);
        delay(500);

        lcd.setCursor(inputIndex, 1);
        lcd.print('*');
        inputIndex++;
      }

      if (inputIndex == PASSWORD_LENGTH) {
        inputPassword[PASSWORD_LENGTH] = '\0';

        if (comparePasswords(inputPassword)) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Pass Correct!");
          buzzerOn();
          delay(2000);
          lcd.clear();
          return;
        } else {
          attempts--;
          lcd.setCursor(0, 1);
          lcd.print("Wrong! Left: ");
          lcd.print(attempts);
          delay(2000);
          buzzerOn();
          inputIndex = 0;
          lcd.clear();
        }
      }
    }
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Locked!");
  while (1);
}

// So sánh mật khẩu
bool comparePasswords(char *input) {
  for (byte i = 0; i < PASSWORD_LENGTH; i++) {
    if (EEPROM.read(i) != input[i]) {
      return false;
    }
  }
  return true;
}

// Kích hoạt buzzer
void buzzerOn() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(100);
    digitalWrite(buzzerPin, LOW);
    delay(100);
  }
}

// Tắt buzzer
void buzzerOff() {
  digitalWrite(buzzerPin, LOW);
}

// Hàm xóa mật khẩu với xác thực
void deletePasswordWithVerification() {
  byte attempts = 5;

  while (attempts > 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Re-Enter Pass:");
    byte inputIndex = 0;

    while (inputIndex < PASSWORD_LENGTH) {
      char key = keypad.getKey();

      if (key) {
        inputPassword[inputIndex] = key;
        lcd.setCursor(inputIndex, 1);
        lcd.print('*');
        inputIndex++;
      }
    }

    inputPassword[PASSWORD_LENGTH] = '\0';

    if (comparePasswords(inputPassword)) {
      resetPassword();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Pass Cleared!");
      delay(2000);
      lcd.clear();
      return;
    } else {
      attempts--;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Wrong! Left: ");
      lcd.print(attempts);
      delay(2000);
    }
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Abort Delete!");
  delay(2000);
}
