#include <Keypad.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#define ANGLE_SERVO_OPEN 80
#define ANGLE_SERVO_CLOSE 100
Servo myServo;

// Cấu hình LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int buzzerPin = 5;  // Chân cho buzzer

// Cấu hình bàn phím ma trận
const byte ROWS = 4;
const byte COLS = 4;

const int ledPin1 = 4;
const int ledPin2 = 2;
const int ledPin3 = 15;

bool isOpenDoor = false;
char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};
byte rowPins[ROWS] = { 27, 14, 12, 13 };
byte colPins[COLS] = { 26, 25, 33, 32 };
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

#define PASSWORD_LENGTH 8
#define FLAG_ADDRESS PASSWORD_LENGTH  // Địa chỉ EEPROM để lưu flag
#define PASSWORD_FLAG 0xAA            // Giá trị flag thể hiện mật khẩu đã lưu
char password[PASSWORD_LENGTH + 1];
char inputPassword[PASSWORD_LENGTH + 1];
byte passwordIndex = 0;

void setup() {
  myServo.setPeriodHertz(60);         // Tăng tần số PWM lên 60Hz để tối ưu tín hiệu
  myServo.attach(18, 500, 2500);      // Đặt biên độ xung (500-2500us) tương ứng 0°-180°
  EEPROM.begin(PASSWORD_LENGTH + 1);  // Khởi tạo EEPROM với kích thước phù hợp
  lcd.init();
  lcd.backlight();
  pinMode(buzzerPin, OUTPUT);  // Khởi tạo buzzer
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);

  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, LOW);

  if (isPasswordStored()) {
    lcd.setCursor(0, 0);
    lcd.print("Input Pass:");
    checkPassword();  // Gọi hàm kiểm tra mật khẩu
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Input New Pass:");
    lcd.setCursor(0, 1);
    lcd.print("                ");
  }
}

void loop() {
  char key = keypad.getKey();

  // Xử lý khi phím '#' được nhấn
  if (key == '#') {
    if (isPasswordStored()) {
      deletePasswordWithVerification();  // Gọi hàm xóa mật khẩu với xác thực
    } else {
      lcd.setCursor(0, 1);
      lcd.print("Chua co mat khau");
      delay(2000);
      lcd.clear();
    }
    //return;  // Kết thúc xử lý cho phím '#'
  }

  // Xử lý khi phím '*' được nhấn
  if (key == '*') {
    if (isPasswordStored()) {
      if (isOpenDoor) {
        lcd.clear();
        lcd.setCursor(0, 0);
        CloseDoor();  // Đóng cửa
        lcd.print("Close Door");
        delay(2000);
        isOpenDoor = false;  // Đặt lại cờ sau khi đóng cửa
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Input Pass:");
        checkPassword();  // Gọi hàm kiểm tra mật khẩu

      } else {
        // lcd.setCursor(0, 1);
        // lcd.print("Door already closed");
        // delay(2000);
        //lcd.clear();
      }
    } else {
      lcd.setCursor(0, 1);
      lcd.print("Chua co mat khau");
      delay(2000);
      lcd.clear();
    }
    // return;  // Kết thúc xử lý cho phím '*'
  }

  // Xử lý nhập mật khẩu mới nếu chưa lưu mật khẩu
  if (!isPasswordStored()) {
    handleNewPasswordInput(key);
  }
}


void OpenDoor() {
  int angle;
  isOpenDoor = true;
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, HIGH);
  for (angle = 0; angle <= ANGLE_SERVO_OPEN; angle++) {
    myServo.write(angle);
    delay(5);  // Di chuyển nhanh hơn
  }
}

void CloseDoor() {
  int angle;
  isOpenDoor = false;
  digitalWrite(ledPin2, HIGH);
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin3, LOW);
  for (angle = ANGLE_SERVO_CLOSE; angle >= -45; angle--) {
    myServo.write(angle);
    delay(5);
  }
}

void handleNewPasswordInput(char key) {
  // Kiểm tra nếu không có phím nào được nhấn
  if (key == NO_KEY) {
    return;  // Thoát khỏi hàm nếu không có phím
  }

  // Kiểm tra xem phím nhập có hợp lệ không (chữ cái hoặc chữ số)
  if ((key >= '0' && key <= '9') || (key >= 'A' && key <= 'Z') || (key >= 'a' && key <= 'z')) {
    if (passwordIndex < PASSWORD_LENGTH) {
      digitalWrite(ledPin1, LOW);
      digitalWrite(ledPin2, HIGH);
      digitalWrite(ledPin3, LOW);
      password[passwordIndex] = key;

      // Hiển thị ký tự vừa nhập
      lcd.setCursor(passwordIndex, 1);
      lcd.print(key);
      delay(500);

      // Thay thế bằng dấu *
      lcd.setCursor(passwordIndex, 1);
      lcd.print('*');

      passwordIndex++;
    }

    if (passwordIndex == PASSWORD_LENGTH) {
      password[PASSWORD_LENGTH] = '\0';  // Kết thúc chuỗi mật khẩu
      savePasswordToEEPROM(password);    // Lưu mật khẩu vào EEPROM

      lcd.setCursor(0, 1);
      lcd.print("Saved EEPROM");
      delay(2000);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Password Saved!");
      delay(2000);
      lcd.clear();
    }
  } else {
    // Hiển thị thông báo lỗi nếu nhập ký tự không hợp lệ
    lcd.setCursor(0, 1);
    lcd.print("Invalid Key!");
    digitalWrite(ledPin1, HIGH);  // Bật LED
    delay(100);                   // Chờ 1 giây
    digitalWrite(ledPin1, LOW);   // Tắt LED
    delay(100);                   // Chờ 1 giây
    delay(500);
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

void checkPassword() {
  const byte MAX_ATTEMPTS = 5;
  const unsigned long LOCK_TIME_MS = 10 * 60 * 1000;  // 15 phút tính bằng milliseconds
  byte attempts = MAX_ATTEMPTS;
  byte inputIndex = 0;

  while (true) {
    // Nhập mật khẩu nếu còn lượt thử
    while (attempts > 0) {
      char key = keypad.getKey();

      if (key) {
        // Nhập từng ký tự của mật khẩu
        if (inputIndex < PASSWORD_LENGTH) {
          inputPassword[inputIndex] = key;
          lcd.setCursor(inputIndex, 1);
          lcd.print(key);
          delay(250);

          lcd.setCursor(inputIndex, 1);
          lcd.print('*');  // Hiển thị dấu '*' thay cho ký tự
          inputIndex++;
        }

        // Kiểm tra mật khẩu sau khi nhập đủ độ dài
        if (inputIndex == PASSWORD_LENGTH) {
          inputPassword[PASSWORD_LENGTH] = '\0';  // Kết thúc chuỗi

          if (comparePasswords(inputPassword)) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Pass Correct!");
            buzzerOn();
            OpenDoor();
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Open Door!");
            return;
          } else {
            // Giảm số lần thử còn lại
            attempts--;
            lcd.setCursor(0, 1);
            lcd.print("Wrong! Left: ");
            lcd.print(attempts);
            buzzerOn();
            inputIndex = 0;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Input Pass:");
          }
        }
      }
    }

    // Khóa hệ thống trong 10 phút sau khi nhập sai quá số lần cho phép
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Locked 10 mins!");

    unsigned long lockStart = millis();  // Ghi lại thời điểm bắt đầu khóa

    while (millis() - lockStart < LOCK_TIME_MS) {
      unsigned long remainingTime = LOCK_TIME_MS - (millis() - lockStart);  // Tính thời gian còn lại
      int minutes = remainingTime / 60000;                                  // Chuyển đổi thành phút
      int seconds = (remainingTime % 60000) / 1000;                         // Phần dư là giây

      lcd.setCursor(0, 1);
      lcd.print("Time Left: ");
      lcd.print(minutes);
      lcd.print(":");
      if (seconds < 10) lcd.print("0");  // Hiển thị số giây với 2 chữ số
      lcd.print(seconds);
      lcd.print(" ");
      digitalWrite(ledPin2, LOW);   // Tắt LED
      digitalWrite(ledPin3, LOW);   // Tắt LED
      digitalWrite(ledPin1, HIGH);  // Bật LED
      delay(100);                   // Chờ 1 giây
      digitalWrite(ledPin1, LOW);   // Tắt LED
      delay(200);                   // Chờ 1 giây
    }

    // Sau 10 phút, cho phép nhập lại mật khẩu
    attempts = MAX_ATTEMPTS;  // Reset số lượt thử
    inputIndex = 0;           // Reset vị trí nhập
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Input Pass:");
  }
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
