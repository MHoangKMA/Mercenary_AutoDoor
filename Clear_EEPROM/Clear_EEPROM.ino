#include <EEPROM.h>

void setup() {
  Serial.begin(115200);
  EEPROM.begin(512); // Khởi tạo EEPROM với kích thước 512 byte (tùy thuộc vào board và yêu cầu)

  // Xóa toàn bộ EEPROM
  clearEEPROM();

  Serial.println("EEPROM đã được xóa.");
}

void loop() {
  // Không có gì trong loop
}

void clearEEPROM() {
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0xFF); // Ghi giá trị 0xFF vào mỗi byte trong EEPROM
  }
  EEPROM.commit(); // Lưu thay đổi vào EEPROM
}
