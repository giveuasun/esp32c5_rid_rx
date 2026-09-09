const int ant_switch_pin = 26; // IO26：拉低->板载天线 拉高->外部天线
/*
void setup() {
  Serial.begin(921600);
  delay(1000);
  
  // 设置为输出模式来控制天线
  pinMode(ant_switch_pin, OUTPUT); 

  // 输出低电平，选择板载天线
  digitalWrite(ant_switch_pin, LOW);
  Serial.println("已切换至板载天线 (LOW)");
}

void loop() {
  Serial.println("LOOP");//serial：串口，println相较于print多一个换行
  int pinState = digitalRead(ant_switch_pin);
  Serial.print("GPIO 26 State: ");
  Serial.println(pinState); // 1 代表高电平(HIGH)，0 代表低电平(LOW)
  delay(1000);
}
*/

#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  // 设置为输出模式来控制天线
  pinMode(ant_switch_pin, OUTPUT); 

  // 输出低电平，选择板载天线
  digitalWrite(ant_switch_pin, LOW);
  Serial.println("已切换至板载天线 (LOW)");

  // 设置为 STA（站点）模式并断开之前的连接
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.println("Wi-Fi 扫描初始化完成");
}

void loop() {
  Serial.println("正在扫描周围的 Wi-Fi 网络...");

  // 开始扫描周围的网络数量
  int n = WiFi.scanNetworks();
  
  if (n == 0) {
    Serial.println("未发现任何 Wi-Fi 网络。");
  } else {
    Serial.print("找到 ");
    Serial.print(n);
    Serial.println(" 个网络：");
    
    for (int i = 0; i < n; ++i) {
      // 打印序号、SSID、信号强度(RSSI)以及是否加密
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));            // Wi-Fi 名称
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));           // 信号强度
      Serial.print(" dBm) 加密方式: ");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "开放" : "加密");
      delay(10);
    }
  }
  
  Serial.println("");
  // 等待 5 秒后进行下一次扫描
  delay(5000); 
}