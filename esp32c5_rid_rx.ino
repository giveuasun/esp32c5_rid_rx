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

/*
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
*/

#include <WiFi.h>
#include <esp_wifi.h>

// 定义 OpenDroneID 的特定 OUI 特征 (根据具体协议版本可能微调)
const uint8_t ODID_OUI[] = {0xFA, 0x0B, 0xBC}; 

// 你的协议解析函数入口，传入纯净的 RID 数据载荷和长度
void parse_your_rid_protocol(uint8_t *payload, uint16_t length) {
    // 在这里接入你的解析代码！
    // 比如：解析 Location Message 获取经度 (Latitude)、纬度 (Longitude) 和高度 (Altitude)
    Serial.printf("捕获到 %d 字节的无人机 RID 数据!\n", length);
    /* 
       例子：
       if (payload[0] == 0x10) { // 假设 0x10 代表 Location Message
           // bit-shifting 还原经纬度...
       }
    */
}

// 混杂模式底层回调函数，每抓到一个 WiFi 包都会触发
void rx_callback(void *buf, wifi_promiscuous_pkt_type_t type) {
    //Serial.println("已进入rx_callback");
    // 我们只关心管理帧 (Management Frames)，其中包含 Beacon 帧
    if (type != WIFI_PKT_MGMT) return; 

    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    uint8_t *frame = pkt->payload;
    uint16_t len = pkt->rx_ctrl.sig_len;

    if (len < 24) return; // 包太短，不是合法的 802.11 帧

    // 1. 检查帧类型：Beacon 帧的 Frame Control 字段低字节为 0x80
    if (frame[0] != 0x80) return; 

    // 2. 略过 802.11 MAC 头 (24字节) 和固定的 Beacon 字段 (12字节：Timestamp, Beacon Interval, Capability Info)
    uint16_t offset = 36;

    // 3. 遍历 Information Elements (IE)
    while (offset < len) {
        uint8_t tag_id = frame[offset];
        uint8_t tag_length = frame[offset + 1];

        // 检查是否到达帧尾或长度异常
        if (offset + 2 + tag_length > len) break; 

        // 寻找 Vendor Specific 标签 (ID = 221 / 0xDD)
        if (tag_id == 0xDD) {
            uint8_t *ie_data = &frame[offset + 2];
            
            // 检查前 3 字节的 OUI 是否匹配 OpenDroneID 协议
            if (tag_length >= 3 && 
                ie_data[0] == ODID_OUI[0] && 
                ie_data[1] == ODID_OUI[1] && 
                ie_data[2] == ODID_OUI[2]) {
                
                // 找到了无人机 RID 突发！
                // ie_data + 4 (跳过 OUI 和 Type) 即为纯净的 RID 数据流（Message Pack）
                uint8_t rid_payload_len = tag_length - 4;
                uint8_t *rid_payload = &ie_data[4];
                
                // 传入你的解析模块
                parse_your_rid_protocol(rid_payload, rid_payload_len);
            }
        }
        
        // 移动到下一个 IE 标签
        offset += (2 + tag_length); 
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // 控制天线
    pinMode(ant_switch_pin, OUTPUT); 
    digitalWrite(ant_switch_pin, LOW);
    Serial.println("已切换至板载天线 (LOW)");

    Serial.println("启动 ESP32-C5 无人机 RID 接收机...");
    // 将 WiFi 设置为 Station 模式，并断开连接，准备进入混杂模式
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // 初始化并开启混杂模式
    esp_wifi_set_promiscuous(true);
    int esp_check = esp_wifi_set_promiscuous_rx_cb(&rx_callback);
    Serial.print("[esp_wifi_set_promiscuous_rx_cb]回调函数注入结果（0：成功）：");Serial.println(esp_check);

    // 无人机通常在 2.4GHz 的信道 1, 6, 11 广播 RID
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);//只监听6信道：2.437GHz
}

void loop() {
    /*
    // 信道跳跃扫描 (Channel Hopping)
    // 监听特定信道 200ms 后切换下一个信道，防止漏掉其他信道的广播
    static int current_channel = 1;
    const int channels[] = {1, 6, 11}; // 重点扫描这三个信道
    
    esp_wifi_set_channel(channels[current_channel], WIFI_SECOND_CHAN_NONE);
    current_channel = (current_channel + 1) % 3;
    */
    
    delay(250); 
}
