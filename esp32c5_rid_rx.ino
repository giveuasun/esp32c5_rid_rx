#include <WiFi.h>
#include <esp_wifi.h>

const int ant_switch_pin = 26; // IO26：拉低->板载天线 拉高->外部天线

// 定义 OpenDroneID 的特定 OUI 特征 (根据具体协议版本可能微调)，OpenDroneID 是全球无人机行业通用的官方/国际标准格式
const uint8_t ODID_OUI[] = {0xFA, 0x0B, 0xBC}; 

typedef struct __attribute__((packed)) {
    uint8_t  messageType;      // 0
    uint8_t  protocolVersion;  // 1
    uint8_t  drid;             // 2
    uint8_t  status;           // 3
    uint8_t  reserved1;        // 4
    uint8_t  uasIdLen;         // 5
    char     uasId[20];        // 6
    uint8_t  reserved2[3];    // 26
    uint8_t  lat[4];           // 27
    uint8_t  lon[4];           // 31
    uint8_t  height[2];        // 35
    uint8_t  speed[4];         // 39
    uint8_t  rest[32];         // 41~78
} RidLocationMsg;

int32_t readInt32LE(const uint8_t* d) {
    return (int32_t)(d[0] | (d[1] << 8) | (d[2] << 16) | (d[3] << 24));
}

int16_t readInt16LE(const uint8_t* d) {
    return (int16_t)(d[0] | (d[1] << 8));
}

// 协议解析函数入口，传入纯净的 RID 数据载荷和长度
void parse_rid_protocol(uint8_t *payload, uint16_t length) {
    Serial.printf("捕获到 %d 字节的无人机 RID 数据!\n", length);
    // --- 打印原始 HEX 数据用于找对齐 ---
    Serial.print("RAW Payload HEX: ");
    for(int i = 0; i < length; i++) {
        Serial.printf("%02X ", payload[i]);
    }
    Serial.println();
    // ------------------------------------------
if (length != 79) {
        Serial.print(F("Bad len: "));
        Serial.println(length);
        return;
    }

    Serial.println(F("===== Drone RID ====="));

    Serial.print(F("MsgType: 0x"));
    Serial.println(payload[0], HEX);

    Serial.print(F("Status: "));
    Serial.println(payload[3]);

    // UAS ID - 固定 20 字节
    Serial.print(F("UAS ID: "));
    for (uint8_t i = 0; i < 20; i++) {
        if (payload[6 + i] >= 0x20 && payload[6 + i] < 0x7F)
            Serial.print((char)payload[6 + i]);
    }
    Serial.println();

    // 经纬度
    int32_t latRaw = readInt32LE(&payload[27]);
    int32_t lonRaw = readInt32LE(&payload[31]);

    double lat = latRaw / 1e7;
    double lon = lonRaw / 1e7;

    Serial.print(F("Lat: "));
    Serial.println(lat, 7);

    Serial.print(F("Lon: "));
    Serial.println(lon, 7);

    // 高度（AGL，单位分米或米取决于 flag，通常分米）
    int16_t height = readInt16LE(&payload[35]);
    Serial.print(F("Height: "));
    Serial.println(height);

    // 速度（cm/s 或类似，取决于协议版本）
    int32_t speed = readInt32LE(&payload[39]);
    Serial.print(F("Speed(raw): "));
    Serial.println(speed);

    Serial.println(F("====================="));
}


// 混杂模式底层回调函数，每抓到一个 WiFi 包都会触发
// @param buf
// @param type 当前wifi帧类型
void rx_callback(void *buf, wifi_promiscuous_pkt_type_t type) {
    //Serial.println("已进入rx_callback");

    // 我们只关心管理帧 (Management Frames)，无人机的 RID（远程标识）、Wi-Fi 路由器的 Beacon（信标广播，也就是告诉你这个 Wi-Fi 叫什么名字的包） 都属于这种帧。如果不加这一行，ESP32 抓到周围任何人在刷视频、下文件的数据包都会进入你的解析代码，不仅没有任何用处，还会极大地占用 CPU 算力，甚至导致处理不过来而丢包。加上这一行，就可以在最开始把 90% 以上的无用数据直接扔掉，只精准处理可能包含无人机 RID 广播的管理帧。
    if (type != WIFI_PKT_MGMT) return; 
    /*
    在 Wi-Fi 的世界里，空中的无线电数据包（包/帧）主要分为三大类：
    数据帧（Data Frames）： 占绝大多数，比如你刷视频、下文件传输的具体网页数据。
    控制帧（Control Frames）： 比如 ACK 确认包，用来保证数据有没有收到的硬件握手包。
    管理帧（Management Frames）： 用来维持无线网络运行的广播包。无人机的 RID（远程标识）、Wi-Fi 路由器的 Beacon（信标广播，也就是告诉你这个 Wi-Fi 叫什么名字的包） 都属于这种帧。
    */

    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    uint8_t *frame = pkt->payload;
    uint16_t len = pkt->rx_ctrl.sig_len;

    if (len < 24) return; // 包太短，不是合法的 802.11 帧

    // 1. 检查帧类型：Beacon 帧的 Frame Control 字段低字节为 0x80
    if (frame[0] != 0x80) return; 

    // 2. 略过 802.11 MAC 头 (24字节) 和固定的 Beacon 字段 (12字节：Timestamp, Beacon Interval, Capability Info)
    uint16_t offset = 36;

    // 3. 遍历 Information Elements (IE) 信息元素
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
                
                // 传入解析模块
                parse_rid_protocol(rid_payload, rid_payload_len);
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

void loop() {}
