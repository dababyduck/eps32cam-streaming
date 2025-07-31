#include <Arduino.h>
#include <esp_camera.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#define QUEUE_SIZE 30


QueueHandle_t tx_queue;
SemaphoreHandle_t tx_mutex;


// WiFiUDP udp;

// TODO: figure out why esp32cma won't flash and then test the code, maybe also recheck it. then send full-on images
// TODO: finish chunk sending in this code and then also make it in the base, with dispalying of it

// IMAGE CONFIG
const int imageWidth = 240;
const int imageHeight = 240;
const int pixelsPerChunk = 700;
const int imageResolution = imageWidth * imageHeight;
const int imageSize = imageResolution*2;
const int chunkAmount = imageResolution/pixelsPerChunk;
const int imageFPS = 10;

const uint8_t esp32s3Address[] = {0x34, 0x85, 0x18, 0x9b, 0x5c, 0x30};
const uint8_t esp32camAddress[] = {0xD0, 0xEF, 0x76, 0xEF, 0x5B, 0xB8};

#define RED 0xF800
#define GREEN 0x07E0
#define BLUE 0x001F

#define RED_LED_PIN 12
#define GREEN_LED_PIN 13
#define YELLOW_LED_PIN 15

// server definition
const char *ssid = "ESP32S3";
const char *password = "BIGCAMERA864";
const uint16_t portUDP = 1235;
const IPAddress IP(192, 168, 4, 1);
const char *IP_char = "192.168.4.1";

#define USE_STA

#define WIFI_CHANNEL 11

#ifdef USE_STA
  #define WIFI_IF WIFI_IF_STA
  #define WIFI_MODE WIFI_MODE_STA
#else
  #define WIFI_IF WIFI_IF_AP
  #define WIFI_MODE WIFI_MODE_AP
#endif





typedef struct imagepacket
{
  uint8_t identifier;
  uint16_t data[pixelsPerChunk];
} imagepacket;

const int chunkySize = sizeof(imagepacket);

typedef struct {
  //uint8_t framecontrol1;
  //uint8_t framecontrol2;
  uint16_t framecontrol;
  uint16_t duration;
  uint8_t addr1[6];
  uint8_t addr2[6];
  uint8_t addr3[6];
  uint16_t seq_ctrl;
} ieee80211header_t;


uint8_t def_framecontrol1 = 0x00;//0x08;
uint8_t def_framecontrol2 = 0x02;   //default value

uint16_t def_framecontrol = 0x0004;
uint16_t def_duration = 0x0000;       //default value

typedef struct __attribute__((packed)) {
  ieee80211header_t header;
  uint16_t data[pixelsPerChunk];
} wifi_80211_frame_t;

const uint16_t frame_size = sizeof(wifi_80211_frame_t);
const uint8_t addr_null[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

typedef struct {
  ieee80211header_t header;
  uint16_t pixel;
} TEST_80211_frame_t;

const size_t size_TEST_80211_frame = sizeof(TEST_80211_frame_t); 

/*typedef struct
{
  // 802.11 Header (24 bytes)
  uint16_t frame_control;
  uint16_t duration;
  uint8_t addr1[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // Receiver (EVERYONE)
  uint8_t addr2[6]; // Transmitter (ESP32CAM)
  uint8_t addr3[6]; // BSSID (ESP32CAM)
  uint16_t seq_ctrl;
  // Payload (imagepacket)
  imagepacket data;
} wifi_80211_frame_t;*/



/*typedef struct
{
  // 802.11 Header (24 bytes)
  uint16_t frame_control;
  uint16_t duration;
  uint8_t addr1[6]; // Receiver (ESP32S3)
  uint8_t addr2[6]; // Transmitter (ESP32CAM)
  uint8_t addr3[6]; // BSSID (ESP32S3)
  uint16_t seq_ctrl;
  // Payload (imagepacket)
  uint16_t pixel;
} TEST_wifi_80211_frame_t;*/

uint8_t wifi_80211_header[24] = {
  0x08,0x00,  //frame control:DATA type, no QoS, fromDS=0, toDS=0
  0x00,0x00,  //duration
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,  //receiver (everyone)
  0x00,0x00,0x00,0x00,0x00,0x00,  //transmitter (esp32cam)
  0x00,0x00,0x00,0x00,0x00,0x00,  //bssid (esp32cam)
  0x00,0x00   //seq control
};





#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22



void wifi_tx_proc(void *param) {
  wifi_80211_frame_t frame;
  while (true) {
    if (xQueueReceive(tx_queue, &frame, (TickType_t)5) == pdTRUE) {
      //xSemaphoreTake(tx_mutex, portMAX_DELAY);
      digitalWrite(GREEN_LED_PIN,HIGH);
      esp_err_t res = ESP_OK;
      do {
        res = esp_wifi_80211_tx(WIFI_IF, (uint8_t *)&frame, frame_size, false);
        if (res == ESP_ERR_NO_MEM) {
          taskYIELD();
        } else if (res != ESP_OK) {
          Serial.printf("TX error: %d\n", res);
          break;
        }
      } while (res == ESP_ERR_NO_MEM);
      //xSemaphoreGive(tx_mutex);
    }
  }
}



void setup()
{

  // camera init
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;

  config.frame_size = FRAMESIZE_240X240;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;


  //freertos init
  tx_queue = xQueueCreate(QUEUE_SIZE, frame_size);
  //tx_mutex = xSemaphoreCreateMutex();

  xTaskCreate(wifi_tx_proc,"espTX",4096,NULL,1,NULL);

  Serial.begin(115200);
  Serial.println("Started. Now initting esp_wifi...");
  

  //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
  esp_netif_init();
  esp_event_loop_create_default();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_mode(WIFI_MODE);
  esp_wifi_set_channel(WIFI_CHANNEL,WIFI_SECOND_CHAN_NONE);
  //esp_wifi_internal_set_log_level(WIFI_LOG_TAG, WIFI_LOG_NONE);
  esp_wifi_start();
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(78));
  
  

  esp_wifi_config_80211_tx_rate(WIFI_IF, WIFI_PHY_RATE_54M);
  //wifi_promiscuous_filter_t filter = { .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA };
  //esp_wifi_set_promiscuous_filter(&filter);
  //esp_wifi_set_promiscuous_ctrl_filter(&filter);
  //put the rx_cb packet here if needed.
  esp_wifi_set_promiscuous(true);
  Serial.print("MAC: ");
  uint8_t volatile_mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, volatile_mac);

  for (int i = 0; i < 6; i++)
  {
    if (volatile_mac[i] < 0x10)
    {
      Serial.print("0"); // Ensure two-digit hex formatting
    }
    Serial.print(volatile_mac[i], HEX);
    if (i < 5)
      Serial.print(":"); // Don't print colon after last byte
  }
  Serial.println();
  Serial.printf("memcmp:%d\n",memcmp(esp32camAddress,volatile_mac,6));

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN,OUTPUT);
  pinMode(YELLOW_LED_PIN,OUTPUT);



  
  

  
  /*// brand new, raw wifi
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_mode(WIFI_MODE_STA);

  // ESP_ERROR_CHECK(set_wifi_fixed_rate(s_ground2air_config_packet.wifi_rate));
  ESP_ERROR_CHECK(esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE));

  wifi_promiscuous_filter_t filter =
      {
          .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA};
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous_ctrl_filter(&filter));
  // ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(packet_received_cb));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

  wifi_config_t sta_cfg = {};
  strcpy((char *)sta_cfg.sta.ssid, ssid);
  strcpy((char *)sta_cfg.sta.password, password);
  sta_cfg.sta.channel = 6;
  sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

  esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);

  // esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N);
  // esp_wifi_config_11b_rate(WIFI_IF_AP, true);

  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_ERROR_CHECK(esp_wifi_connect());

  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(78));

    */
  Serial.println("WiFi init'ed, starting camera...");

  if (esp_camera_init(&config) != ESP_OK)
  {
    Serial.println("Camera init failed.");
    digitalWrite(RED_LED_PIN, 1);
    while (1)
      ;
  }
  digitalWrite(RED_LED_PIN, 0);
  
  // "Prime" the camera by pulling the first few frames that usually suck.
  
  for (uint8_t i = 0; i < 10; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    esp_camera_fb_return(fb);
    delay(20);
  }
  Serial.println("Camera setup finished, running loop in 500ms");
  delay(500);
  //while (1);
}

void loop()
{
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("CamCapture failed. Trying again...");
    delay(200);
    return;
  }
  uint16_t *image565 = (uint16_t *)fb->buf;

  /*wifi_80211_frame_t* frame = (wifi_80211_frame_t*)malloc(frame_size);
  if (!frame) {
      Serial.println("Failed to allocate memory for frame.");
      esp_camera_fb_return(fb);
      return;
  }*/

  wifi_80211_frame_t frame;
  //imagepacket currentChunk = {0};
  int identifierCounter = 0;
  int currentChunkCounter = 0;

  //frame.header.framecontrol1 = def_framecontrol1;
  //frame.header.framecontrol2 = def_framecontrol2;
  frame.header.framecontrol = def_framecontrol;
  frame.header.duration = 0x00;
  memcpy(frame.header.addr1,addr_null,6);
  memcpy(frame.header.addr2,esp32camAddress,6);
  memcpy(frame.header.addr3,frame.header.addr2,6);

  frame.header.seq_ctrl = 0;                      // Handled by driver if en_sys_seq = true
  for (int i = 0; i < fb->width * fb->height; i++)
  { // parses each pixel
    frame.header.seq_ctrl = identifierCounter;
    frame.data[currentChunkCounter] = image565[i];
    currentChunkCounter++;
    if (currentChunkCounter >= pixelsPerChunk)
    {
      currentChunkCounter = 0;
      identifierCounter++;
      //esp_wifi_80211_tx(WIFI_IF, (uint8_t *)frame, frame_size, false);
      //xSemaphoreTake(tx_mutex, portMAX_DELAY);
      xQueueSend(tx_queue, &frame, (TickType_t)0);
      digitalWrite(YELLOW_LED_PIN,HIGH);
      
      //xSemaphoreGive(tx_mutex);
      vTaskDelay(pdMS_TO_TICKS((1000/imageFPS)/chunkAmount));
    }
  }
  if (currentChunkCounter > 0)
  
  {
    for (int i = currentChunkCounter; i < pixelsPerChunk; i++)
    {
      frame.data[i] = 0x0000; // writing a blank pixel just to prevent errors
    }
    //esp_wifi_80211_tx(WIFI_IF, (uint8_t *)frame, frame_size, false);
    //xSemaphoreTake(tx_mutex, portMAX_DELAY);
    xQueueSend(tx_queue, &frame, (TickType_t)0);
    digitalWrite(YELLOW_LED_PIN,HIGH);
    //xSemaphoreGive(tx_mutex);
    vTaskDelay(pdMS_TO_TICKS((1000/imageFPS)/chunkAmount));
  }
  
  esp_camera_fb_return(fb);
  //free(frame);
  //vTaskDelay(pdMS_TO_TICKS(100));
}
