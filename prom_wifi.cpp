#include <subsystems/prom_wifi.h>

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <pinout.h>

#define WIFI_CHANNEL 11
#define RX_QUEUE_SIZE 30

// IMAGE CONFIG
static const int imageWidth = 240;
static const int imageHeight = 240;
static const int imageSize = imageWidth * imageHeight;
static const int pixelsPerChunk = 700;
static uint16_t imageBuffer[imageSize];

static const char *TAG = "Receiver";
static const uint8_t esp32camAddress[] = {0xD0, 0xEF, 0x76, 0xEF, 0x5B, 0xB8};
static volatile uint8_t updateImage = 0;
static volatile uint8_t TEST_MODE = 0;
static QueueHandle_t rx_queue;
static SemaphoreHandle_t rx_mutex;
static SemaphoreHandle_t display_mutex;

typedef struct
{
  uint8_t data[24 + 700 * 2 + 4]; // 24-ieee header, 700*2-pixels, 4- FCS
  size_t len;
} rx_packet_t;

struct
{
  uint32_t packets_received;
  uint32_t packets_dropped;
} stats;

static void (*display_image)(uint16_t *buffer, uint16_t width, uint16_t height, size_t size) = nullptr;
static void (*state_callback)(uint8_t state_type, int16_t value) = nullptr;

void prom_wifi_setDisplayCallback(void (*callbackfunc)(uint16_t *buffer, uint16_t width, uint16_t height, size_t size))
{
  display_image = callbackfunc;
}

void prom_wifi_setStateCallback(void (*statecallbackfunc)(uint8_t state_type, int16_t value))
{
  state_callback = statecallbackfunc;
}

static void IRAM_ATTR promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
  
  if (type != WIFI_PKT_MGMT&&type != WIFI_PKT_DATA)
    return;
  //used to have the red LED line here and it did light up. it does not light up after MAC address checking

  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
  //if (memcmp(ppkt->payload + 10, esp32camAddress, 6) != 0)// previously used this line to compare MAC
    //return;

  byte match_found = 255;
  for (int i = 0; i < 32; i++) {
      if (memcmp(ppkt->payload + i, esp32camAddress, 6) == 0) {
          match_found = i;
          break;
      }
  }
  if (match_found==255) return;

  digitalWrite(RED_LED_PIN,HIGH);
  rx_packet_t packet;
  packet.len = ppkt->rx_ctrl.sig_len;
  state_callback(PW_CALLBACK_RECEIVED,packet.len);
  if (packet.len == 30 || packet.len == 24 + pixelsPerChunk * 2 + 4)
  {
    memcpy(packet.data, ppkt->payload, packet.len);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    

    if (xQueueSendToBackFromISR(rx_queue, &packet, &xHigherPriorityTaskWoken) != pdTRUE)
    {
      stats.packets_dropped++;
      if (state_callback)
      {
        state_callback(PW_CALLBACK_PACKETSDROPPED, stats.packets_dropped);
      }
    }

    // if sending to the queue caused a higher priority task to unblock, yield immediately

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    /*if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }*/
  }
}

static void rx_task(void *param)
{
  rx_packet_t packet;
  while (true)
  {
    
    if (xQueueReceive(rx_queue, &packet, (TickType_t)5))
    {
      digitalWrite(GREEN_LED_PIN,HIGH);
      stats.packets_received++;
      state_callback(PW_CALLBACK_PACKETSRECEIVED, stats.packets_received);
      const uint8_t *payload = packet.data;
      size_t len = packet.len;

      if (len == 30 && TEST_MODE == 0)
      {
        TEST_MODE = 1;
        uint16_t seq, pixel;
        memcpy(&seq, payload + 22, 2);
        // seq = (seq >> 8) | (seq << 8); // Handle endianness if needed
        memcpy(&pixel, payload + 24, 2);
        state_callback(PW_CALLBACK_TESTMODE, pixel);
      }
      else if (len == 24 + pixelsPerChunk * 2 + 4)
      {
        uint16_t seq;
        memcpy(&seq, payload + 22, 2);
        // seq = (seq >> 8) | (seq << 8); // Handle endianness if needed
        if (seq * pixelsPerChunk >= imageSize)
        {
          display_image(imageBuffer, imageWidth, imageHeight, imageSize);
          continue;
        }
        uint initialIndex = seq * pixelsPerChunk;
        size_t copy_len = pixelsPerChunk;
        if (imageSize - initialIndex < pixelsPerChunk)
        {
          copy_len = imageSize - initialIndex;
        }
        memcpy(&imageBuffer[initialIndex], payload + 24, copy_len * 2);
        if (initialIndex + copy_len >= imageSize)
        {
          updateImage = 1;
          display_image(imageBuffer, imageWidth, imageHeight, imageSize);
          state_callback(PW_CALLBACK_UPDATEIMAGE, 0);
        }
      }
      else
      {

        state_callback(PW_CALLBACK_SIZEMISMATCH, len);
      }
    }
  }
}

void prom_wifi_sniffer_init()
{
  //wifi init
  state_callback(PW_CALLBACK_DEFAULT,1);
  
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
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(78));
  wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_DATA};
  
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous_ctrl_filter(&filter));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
  ESP_LOGI(TAG, "Promiscuous mode enabled");
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);

  rx_queue = xQueueCreate(RX_QUEUE_SIZE, sizeof(rx_packet_t));

  vTaskDelay(pdMS_TO_TICKS(500));
  xTaskCreate(rx_task, "RX Task", 4096, NULL, 1, NULL);
  state_callback(PW_CALLBACK_INIT,1);
  
}
