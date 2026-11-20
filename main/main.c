#include "esp_camera.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_psram.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>

#include "camera_pins.h"

static const char *TAG = "ESP32S3_CAM";

// ==============================================================================
// Wi-Fi Configuration (Replace with your actual Wi-Fi credentials)
// ==============================================================================
#define WIFI_SSID "SHRINATHPOL"
#define WIFI_PASSWORD "12345678"
#define MAXIMUM_RETRY 10

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static int s_retry_num = 0;

// HTTP streaming multipart boundary
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ==============================================================================
// Wi-Fi Event Handler
// ==============================================================================
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < MAXIMUM_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "Retrying connection to AP (%d/%d)...", s_retry_num,
               MAXIMUM_RETRY);
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Connected! Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

static esp_err_t wifi_init_sta(void) {
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL,
      &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL,
      &instance_got_ip));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = WIFI_SSID,
              .password = WIFI_PASSWORD,
              .threshold.authmode = WIFI_AUTH_WPA2_PSK,
          },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "Connecting to Wi-Fi SSID: %s ...", WIFI_SSID);

  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "Successfully connected to SSID: %s", WIFI_SSID);
    return ESP_OK;
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGE(TAG, "Failed to connect to SSID: %s", WIFI_SSID);
    return ESP_FAIL;
  }

  return ESP_ERR_TIMEOUT;
}

// ==============================================================================
// Camera Initialization
// ==============================================================================
static esp_err_t init_camera(void) {
  camera_config_t config;
  memset(&config, 0, sizeof(camera_config_t));

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
  config.xclk_freq_hz = 10000000; // 10MHz XCLK for PY260 / MEGA_CCM stability
  config.frame_size = FRAMESIZE_VGA; // 640x480 stable baseline
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST; // Drops stale frames, prevents buffer overflow & NO-SOI
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 1; // PY260: 0=High, 1=Default, 2=Low
  config.fb_count = 4;
  config.jpeg_buffer_size = 512 * 1024; // 512KB frame buffer prevents clipping

  // Check if PSRAM is initialized
  if (esp_psram_is_initialized()) {
    ESP_LOGI(TAG, "PSRAM detected! Setting camera with multi-buffering in PSRAM.");
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 1;
    config.fb_count = 4;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.jpeg_buffer_size = 512 * 1024;
  } else {
    ESP_LOGW(TAG, "No PSRAM detected. Falling back to internal RAM settings (VGA/single-buffer).");
    config.frame_size = FRAMESIZE_VGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.jpeg_quality = 1;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.jpeg_buffer_size = 0;
  }

  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
    return err;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    ESP_LOGI(TAG, "Camera sensor detected (PID: 0x%04x)", s->id.PID);
    if (s->id.PID == 0x039e) {
      // Leave PY260 / MEGA_CCM registers at neutral defaults and capture 1 warmup frame
      vTaskDelay(pdMS_TO_TICKS(300));
      camera_fb_t *fb = esp_camera_fb_get();
      if (fb) {
        ESP_LOGI(TAG, "Warmup frame OK: len=%u SOI=%02X%02X EOI=%02X%02X",
                 (unsigned)fb->len, fb->buf[0], fb->buf[1],
                 fb->len >= 2 ? fb->buf[fb->len - 2] : 0,
                 fb->len >= 2 ? fb->buf[fb->len - 1] : 0);
        esp_camera_fb_return(fb);
      } else {
        ESP_LOGW(TAG, "Warmup frame capture returned NULL");
      }
    } else {
      // Align sensor orientation and picture parameters as per OceanLabz calibration
      if (s->set_vflip) s->set_vflip(s, 1);
      if (s->set_brightness) s->set_brightness(s, 1);
      if (s->set_saturation) s->set_saturation(s, 0);
    }
  }

  ESP_LOGI(TAG, "Camera initialized successfully.");
  return ESP_OK;
}

// ==============================================================================
// HTTP Handlers (Web Interface, Stream, Snapshot, Status)
// ==============================================================================

// Index Page with live preview, controls, and Wi-Fi OTA firmware update
static const char INDEX_HTML[] =
    "<!DOCTYPE html>"
    "<html>"
    "<head>"
    "<meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<title>ESP32-S3 Camera Stream & OTA</title>"
    "<style>"
    "body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', "
    "Roboto, Helvetica, Arial, sans-serif; background: #0f172a; color: "
    "#f8fafc; margin: 0; padding: 20px; display: flex; flex-direction: column; "
    "align-items: center; }"
    ".container { max-width: 900px; width: 100%; display: flex; "
    "flex-direction: column; align-items: center; gap: 16px; }"
    "h1 { margin: 10px 0; font-size: 26px; color: #38bdf8; }"
    ".stream-box { width: 100%; max-width: 800px; background: #1e293b; "
    "border-radius: 12px; overflow: hidden; box-shadow: 0 10px 25px -5px "
    "rgba(0,0,0,0.5); display: flex; justify-content: center; align-items: "
    "center; min-height: 360px; }"
    ".stream-box img { width: 100%; height: auto; display: block; "
    "border-radius: 12px; }"
    ".actions { display: flex; gap: 12px; flex-wrap: wrap; justify-content: "
    "center; }"
    "a.button, button { background: #0284c7; color: white; border: none; "
    "padding: 10px 20px; font-size: 15px; border-radius: 8px; text-decoration: "
    "none; cursor: pointer; transition: background 0.2s; font-weight: 500; }"
    "a.button:hover, button:hover { background: #0369a1; }"
    ".card { background: #1e293b; border-radius: 8px; padding: 16px 20px; "
    "width: 100%; max-width: 760px; font-size: 14px; color: #94a3b8; "
    "line-height: 1.6; }"
    ".card strong { color: #f1f5f9; font-size: 15px; }"
    ".bar-container { background: #0f172a; border-radius: 6px; overflow: hidden; "
    "height: 18px; margin-top: 10px; display: none; }"
    ".bar-fill { background: #10b981; width: 0%; height: 100%; transition: width 0.2s; }"
    "</style>"
    "</head>"
    "<body>"
    "<div class='container'>"
    "  <h1>ESP32-S3 Camera Monitor</h1>"
    "  <div class='stream-box'>"
    "    <img src='' alt='Live Video Stream' id='stream-view' />"
    "  </div>"
    "  <div class='actions'>"
    "    <a class='button' href='/capture' target='_blank'>Capture Full Snapshot</a>"
    "    <button onclick=\"document.getElementById('stream-view').src = getStreamUrl() + '?t=' + new Date().getTime();\">Reload Stream</button>"
    "  </div>"
    "  <div class='card'>"
    "    <strong>Firmware Update (Over-the-Air / Wi-Fi)</strong><br>"
    "    <span>Upload a new <code>esp32s3_camera.bin</code> wirelessly:</span>"
    "    <div style='display:flex; gap:10px; align-items:center; margin-top:10px; flex-wrap:wrap;'>"
    "      <input type='file' id='ota-file' accept='.bin' style='color:#94a3b8;' />"
    "      <button onclick='uploadOTA()'>Upload & Flash</button>"
    "    </div>"
    "    <div class='bar-container' id='bar-box'>"
    "      <div class='bar-fill' id='bar'></div>"
    "    </div>"
    "    <p id='ota-status' style='margin:8px 0 0 0; color:#38bdf8; font-weight:bold;'></p>"
    "  </div>"
    "  <div class='card'>"
    "    <strong>Quick Endpoints:</strong><br>"
    "    &bull; Live MJPEG Stream: <code><a href='#' onclick=\"this.href=getStreamUrl();\" style='color:#38bdf8;'>/stream (:81)</a></code><br>"
    "    &bull; Single JPEG Snapshot: <code><a href='/capture' style='color:#38bdf8;'>/capture</a></code><br>"
    "    &bull; Device Status: <code><a href='/status' style='color:#38bdf8;'>/status</a></code><br>"
    "    &bull; Wireless OTA POST: <code><a href='/update' style='color:#38bdf8;'>/update</a></code>"
    "  </div>"
    "</div>"
    "<script>"
    "function getStreamUrl() {"
    "  return window.location.protocol + '//' + window.location.hostname + ':81/stream';"
    "}"
    "window.addEventListener('DOMContentLoaded', function() {"
    "  document.getElementById('stream-view').src = getStreamUrl();"
    "});"
    "function uploadOTA() {"
    "  var fileInput = document.getElementById('ota-file');"
    "  if (fileInput.files.length === 0) {"
    "    alert('Please choose a .bin firmware file first!');"
    "    return;"
    "  }"
    "  document.getElementById('stream-view').src = '';"
    "  var file = fileInput.files[0];"
    "  var xhr = new XMLHttpRequest();"
    "  var barBox = document.getElementById('bar-box');"
    "  var bar = document.getElementById('bar');"
    "  var status = document.getElementById('ota-status');"
    "  barBox.style.display = 'block';"
    "  bar.style.backgroundColor = '#10b981';"
    "  bar.style.width = '0%';"
    "  status.innerText = 'Starting wireless upload...';"
    "  xhr.upload.onprogress = function(e) {"
    "    if (e.lengthComputable) {"
    "      var pct = Math.round((e.loaded / e.total) * 100);"
    "      bar.style.width = pct + '%';"
    "      status.innerText = 'Uploading: ' + pct + '% (' + Math.round(e.loaded/1024) + ' KB / ' + Math.round(e.total/1024) + ' KB)';"
    "    }"
    "  };"
    "  xhr.onload = function() {"
    "    if (xhr.status === 200) {"
    "      bar.style.width = '100%';"
    "      status.innerHTML = '<strong>' + xhr.responseText + '</strong> Reconnecting in 10s...';"
    "      setTimeout(function() { location.reload(); }, 10000);"
    "    } else {"
    "      bar.style.backgroundColor = '#ef4444';"
    "      status.innerText = 'Update failed: ' + xhr.responseText;"
    "    }"
    "  };"
    "  xhr.onerror = function() {"
    "    bar.style.backgroundColor = '#ef4444';"
    "    status.innerText = 'Upload error or connection lost!';"
    "  };"
    "  xhr.open('POST', '/update', true);"
    "  xhr.send(file);"
    "}"
    "</script>"
    "</body>"
    "</html>";

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// MJPEG Stream Handler
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char part_buf[128];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(pdMS_TO_TICKS(40));
      continue;
    }

    if (fb->format != PIXFORMAT_JPEG) {
      bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
      esp_camera_fb_return(fb);
      fb = NULL;
      if (!jpeg_converted) {
        ESP_LOGE(TAG, "JPEG compression failed");
        res = ESP_FAIL;
        break;
      }
    } else {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }

    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY,
                                  strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen =
          snprintf(part_buf, sizeof(part_buf), _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }

    if (res != ESP_OK) {
      // Client likely closed the connection
      break;
    }
  }

  return res;
}

// Single JPEG capture handler
static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    ESP_LOGE(TAG, "Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition",
                     "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);

  return res;
}

// Device Status JSON handler
static esp_err_t status_handler(httpd_req_t *req) {
  char json_buf[256];
  sensor_t *s = esp_camera_sensor_get();
  int pid = s ? s->id.PID : 0;

  snprintf(
      json_buf, sizeof(json_buf),
      "{\"camera_sensor_pid\":\"0x%04x\",\"free_heap\":%lu,\"free_psram\":%lu}",
      pid, (unsigned long)esp_get_free_heap_size(),
      (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_buf, HTTPD_RESP_USE_STRLEN);
}

// Delayed restart task to allow HTTP response to flush cleanly before reboot
static void delayed_restart_task(void *pvParameter) {
  vTaskDelay(pdMS_TO_TICKS(1500));
  esp_restart();
}

// Wi-Fi OTA Update POST Handler
#define OTA_BUFFSIZE 1024
static esp_err_t ota_update_post_handler(httpd_req_t *req) {
  esp_ota_handle_t ota_handle = 0;
  const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

  if (update_partition == NULL) {
    ESP_LOGE(TAG, "Cannot find next OTA partition! Check partition table.");
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition found");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Starting OTA update -> partition '%s' @ 0x%08lx (size 0x%lx)",
           update_partition->label, update_partition->address, update_partition->size);

  int remaining = req->content_len;
  if (remaining <= 0) {
    ESP_LOGE(TAG, "OTA payload empty or missing Content-Length");
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid content length");
    return ESP_FAIL;
  }

  char ota_buf[OTA_BUFFSIZE];
  int received = 0;
  bool is_first_chunk = true;

  while (remaining > 0) {
    int to_read = remaining < (int)sizeof(ota_buf) ? remaining : (int)sizeof(ota_buf);
    received = httpd_req_recv(req, ota_buf, to_read);
    if (received <= 0) {
      if (received == HTTPD_SOCK_ERR_TIMEOUT) {
        continue;
      }
      ESP_LOGE(TAG, "OTA stream receive error: %d", received);
      if (ota_handle) esp_ota_abort(ota_handle);
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA receive error");
      return ESP_FAIL;
    }

    if (is_first_chunk) {
      is_first_chunk = false;
      esp_camera_deinit();
      esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_begin failed");
        return ESP_FAIL;
      }
      ESP_LOGI(TAG, "esp_ota_begin initialized successfully");
    }

    esp_err_t err = esp_ota_write(ota_handle, (const void *)ota_buf, received);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "esp_ota_write failed (%s)", esp_err_to_name(err));
      esp_ota_abort(ota_handle);
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_write failed");
      return ESP_FAIL;
    }

    remaining -= received;
  }

  esp_err_t err = esp_ota_end(ota_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_end validation failed (%s)", esp_err_to_name(err));
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA image verification failed");
    return ESP_FAIL;
  }

  err = esp_ota_set_boot_partition(update_partition);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)", esp_err_to_name(err));
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition failed");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "OTA Update successful! New boot partition: '%s'. Restarting in 1.5s...",
           update_partition->label);

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_sendstr(req, "OTA update successful! Rebooting...");

  // Launch delayed reboot task
  xTaskCreate(delayed_restart_task, "delayed_restart", 2048, NULL, 5, NULL);
  return ESP_OK;
}

// Start HTTP Server
static httpd_handle_t start_webserver(void) {
  httpd_handle_t server = NULL;
  httpd_handle_t stream_server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;
  config.max_open_sockets = 4;
  config.stack_size = 12288;

  httpd_uri_t index_uri = {.uri = "/",
                           .method = HTTP_GET,
                           .handler = index_handler,
                           .user_ctx = NULL};

  httpd_uri_t stream_uri = {.uri = "/stream",
                            .method = HTTP_GET,
                            .handler = stream_handler,
                            .user_ctx = NULL};

  httpd_uri_t capture_uri = {.uri = "/capture",
                             .method = HTTP_GET,
                             .handler = capture_handler,
                             .user_ctx = NULL};

  httpd_uri_t status_uri = {.uri = "/status",
                            .method = HTTP_GET,
                            .handler = status_handler,
                            .user_ctx = NULL};

  httpd_uri_t update_uri = {.uri = "/update",
                            .method = HTTP_POST,
                            .handler = ota_update_post_handler,
                            .user_ctx = NULL};

  ESP_LOGI(TAG, "Starting HTTP server on port %d...", config.server_port);
  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &stream_uri);
    httpd_register_uri_handler(server, &capture_uri);
    httpd_register_uri_handler(server, &status_uri);
    httpd_register_uri_handler(server, &update_uri);
  } else {
    ESP_LOGE(TAG, "Failed to start HTTP server!");
  }

  config.server_port = 81;
  config.ctrl_port = 32769;
  config.max_open_sockets = 3;
  ESP_LOGI(TAG, "Starting stream HTTP server on port %d...", config.server_port);
  if (httpd_start(&stream_server, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_server, &stream_uri);
  } else {
    ESP_LOGE(TAG, "Failed to start stream HTTP server!");
  }

  return server;
}

// ==============================================================================
// Application Main
// ==============================================================================
void app_main(void) {
  ESP_LOGI(TAG, "==========================================");
  ESP_LOGI(TAG, " Starting ESP32-S3 Camera Application     ");
  ESP_LOGI(TAG, "==========================================");

  // Initialize Non-Volatile Storage (NVS)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Log Memory Diagnostics
  ESP_LOGI(TAG, "Free DRAM heap : %lu bytes",
           (unsigned long)esp_get_free_heap_size());
  if (esp_psram_is_initialized()) {
    ESP_LOGI(TAG, "PSRAM size     : %lu bytes",
             (unsigned long)heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "Free PSRAM     : %lu bytes",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  } else {
    ESP_LOGW(TAG, "PSRAM is NOT detected/initialized.");
  }

  // Connect to Wi-Fi FIRST (prevents ESP32-S3 GDMA freeze when Wi-Fi starts)
  if (wifi_init_sta() != ESP_OK) {
    ESP_LOGE(
        TAG,
        "Wi-Fi connection failed! Please check your credentials in main.c.");
    return;
  }

  // Allow Wi-Fi STA and system clocks to settle before camera DMA starts
  vTaskDelay(pdMS_TO_TICKS(300));

  // Initialize Camera Driver AFTER Wi-Fi is connected
  if (init_camera() != ESP_OK) {
    ESP_LOGE(TAG, "Camera initialization failed! Continuing to start web server for OTA (/update) and status (/status).");
  }

  // Start Web Server
  start_webserver();

  // Fetch and display IP Address
  esp_netif_ip_info_t ip_info;
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
    ESP_LOGI(TAG, "------------------------------------------");
    ESP_LOGI(TAG, "Camera web server is ready!");
    ESP_LOGI(TAG, "Open your web browser and navigate to:");
    ESP_LOGI(TAG, "   http://" IPSTR "/", IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "Stream directly at:");
    ESP_LOGI(TAG, "   http://" IPSTR "/stream", IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "Capture snapshot at:");
    ESP_LOGI(TAG, "   http://" IPSTR "/capture", IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "------------------------------------------");
  }
}
