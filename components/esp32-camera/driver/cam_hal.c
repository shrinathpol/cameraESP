// Copyright 2010-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ll_cam.h"
#include "cam_hal.h"
#include "esp_timer.h"
#include "esp_cache.h"
#include "esp_ipc.h"
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"
#include "esp_idf_version.h"

#if CONFIG_IDF_TARGET_ESP32S3
#include "soc/gdma_struct.h"
#include "soc/lcd_cam_struct.h"
#endif

#if (ESP_IDF_VERSION_MAJOR == 3) && (ESP_IDF_VERSION_MINOR == 3)
#include "rom/ets_sys.h"
#else
#if CONFIG_IDF_TARGET_ESP32
#include "esp32/rom/ets_sys.h"  // will be removed in idf v5.0
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/ets_sys.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/ets_sys.h"
#endif
#endif // ESP_IDF_VERSION_MAJOR
#define ESP_CAMERA_ETS_PRINTF ets_printf

#define CAM_TASK_STACK CONFIG_CAMERA_TASK_STACK_SIZE

#ifndef CAMERA_PSRAM_DMA_ENABLED
#if CONFIG_CAMERA_PSRAM_DMA
#define CAMERA_PSRAM_DMA_ENABLED 1
#else
#define CAMERA_PSRAM_DMA_ENABLED 0
#endif
#endif

static const char *TAG = "cam_hal";
static cam_obj_t *cam_obj = NULL;
static portMUX_TYPE s_cam_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool g_psram_dma_mode = CAMERA_PSRAM_DMA_ENABLED;
static portMUX_TYPE g_psram_dma_lock = portMUX_INITIALIZER_UNLOCKED;

static int cam_verify_jpeg_soi(const uint8_t *inbuf, uint32_t length)
{
    if (length < 4) {
        return -1;
    }
    // Limit search to first 4KB to save CPU/cache
    uint32_t search_len = (length > 4096) ? 4096 : length;
    const uint8_t *ptr = inbuf;
    // Ensure we don't read past the end (need 3 bytes for SOI check)
    const uint8_t *end = inbuf + search_len - 3;

    while (ptr <= end && (ptr = memchr(ptr, 0xFF, (end - ptr) + 1)) != NULL) {
        if (ptr[1] == 0xD8 && ptr[2] == 0xFF) {
            int i = (int)(ptr - inbuf);
            if (cam_obj) {
                if (i == 0) cam_obj->debug.soi_offset_histogram[0]++;
                else if (i < 16) cam_obj->debug.soi_offset_histogram[1]++;
                else if (i < 64) cam_obj->debug.soi_offset_histogram[2]++;
                else if (i < 256) cam_obj->debug.soi_offset_histogram[3]++;
                else if (i < 1024) cam_obj->debug.soi_offset_histogram[4]++;
                else if (i < 4096) cam_obj->debug.soi_offset_histogram[5]++;
                else cam_obj->debug.soi_offset_histogram[6]++;
            }
            return i;
        }
        ptr++;
    }
    if (cam_obj) {
        cam_obj->debug.no_soi_count++;
    }
    if (length > 128) {
        ESP_LOGW(TAG, "NO-SOI in %u bytes", (unsigned)length);
    }
    return -1;
}

static int cam_verify_jpeg_eoi(const uint8_t *inbuf, uint32_t length)
{
    if (length < 2) {
        return -1;
    }
    // Limit search to last 32KB
    uint32_t search_len = (length > 32768) ? 32768 : length;
    uint32_t start = length - search_len;

    for (uint32_t p = length - 2; ; p--) {
        if (inbuf[p] == 0xFF && inbuf[p + 1] == 0xD9) {
            return (int)p;
        }
        if (p == start) {
            break;
        }
    }

    if (cam_obj) {
        cam_obj->debug.no_eoi_count++;
    }
    return -1;
}

// Forward search for the first EOI (FF D9) in inbuf[from, length) for PSRAM JPEG frames
static int cam_find_jpeg_eoi_forward(const uint8_t *inbuf, uint32_t from, uint32_t length)
{
    if (length < 2) {
        return -1;
    }
    for (uint32_t p = from; p + 1 < length; p++) {
        if (inbuf[p] == 0xFF && inbuf[p + 1] == 0xD9) {
            return (int)p;
        }
    }
    if (cam_obj) {
        cam_obj->debug.no_eoi_count++;
    }
    return -1;
}

static bool cam_get_next_frame(int *frame_pos)
{
    if (!cam_obj->frames[*frame_pos].en) {
        for (int x = 0; x < cam_obj->frame_cnt; x++) {
            if (cam_obj->frames[x].en) {
                *frame_pos = x;
                return true;
            }
        }
    } else {
        return true;
    }
    return false;
}

static bool cam_start_frame(int *frame_pos)
{
    if (cam_get_next_frame(frame_pos)) {
        if (ll_cam_prepare_frame(cam_obj, *frame_pos)) {
            uint64_t us = (uint64_t)esp_timer_get_time();
            cam_obj->frames[*frame_pos].fb.timestamp.tv_sec = us / 1000000UL;
            cam_obj->frames[*frame_pos].fb.timestamp.tv_usec = us % 1000000UL;
            return true;
        }
    }
    return false;
}

void IRAM_ATTR ll_cam_send_event(cam_obj_t *cam, cam_event_t cam_event, BaseType_t *HPTaskAwoken)
{
    if (xQueueSendFromISR(cam->event_queue, (void *)&cam_event, HPTaskAwoken) != pdTRUE) {
        cam->debug.queue_overflow_count++;
        if (!(cam->psram_mode && cam->jpeg_mode)) {
            ll_cam_stop(cam);
            cam->state = CAM_STATE_IDLE;
        }
        ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: EV-%s-OVF\r\n"), cam_event == CAM_IN_SUC_EOF_EVENT ? DRAM_STR("EOF") : DRAM_STR("VSYNC"));
    }
}

// Reset DMA descriptors and return buffer to free pool
static void cam_reset_and_free_frame(int frame_pos)
{
    size_t off = cam_obj->frames[frame_pos].jpeg_soi_offset;
    if (off) {
        cam_obj->frames[frame_pos].fb.buf -= off;
        cam_obj->frames[frame_pos].jpeg_soi_offset = 0;
    }
    lldesc_t *dma = cam_obj->frames[frame_pos].dma;
    if (dma) {
        for (int i = 0; i < cam_obj->dma_node_cnt; i++) {
            dma[i].length = 0;
            dma[i].sosf = 0;
            dma[i].eof = 0;
            dma[i].owner = 1;
        }
    }
    cam_obj->frames[frame_pos].en = 1;

    portENTER_CRITICAL(&s_cam_mux);
    __atomic_fetch_or(&cam_obj->free_mask, (1u << frame_pos), __ATOMIC_SEQ_CST);
    portEXIT_CRITICAL(&s_cam_mux);
}

// PSRAM JPEG task loop — ISR-driven frame swap, task finalizes
static void cam_psram_jpeg_task(void)
{
    cam_frame_done_t done;
    uint32_t good_frames = 0;
    uint32_t last_vsync = 0;
    uint32_t last_eof = 0;

    while (1) {
        if (xQueueReceive(cam_obj->frame_done_queue, &done, pdMS_TO_TICKS(2000)) != pdTRUE) {
            if (!LCD_CAM.lc_dma_int_ena.cam_vsync_int_ena) {
                last_vsync = cam_obj->debug.vsync_isr_count;
                last_eof = cam_obj->debug.eof_count;
                continue;
            }
            uint32_t free_mask = __atomic_load_n(&cam_obj->free_mask, __ATOMIC_SEQ_CST);
            if (free_mask != 0 && cam_obj->debug.vsync_isr_count > last_vsync + 10 && cam_obj->debug.eof_count == last_eof) {
                ESP_LOGE(TAG, "Hard Stall detected (GDMA dead with %u buffers free)", (unsigned)__builtin_popcount(free_mask));
            } else if (free_mask != 0) {
                ESP_LOGW(TAG, "PSRAM-JPEG-WAIT: vsync=%lu eof=%lu good=%lu (Backpressure or Sluggish)",
                         (unsigned long)cam_obj->debug.vsync_isr_count,
                         (unsigned long)cam_obj->debug.eof_count,
                         (unsigned long)good_frames);
            }

            last_vsync = cam_obj->debug.vsync_isr_count;
            last_eof = cam_obj->debug.eof_count;
            continue;
        }

        last_vsync = cam_obj->debug.vsync_isr_count;
        last_eof = cam_obj->debug.eof_count;

        camera_fb_t *fb = &cam_obj->frames[done.frame_pos].fb;

        // Compute length from complete DMA EOF chunks
        size_t len = (size_t)done.eof_cnt * cam_obj->dma_half_buffer_size;
        if (len > cam_obj->fb_size) {
            ESP_LOGW(TAG, "PSRAM-OVF: %u > %u", (unsigned)len, (unsigned)cam_obj->fb_size);
            len = cam_obj->fb_size;
        }
        fb->len = len;

        // Extend cache sync and EOI scan by one extra DMA chunk to cover trailing partial data
        size_t eoi_search_len = len + cam_obj->dma_half_buffer_size;
        if (eoi_search_len > cam_obj->fb_size) {
            eoi_search_len = cam_obj->fb_size;
        }

        // Cache invalidate (PSRAM DMA coherency)
        size_t alignment = 64;
        uint8_t *start = (uint8_t *)((uintptr_t)fb->buf & ~(uintptr_t)(alignment - 1));
        uint8_t *end = (uint8_t *)(((uintptr_t)(fb->buf + eoi_search_len) + alignment - 1) & ~(uintptr_t)(alignment - 1));
        esp_cache_msync(start, (size_t)(end - start), ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_INVALIDATE);
        cam_obj->debug.psram_cache_msync_count++;

        uint64_t us = (uint64_t)esp_timer_get_time();
        fb->timestamp.tv_sec = us / 1000000UL;
        fb->timestamp.tv_usec = us % 1000000UL;

        // SOI check (search first 4KB)
        size_t search_len = (len > 4096) ? 4096 : len;
        int soi = cam_verify_jpeg_soi(fb->buf, (uint32_t)search_len);
        if (soi < 0) {
            cam_reset_and_free_frame(done.frame_pos);
            ll_cam_vsync_intr_enable(cam_obj, true);
            continue;
        }

        // Logical SOI trim via pointer shift
        if (soi > 0) {
            cam_obj->frames[done.frame_pos].jpeg_soi_offset = (size_t)soi;
            fb->buf += soi;
            fb->len -= (size_t)soi;
        }

        // Forward EOI scan from the start of the last complete DMA chunk
        size_t eoi_len = eoi_search_len - (size_t)(soi > 0 ? soi : 0);
        size_t eoi_from = (fb->len > cam_obj->dma_half_buffer_size + 2)
                          ? fb->len - cam_obj->dma_half_buffer_size : 2;
        int eoi = cam_find_jpeg_eoi_forward(fb->buf, (uint32_t)eoi_from, (uint32_t)eoi_len);
        if (eoi < 0) {
            cam_reset_and_free_frame(done.frame_pos);
            ll_cam_vsync_intr_enable(cam_obj, true);
            continue;
        }
        fb->len = (size_t)eoi + 2;

        // Enqueue to consumer (GRAB_LATEST: pop oldest if queue full)
        cam_obj->frames[done.frame_pos].en = 0;
        if (xQueueSend(cam_obj->frame_buffer_queue, (void *)&fb, 0) != pdTRUE) {
            camera_fb_t *old = NULL;
            if (xQueueReceive(cam_obj->frame_buffer_queue, &old, 0) == pdTRUE) {
                cam_give(old);
            }
            if (xQueueSend(cam_obj->frame_buffer_queue, (void *)&fb, 0) != pdTRUE) {
                cam_reset_and_free_frame(done.frame_pos);
                continue;
            }
        }
        good_frames++;
    }
}

// Copy frame from DMA dma_buffer to frame dma_buffer
static void cam_task(void *arg)
{
    if (cam_obj->psram_mode && cam_obj->jpeg_mode) {
        cam_psram_jpeg_task();
    }

    int cnt = 0;
    int frame_pos = 0;
    cam_obj->state = CAM_STATE_IDLE;
    cam_event_t cam_event = 0;
    int soi_offset = 0;

    xQueueReset(cam_obj->event_queue);

    while (1) {
        xQueueReceive(cam_obj->event_queue, (void *)&cam_event, portMAX_DELAY);
        DBG_PIN_SET(1);
        switch (cam_obj->state) {
            case CAM_STATE_IDLE: {
                if (cam_event == CAM_VSYNC_EVENT) {
                    soi_offset = 0;
                    if (cam_start_frame(&frame_pos)) {
                        cam_obj->frames[frame_pos].fb.len = 0;
                        cam_obj->state = CAM_STATE_READ_BUF;
                    }
                    cnt = 0;
                }
            }
            break;

            case CAM_STATE_READ_BUF: {
                camera_fb_t *frame_buffer_event = &cam_obj->frames[frame_pos].fb;
                size_t pixels_per_dma = (cam_obj->dma_half_buffer_size * cam_obj->fb_bytes_per_pixel) / (cam_obj->dma_bytes_per_item * cam_obj->in_bytes_per_pixel);

                if (cam_event == CAM_IN_SUC_EOF_EVENT) {
                    if (!cam_obj->psram_mode) {
                        if (cam_obj->fb_size < (frame_buffer_event->len + pixels_per_dma)) {
                            ESP_LOGW(TAG, "FB-OVF");
                            ll_cam_stop(cam_obj);
                            DBG_PIN_SET(0);
                            continue;
                        }
                        frame_buffer_event->len += ll_cam_memcpy(cam_obj,
                            &frame_buffer_event->buf[frame_buffer_event->len],
                            &cam_obj->dma_buffer[(cnt % cam_obj->dma_half_buffer_cnt) * cam_obj->dma_half_buffer_size],
                            cam_obj->dma_half_buffer_size);
                    }

                    if (cam_obj->jpeg_mode && cnt == 0) {
                        soi_offset = cam_verify_jpeg_soi(frame_buffer_event->buf, cam_obj->dma_half_buffer_size);
                        if (soi_offset < 0) {
                            ll_cam_stop(cam_obj);
                            cam_obj->state = CAM_STATE_IDLE;
                        }
                    }
                    cnt++;

                } else if (cam_event == CAM_VSYNC_EVENT) {
                    if (!cam_obj->capturing) {
                        ll_cam_stop(cam_obj);
                    }

                    if (cnt || !cam_obj->jpeg_mode) {
                        if (cam_obj->jpeg_mode) {
                            if (cam_obj->fb_size < (frame_buffer_event->len + pixels_per_dma)) {
                                ESP_LOGW(TAG, "FB-OVF");
                                cnt--;
                            } else {
                                frame_buffer_event->len += ll_cam_memcpy(cam_obj,
                                    &frame_buffer_event->buf[frame_buffer_event->len],
                                    &cam_obj->dma_buffer[(cnt % cam_obj->dma_half_buffer_cnt) * cam_obj->dma_half_buffer_size],
                                    cam_obj->dma_half_buffer_size);
                            }
                            cnt++;
                        }

                        cam_obj->frames[frame_pos].en = 0;

                        if (!cam_obj->jpeg_mode) {
                            if (frame_buffer_event->len != cam_obj->fb_size) {
                                cam_obj->frames[frame_pos].en = 1;
                                ESP_LOGE(TAG, "FB-SIZE: %u != %u", (unsigned)frame_buffer_event->len, (unsigned)cam_obj->fb_size);
                            }
                        } else if (!cam_obj->psram_mode && soi_offset > 0) {
                            // Trim leading padding bytes in DRAM JPEG mode
                            frame_buffer_event->len -= (size_t)soi_offset;
                            memmove(frame_buffer_event->buf, &frame_buffer_event->buf[soi_offset], frame_buffer_event->len);
                        }

                        if (!cam_obj->frames[frame_pos].en && xQueueSend(cam_obj->frame_buffer_queue, (void *)&frame_buffer_event, 0) != pdTRUE) {
                            camera_fb_t *fb2 = NULL;
                            if (xQueueReceive(cam_obj->frame_buffer_queue, &fb2, 0) == pdTRUE) {
                                if (xQueueSend(cam_obj->frame_buffer_queue, (void *)&frame_buffer_event, 0) != pdTRUE) {
                                    cam_obj->frames[frame_pos].en = 1;
                                    ESP_LOGE(TAG, "FBQ-SND");
                                }
                                cam_give(fb2);
                            } else {
                                cam_obj->frames[frame_pos].en = 1;
                                ESP_LOGE(TAG, "FBQ-RCV");
                            }
                        }
                    }

                    if (!cam_start_frame(&frame_pos)) {
                        cam_obj->state = CAM_STATE_IDLE;
                    } else {
                        cam_obj->frames[frame_pos].fb.len = 0;
                    }
                    cnt = 0;
                    soi_offset = 0;
                }
            }
            break;
        }
        DBG_PIN_SET(0);
    }
}

static lldesc_t *allocate_dma_descriptors(uint32_t count, uint16_t size, uint8_t *buffer)
{
    lldesc_t *dma = (lldesc_t *)heap_caps_malloc(count * sizeof(lldesc_t), MALLOC_CAP_DMA);
    if (dma == NULL) {
        return dma;
    }

    for (uint32_t x = 0; x < count; x++) {
        dma[x].size = size;
        dma[x].length = 0;
        dma[x].sosf = 0;
        dma[x].eof = 0;
        dma[x].owner = 1;
        dma[x].buf = (buffer + size * x);
        dma[x].empty = (uint32_t)&dma[(x + 1) % count];
    }
    return dma;
}

static esp_err_t cam_dma_config(const camera_config_t *config)
{
    bool ret = ll_cam_dma_sizes(cam_obj);
    if (0 == ret) {
        return ESP_FAIL;
    }

    cam_obj->dma_node_cnt = (cam_obj->dma_buffer_size) / cam_obj->dma_node_buffer_size;
    cam_obj->frame_copy_cnt = cam_obj->recv_size / cam_obj->dma_half_buffer_size;

    ESP_LOGI(TAG, "buffer_size: %d, half_buffer_size: %d, node_buffer_size: %d, node_cnt: %d, total_cnt: %d",
             (int)cam_obj->dma_buffer_size, (int)cam_obj->dma_half_buffer_size, (int)cam_obj->dma_node_buffer_size,
             (int)cam_obj->dma_node_cnt, (int)cam_obj->frame_copy_cnt);

    cam_obj->dma_buffer = NULL;
    cam_obj->dma = NULL;

    cam_obj->frames = (cam_frame_t *)heap_caps_calloc(1, cam_obj->frame_cnt * sizeof(cam_frame_t), MALLOC_CAP_DEFAULT);
    CAM_CHECK(cam_obj->frames != NULL, "frames malloc failed", ESP_FAIL);

    size_t fb_size = cam_obj->fb_size;
    if (cam_obj->psram_mode) {
        if (cam_obj->fb_size < cam_obj->recv_size) {
            fb_size = cam_obj->recv_size;
        }
    }

    size_t alloc_size = fb_size * sizeof(uint8_t) + 64;
    uint32_t _caps = MALLOC_CAP_8BIT;
    if (CAMERA_FB_IN_DRAM == config->fb_location) {
        _caps |= MALLOC_CAP_INTERNAL;
    } else {
        _caps |= MALLOC_CAP_SPIRAM;
    }

    for (int x = 0; x < cam_obj->frame_cnt; x++) {
        cam_obj->frames[x].dma = NULL;
        cam_obj->frames[x].fb_orig = NULL;
        cam_obj->frames[x].jpeg_soi_offset = 0;
        cam_obj->frames[x].en = 0;
        ESP_LOGI(TAG, "Allocating %u Byte frame buffer in %s (+64 padding)", (unsigned)alloc_size, _caps & MALLOC_CAP_SPIRAM ? "PSRAM" : "OnBoard RAM");

        // Force manual 64-byte alignment for S3 cache line safety
        uint8_t *raw_buf = (uint8_t *)heap_caps_malloc(alloc_size + 64, _caps);
        CAM_CHECK(raw_buf != NULL, "frame buffer malloc failed", ESP_FAIL);

        cam_obj->frames[x].fb_orig = raw_buf;
        uintptr_t addr = (uintptr_t)raw_buf;
        size_t offset = (64 - (addr & 63)) & 63;
        cam_obj->frames[x].fb.buf = raw_buf + offset;

        if (cam_obj->psram_mode) {
            ESP_LOGI(TAG, "Frame[%d]: Offset: %u, Addr: 0x%08X", x, (unsigned)offset, (unsigned)(uintptr_t)cam_obj->frames[x].fb.buf);
            cam_obj->frames[x].dma = allocate_dma_descriptors(cam_obj->dma_node_cnt, cam_obj->dma_node_buffer_size, cam_obj->frames[x].fb.buf);
            CAM_CHECK(cam_obj->frames[x].dma != NULL, "frame dma malloc failed", ESP_FAIL);
        }
        cam_obj->frames[x].en = 1;
    }

    if (!cam_obj->psram_mode) {
        cam_obj->dma_buffer = (uint8_t *)heap_caps_malloc(cam_obj->dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);
        if (NULL == cam_obj->dma_buffer) {
            ESP_LOGE(TAG, "%s(%d): DMA buffer %d Byte malloc failed, the current largest free block:%d Byte", __FUNCTION__, __LINE__,
                     (int)cam_obj->dma_buffer_size, (int)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
            return ESP_FAIL;
        }

        cam_obj->dma = allocate_dma_descriptors(cam_obj->dma_node_cnt, cam_obj->dma_node_buffer_size, cam_obj->dma_buffer);
        CAM_CHECK(cam_obj->dma != NULL, "dma malloc failed", ESP_FAIL);
    }

    return ESP_OK;
}

esp_err_t cam_init(const camera_config_t *config)
{
    CAM_CHECK(NULL != config, "config pointer is invalid", ESP_ERR_INVALID_ARG);

    esp_err_t ret = ESP_OK;
    cam_obj = (cam_obj_t *)heap_caps_calloc(1, sizeof(cam_obj_t), MALLOC_CAP_DMA);
    CAM_CHECK(NULL != cam_obj, "lcd_cam object malloc error", ESP_ERR_NO_MEM);

    cam_obj->swap_data = 0;
    cam_obj->vsync_pin = config->pin_vsync;
    cam_obj->vsync_invert = false;

#if CONFIG_IDF_TARGET_ESP32
    cam_obj->psram_mode = false;
#else
    cam_obj->psram_mode = (config->fb_location == CAMERA_FB_IN_PSRAM);
#endif

    ll_cam_set_pin(cam_obj, config);
    ret = ll_cam_config(cam_obj, config);
    CAM_CHECK_GOTO(ret == ESP_OK, "ll_cam initialize failed", err);

#if CAMERA_DBG_PIN_ENABLE
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[DBG_PIN_NUM], PIN_FUNC_GPIO);
    gpio_set_direction(DBG_PIN_NUM, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(DBG_PIN_NUM, GPIO_FLOATING);
#endif

    ESP_LOGI(TAG, "cam init ok");
    return ESP_OK;

err:
    free(cam_obj);
    cam_obj = NULL;
    return ESP_FAIL;
}

typedef struct {
    cam_obj_t *cam;
    esp_err_t ret;
} cam_isr_init_arg_t;

static void cam_isr_init_ipc_wrapper(void *arg)
{
    cam_isr_init_arg_t *p = (cam_isr_init_arg_t *)arg;
    p->ret = ll_cam_init_isr(p->cam);
}

esp_err_t cam_config(const camera_config_t *config, framesize_t frame_size, uint16_t sensor_pid)
{
    CAM_CHECK(NULL != config, "config pointer is invalid", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;

    ret = ll_cam_set_sample_mode(cam_obj, (pixformat_t)config->pixel_format, config->xclk_freq_hz, sensor_pid);
    CAM_CHECK_GOTO(ret == ESP_OK, "ll_cam_set_sample_mode failed", err);

    cam_obj->jpeg_mode = config->pixel_format == PIXFORMAT_JPEG;
#if CONFIG_IDF_TARGET_ESP32
    cam_obj->psram_mode = false;
#else
    cam_obj->psram_mode = (config->fb_location == CAMERA_FB_IN_PSRAM);
#endif
    cam_obj->frame_cnt = config->fb_count;
    cam_obj->width = resolution[frame_size].width;
    cam_obj->height = resolution[frame_size].height;

    if (cam_obj->jpeg_mode) {
        if (cam_obj->psram_mode) {
            size_t min_size = (frame_size <= FRAMESIZE_VGA) ? (512 * 1024) :
                              (frame_size <= FRAMESIZE_XGA) ? (768 * 1024) : (1024 * 1024);
            cam_obj->recv_size = (config->jpeg_buffer_size >= min_size) ? config->jpeg_buffer_size : min_size;
        } else {
            cam_obj->recv_size = (config->jpeg_buffer_size > 0) ? config->jpeg_buffer_size : (cam_obj->width * cam_obj->height / 5);
        }
        cam_obj->fb_size = cam_obj->recv_size;
    } else {
        cam_obj->recv_size = cam_obj->width * cam_obj->height * cam_obj->in_bytes_per_pixel;
        cam_obj->fb_size = cam_obj->width * cam_obj->height * cam_obj->fb_bytes_per_pixel;
    }

    ret = cam_dma_config(config);
    CAM_CHECK_GOTO(ret == ESP_OK, "cam_dma_config failed", err);

    size_t queue_size = cam_obj->dma_half_buffer_cnt - 1;
    if (queue_size == 0) {
        queue_size = 1;
    }
    cam_obj->event_queue = xQueueCreate(queue_size, sizeof(cam_event_t));
    CAM_CHECK_GOTO(cam_obj->event_queue != NULL, "event_queue create failed", err);

    size_t frame_buffer_queue_len = cam_obj->frame_cnt;
    if (config->grab_mode == CAMERA_GRAB_LATEST && cam_obj->frame_cnt > 1) {
        frame_buffer_queue_len = (cam_obj->psram_mode && cam_obj->jpeg_mode) ? 1 : (cam_obj->frame_cnt - 1);
    }
    cam_obj->frame_buffer_queue = xQueueCreate(frame_buffer_queue_len, sizeof(camera_fb_t *));
    CAM_CHECK_GOTO(cam_obj->frame_buffer_queue != NULL, "frame_buffer_queue create failed", err);

    if (cam_obj->psram_mode && cam_obj->jpeg_mode) {
        cam_obj->frame_done_queue = xQueueCreate(8, sizeof(cam_frame_done_t));
        CAM_CHECK_GOTO(cam_obj->frame_done_queue != NULL, "frame_done_queue create failed", err);
    }

    cam_obj->free_mask = (1u << cam_obj->frame_cnt) - 1;
    cam_obj->cur_frame_pos = 0;
    cam_obj->cur_eof_cnt = 0;
    cam_obj->isr_seq = 0;
    cam_obj->drops_no_free_buf = 0;
    cam_obj->frames_via_recovery = 0;
    cam_obj->capturing = false;

#if CONFIG_CAMERA_CORE1
    cam_isr_init_arg_t isr_arg = {.cam = cam_obj, .ret = ESP_OK};
    if (esp_ipc_call_blocking(1, cam_isr_init_ipc_wrapper, &isr_arg) != ESP_OK) {
        ESP_LOGE(TAG, "IPC call for ISR init failed");
        goto err;
    }
    ret = isr_arg.ret;
#else
    ret = ll_cam_init_isr(cam_obj);
#endif
    CAM_CHECK_GOTO(ret == ESP_OK, "cam intr alloc failed", err);

#if CONFIG_CAMERA_CORE0
    xTaskCreatePinnedToCore(cam_task, "cam_task", CAM_TASK_STACK, NULL, 10, &cam_obj->task_handle, 0);
#elif CONFIG_CAMERA_CORE1
    xTaskCreatePinnedToCore(cam_task, "cam_task", CAM_TASK_STACK, NULL, 10, &cam_obj->task_handle, 1);
#else
    xTaskCreate(cam_task, "cam_task", CAM_TASK_STACK, NULL, 10, &cam_obj->task_handle);
#endif

    ESP_LOGI(TAG, "cam config ok");
    return ESP_OK;

err:
    cam_deinit();
    return ESP_FAIL;
}

esp_err_t cam_deinit(void)
{
    ESP_LOGW(TAG, "cam_deinit called");
    if (!cam_obj) {
        ESP_LOGE(TAG, "cam_deinit: cam_obj is NULL!");
        return ESP_FAIL;
    }

    cam_stop();
    if (cam_obj->task_handle) {
        ESP_LOGW(TAG, "cam_deinit: Suspending and deleting cam_task...");
        vTaskSuspend(cam_obj->task_handle);
        vTaskDelete(cam_obj->task_handle);
        vTaskDelay(pdMS_TO_TICKS(50));
        cam_obj->task_handle = NULL;
        ESP_LOGW(TAG, "cam_deinit: cam_task deleted");
    } else {
        ESP_LOGW(TAG, "cam_deinit: No task handle found");
    }

    ll_cam_deinit(cam_obj);

    if (cam_obj->event_queue) {
        vQueueDelete(cam_obj->event_queue);
    }
    if (cam_obj->frame_buffer_queue) {
        vQueueDelete(cam_obj->frame_buffer_queue);
    }
    if (cam_obj->frame_done_queue) {
        vQueueDelete(cam_obj->frame_done_queue);
    }

    if (cam_obj->dma) {
        free(cam_obj->dma);
    }
    if (cam_obj->dma_buffer) {
        free(cam_obj->dma_buffer);
    }
    if (cam_obj->frames) {
        for (int x = 0; x < cam_obj->frame_cnt; x++) {
            if (cam_obj->frames[x].fb_orig) {
                free(cam_obj->frames[x].fb_orig);
            }
            if (cam_obj->frames[x].dma) {
                free(cam_obj->frames[x].dma);
            }
        }
        free(cam_obj->frames);
    }

    free(cam_obj);
    cam_obj = NULL;
    ESP_LOGW(TAG, "cam_deinit: Hardware and memory released");
    return ESP_OK;
}

void cam_stop(void)
{
    if (!cam_obj) {
        return;
    }
    ll_cam_vsync_intr_enable(cam_obj, false);
    ll_cam_stop(cam_obj);
}

void cam_start(void)
{
    if (!cam_obj) {
        return;
    }
    ll_cam_vsync_intr_enable(cam_obj, true);
}

camera_fb_t *cam_take(TickType_t timeout)
{
    if (!cam_obj || !cam_obj->frame_buffer_queue) {
        return NULL;
    }

    camera_fb_t *dma_buffer = NULL;
    TickType_t start = xTaskGetTickCount();
    TickType_t remaining = timeout;

    while (remaining > 0) {
        if (cam_obj->psram_mode && cam_obj->jpeg_mode &&
            uxQueueMessagesWaiting(cam_obj->frame_buffer_queue) == 0) {
            ll_cam_vsync_intr_enable(cam_obj, true);
        }
        dma_buffer = NULL;
        xQueueReceive(cam_obj->frame_buffer_queue, (void *)&dma_buffer, remaining);
        if (!dma_buffer) {
            ESP_LOGW(TAG, "Failed to get the frame on time!");
            return NULL;
        }

        if (cam_obj->jpeg_mode) {
            if (cam_obj->psram_mode) {
                return dma_buffer;
            }
            int offset_e = cam_verify_jpeg_eoi(dma_buffer->buf, (uint32_t)dma_buffer->len);
            if (offset_e >= 0) {
                dma_buffer->len = (size_t)offset_e + 2;
                return dma_buffer;
            }
            ESP_LOGW(TAG, "NO-EOI");
            cam_give(dma_buffer);
            TickType_t elapsed = xTaskGetTickCount() - start;
            remaining = (elapsed < timeout) ? (timeout - elapsed) : 0;
            continue;
        }

        if (cam_obj->psram_mode && cam_obj->in_bytes_per_pixel != cam_obj->fb_bytes_per_pixel) {
            dma_buffer->len = ll_cam_memcpy(cam_obj, dma_buffer->buf, dma_buffer->buf, dma_buffer->len);
        }
        return dma_buffer;
    }

    ESP_LOGW(TAG, "Failed to get the frame on time!");
    return NULL;
}

void cam_give(camera_fb_t *dma_buffer)
{
    if (!cam_obj) {
        return;
    }
    for (int x = 0; x < cam_obj->frame_cnt; x++) {
        if (&cam_obj->frames[x].fb == dma_buffer) {
            if (cam_obj->psram_mode && cam_obj->jpeg_mode) {
                cam_reset_and_free_frame(x);
            } else {
                cam_obj->frames[x].en = 1;
                if (cam_obj->state == CAM_STATE_IDLE) {
                    ll_cam_vsync_intr_enable(cam_obj, true);
                    cam_event_t vsync = CAM_VSYNC_EVENT;
                    xQueueSend(cam_obj->event_queue, &vsync, 0);
                }
            }
            break;
        }
    }
}

void cam_give_all(void)
{
    if (!cam_obj) {
        return;
    }
    for (int x = 0; x < cam_obj->frame_cnt; x++) {
        if (cam_obj->psram_mode && cam_obj->jpeg_mode) {
            cam_reset_and_free_frame(x);
        } else {
            cam_obj->frames[x].en = 1;
        }
    }
}

bool cam_get_available_frames(void)
{
    if (!cam_obj || !cam_obj->frame_buffer_queue) {
        return false;
    }
    return 0 < uxQueueMessagesWaiting(cam_obj->frame_buffer_queue);
}

void cam_set_psram_mode(bool enable)
{
    portENTER_CRITICAL(&g_psram_dma_lock);
    g_psram_dma_mode = enable;
    portEXIT_CRITICAL(&g_psram_dma_lock);
}

bool cam_get_psram_mode(void)
{
    return g_psram_dma_mode;
}
