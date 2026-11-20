#pragma once

// ==============================================================================
// Select your ESP32-S3 Camera Board Model by uncommenting ONE of the lines below
// ==============================================================================
#define CAMERA_MODEL_FREENOVE_ESP32S3_CAM   // Freenove ESP32-S3 WROOM CAM / Generic S3-CAM
// #define CAMERA_MODEL_ESP32S3_EYE         // Espressif ESP32-S3-EYE official board
// #define CAMERA_MODEL_XIAO_ESP32S3_SENSE  // Seeed Studio XIAO ESP32S3 Sense
// #define CAMERA_MODEL_WAVESHARE_ESP32S3   // Waveshare ESP32-S3-CAM

// ==============================================================================
// Pin Definitions
// ==============================================================================
#if defined(CAMERA_MODEL_FREENOVE_ESP32S3_CAM) || defined(CAMERA_MODEL_ESP32S3_EYE)
    #define PWDN_GPIO_NUM     -1
    #define RESET_GPIO_NUM    -1
    #define XCLK_GPIO_NUM     15
    #define SIOD_GPIO_NUM      4
    #define SIOC_GPIO_NUM      5

    #define Y9_GPIO_NUM       16
    #define Y8_GPIO_NUM       17
    #define Y7_GPIO_NUM       18
    #define Y6_GPIO_NUM       12
    #define Y5_GPIO_NUM       10
    #define Y4_GPIO_NUM        8
    #define Y3_GPIO_NUM        9
    #define Y2_GPIO_NUM       11
    #define VSYNC_GPIO_NUM     6
    #define HREF_GPIO_NUM      7
    #define PCLK_GPIO_NUM     13

#elif defined(CAMERA_MODEL_XIAO_ESP32S3_SENSE)
    #define PWDN_GPIO_NUM     -1
    #define RESET_GPIO_NUM    -1
    #define XCLK_GPIO_NUM     10
    #define SIOD_GPIO_NUM     40
    #define SIOC_GPIO_NUM     39

    #define Y9_GPIO_NUM       48
    #define Y8_GPIO_NUM       11
    #define Y7_GPIO_NUM       12
    #define Y6_GPIO_NUM       14
    #define Y5_GPIO_NUM       16
    #define Y4_GPIO_NUM       18
    #define Y3_GPIO_NUM       17
    #define Y2_GPIO_NUM       15
    #define VSYNC_GPIO_NUM    38
    #define HREF_GPIO_NUM     47
    #define PCLK_GPIO_NUM     13

#elif defined(CAMERA_MODEL_WAVESHARE_ESP32S3)
    #define PWDN_GPIO_NUM     -1
    #define RESET_GPIO_NUM    -1
    #define XCLK_GPIO_NUM     39
    #define SIOD_GPIO_NUM     21
    #define SIOC_GPIO_NUM     14

    #define Y9_GPIO_NUM       40
    #define Y8_GPIO_NUM       41
    #define Y7_GPIO_NUM       42
    #define Y6_GPIO_NUM       38
    #define Y5_GPIO_NUM       37
    #define Y4_GPIO_NUM       36
    #define Y3_GPIO_NUM       35
    #define Y2_GPIO_NUM       34
    #define VSYNC_GPIO_NUM    47
    #define HREF_GPIO_NUM     48
    #define PCLK_GPIO_NUM     45

#else
    #error "Please select a camera model in camera_pins.h!"
#endif
