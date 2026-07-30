#ifndef DUMMY_LVGL_H
#define DUMMY_LVGL_H

#include <stdint.h>

#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif
#ifndef LV_ATTRIBUTE_LARGE_CONST
#define LV_ATTRIBUTE_LARGE_CONST
#endif
#ifndef LV_ATTRIBUTE_IMG_START
#define LV_ATTRIBUTE_IMG_START
#endif
#ifndef LV_ATTRIBUTE_IMG_CIRCLE
#define LV_ATTRIBUTE_IMG_CIRCLE
#endif
#ifndef LV_ATTRIBUTE_IMG_CUP
#define LV_ATTRIBUTE_IMG_CUP
#endif
#ifndef LV_ATTRIBUTE_IMG_GOODJOB
#define LV_ATTRIBUTE_IMG_GOODJOB
#endif
#ifndef LV_ATTRIBUTE_IMG_MOUSE
#define LV_ATTRIBUTE_IMG_MOUSE
#endif
#ifndef LV_ATTRIBUTE_IMG_PHONE
#define LV_ATTRIBUTE_IMG_PHONE
#endif
#ifndef LV_ATTRIBUTE_IMG_RETRY
#define LV_ATTRIBUTE_IMG_RETRY
#endif
#ifndef LV_ATTRIBUTE_IMG_SHAKE
#define LV_ATTRIBUTE_IMG_SHAKE
#endif
#ifndef LV_ATTRIBUTE_IMG_TIMER
#define LV_ATTRIBUTE_IMG_TIMER
#endif

typedef struct {
  struct {
    uint32_t cf: 5;
    uint32_t always_zero: 3;
    uint32_t reserved: 2;
    uint32_t w: 11;
    uint32_t h: 11;
  } header;
  uint32_t data_size;
  const uint8_t *data;
} lv_img_dsc_t;

#define LV_IMG_CF_RAW_CHROMA_KEYED 0

#endif
