#pragma once
#include "liblvgl/lvgl.h"

void initBarGraph();
void setBarAttributes(lv_obj_t* barObj, lv_obj_t* allignTo, int offSet);
void setBarValAndColor(lv_obj_t *bar, lv_obj_t *temperatureLabel, int temperature);
void updateBarGraph();
void updateBarGraph_fn(void* param);