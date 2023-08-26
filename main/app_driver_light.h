#pragma once
#include <esp_err.h>
#include <esp_matter.h>
esp_err_r app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val);
