#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <device.h>
#include <esp_matter.h>
#include <led_driver.h>

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val){
  esp_err_r err = ESP_OK;
  if (cluster_id == OnOff::Id){
    if (attribute_id == OnOff::Attributes::OnOff::Id){
      bool new_onoff_state = val->val.b;
    }
  }
}
