/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define EXAMPLE_IR_RESOLUTION_HZ     1000000 // 1MHz resolution, 1 tick = 1us
#define EXAMPLE_IR_TX_GPIO_NUM       5

#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "ir_nec_encoder.h"
#include "ir_sharpac_encoder.h"

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <app_priv.h>
#include <app_reset.h>
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

static const char *TAG = "app_main";
uint16_t light_endpoint_id = 0;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

constexpr auto k_timeout_seconds = 300;

#if CONFIG_ENABLE_ENCRYPTED_OTA
extern const char decryption_key_start[] asm("_binary_esp_image_encryption_key_pem_start");
extern const char decryption_key_end[] asm("_binary_esp_image_encryption_key_pem_end");

static const char *s_decryption_key = decryption_key_start;
static const uint16_t s_decryption_key_len = decryption_key_end - decryption_key_start;
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

rmt_tx_channel_config_t tx_channel_cfg;
rmt_channel_handle_t tx_channel;
rmt_carrier_config_t carrier_cfg;
rmt_transmit_config_t transmit_config;

rmt_encoder_handle_t nec_encoder;
rmt_encoder_handle_t sharpac_encoder;

#define MATTER_AC_TYPE_POWER_OFF 0
#define MATTER_AC_TYPE_COOLING 3
#define MATTER_AC_TYPE_HEATING 4

int16_t currentTargetTemp = 26;
uint8_t currentTypeInfo = MATTER_AC_TYPE_POWER_OFF;

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        {
            ESP_LOGI(TAG, "Fabric removed successfully");
            if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0)
            {
                chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
                constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
                if (!commissionMgr.IsCommissioningWindowOpen())
                {
                    /* After removing last fabric, this example does not remove the Wi-Fi credentials
                     * and still has IP connectivity so, only advertising on DNS-SD.
                     */
                    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                                                    chip::CommissioningWindowAdvertisement::kDnssdOnly);
                    if (err != CHIP_NO_ERROR)
                    {
                        ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
                    }
                }
            }
        break;
        }

    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(TAG, "Fabric is committed");
        break;
    default:
        break;
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

static esp_err_t rmt_transmit_sharpac(int16_t targetTemp, uint8_t typeInfo){
  uint8_t sharp_scan_code[13] = {0xAA, 0x5A, 0xCF, 0x10, 0x0A, 0x21, 0x22, 0x00, 0x08, 0xA0, 0x00, 0xE4, 0xD1};
  if (typeInfo == MATTER_AC_TYPE_POWER_OFF){
    ESP_ERROR_CHECK(rmt_transmit(tx_channel, sharpac_encoder, sharp_scan_code, sizeof(sharp_scan_code), &transmit_config));
    ESP_LOGI(TAG, "power off");
    return ESP_OK;
  }
  if (typeInfo == MATTER_AC_TYPE_COOLING){
    sharp_scan_code[5] = 0x11;
    ESP_LOGI(TAG, "cooling");
    //sharp_scan_code[6] = 0x22;
  } else if (typeInfo == MATTER_AC_TYPE_HEATING){
    ESP_LOGI(TAG, "heating");
    sharp_scan_code[5] = 0x11;
    sharp_scan_code[6] = 0x21;
  }
  int8_t tempInDegree = targetTemp/100;
  ESP_LOGI(TAG, "temp: %i", tempInDegree);
  sharp_scan_code[4] = tempInDegree - 17;
  uint8_t checkSum = 0x00;
  for(uint8_t index=0; index<11; index++){
    checkSum ^= sharp_scan_code[index];
  }
  sharp_scan_code[12] = ((checkSum << 4) & 0xF0) | 0x01;
  ESP_ERROR_CHECK(rmt_transmit(tx_channel, sharpac_encoder, sharp_scan_code, sizeof(sharp_scan_code), &transmit_config));
  return ESP_OK;
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    if (type == PRE_UPDATE) {
        /* Driver update */
        // app_driver_handle_t driver_handle = (app_driver_handle_t)priv_data;
        // err = app_driver_attribute_update(driver_handle, endpoint_id, cluster_id, attribute_id, val);
      if (cluster_id == OnOff::Id){
        if (attribute_id == OnOff::Attributes::OnOff::Id){
          bool new_onoff_state = val->val.b;

          const ir_nec_scan_code_t scan_code = {
              .address = 0x6d82,
              .command = 0x40bf,
          };
          ESP_LOGI(TAG, "sending light IR");
          ESP_ERROR_CHECK(rmt_transmit(tx_channel, nec_encoder, &scan_code, sizeof(scan_code), &transmit_config));
        }
      } else if (cluster_id == Thermostat::Id){
        if (attribute_id == Thermostat::Attributes::OccupiedCoolingSetpoint::Id || attribute_id == Thermostat::Attributes::OccupiedHeatingSetpoint::Id){
          int16_t targetTemp = val->val.i16;
          // uint8_t sharp_scan_code[13] = {0xAA, 0x5A, 0xCF, 0x10, 0x0A, 0x21, 0x22, 0x00, 0x08, 0xA0, 0x00, 0xE4, 0xD1};
          // // TODO: encode system mode, temp

          // ESP_ERROR_CHECK(rmt_transmit(tx_channel, sharpac_encoder, sharp_scan_code, sizeof(sharp_scan_code), &transmit_config));
          // 2800 -> 28.00 degree
          currentTargetTemp = targetTemp;
          ESP_LOGI(TAG, "updating target temperature");
          ESP_ERROR_CHECK(rmt_transmit_sharpac(currentTargetTemp, currentTypeInfo));
        } else if (attribute_id == Thermostat::Attributes::SystemMode::Id){
          uint8_t typeInfo = val->val.u8;
          currentTypeInfo = typeInfo;
          ESP_LOGI(TAG, "updating type info");
          ESP_ERROR_CHECK(rmt_transmit_sharpac(currentTargetTemp, currentTypeInfo));
          // uint8_t sharp_scan_code[13] = {0xAA, 0x5A, 0xCF, 0x10, 0x0A, 0x21, 0x22, 0x00, 0x08, 0xA0, 0x00, 0xE4, 0xD1};
          // // TODO: encode system mode, temp
          // ESP_ERROR_CHECK(rmt_transmit(tx_channel, sharpac_encoder, sharp_scan_code, sizeof(sharp_scan_code), &transmit_config));
        }
      }
    }

    return err;
}

void init_ir(){
  tx_channel_cfg = {
                    .gpio_num = EXAMPLE_IR_TX_GPIO_NUM,
                    .clk_src = RMT_CLK_SRC_DEFAULT,
                    .resolution_hz = EXAMPLE_IR_RESOLUTION_HZ,
                    .mem_block_symbols = 128, // amount of RMT symbols that the channel can store at Ca time
                    .trans_queue_depth = 4,  // number of transactions that allowed to pending in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
  };
  tx_channel = NULL;
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &tx_channel));
  carrier_cfg = {
                 .frequency_hz = 38000, // 38KHz
                 .duty_cycle = 0.50,
  };
  ESP_ERROR_CHECK(rmt_apply_carrier(tx_channel, &carrier_cfg));
  transmit_config = {
                     .loop_count = 0, // no loop
  };
  ir_nec_encoder_config_t nec_encoder_cfg = {
                                             .resolution = EXAMPLE_IR_RESOLUTION_HZ,
  };
  ir_sharpac_encoder_config_t sharpac_encoder_cfg = {
                                                     .resolution = EXAMPLE_IR_RESOLUTION_HZ,
  };
  nec_encoder = NULL;
  ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_encoder_cfg, &nec_encoder));
  sharpac_encoder = NULL;
  ESP_ERROR_CHECK(rmt_new_ir_sharpac_encoder(&sharpac_encoder_cfg, &sharpac_encoder));
  ESP_ERROR_CHECK(rmt_enable(tx_channel));
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();
    //nvs_flash_erase();

    /* initialize IR */
    init_ir();

    /* Initialize driver */
    // app_driver_handle_t light_handle = app_driver_light_init();
    // app_driver_handle_t button_handle = app_driver_button_init();
    // app_reset_button_register(button_handle);

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);

    // extended_color_light::config_t light_config;
    // light_config.on_off.on_off = DEFAULT_POWER;
    // light_config.on_off.lighting.start_up_on_off = nullptr;
    // light_config.level_control.current_level = DEFAULT_BRIGHTNESS;
    // light_config.level_control.lighting.start_up_current_level = DEFAULT_BRIGHTNESS;
    // light_config.color_control.color_mode = EMBER_ZCL_COLOR_MODE_COLOR_TEMPERATURE;
    // light_config.color_control.enhanced_color_mode = EMBER_ZCL_COLOR_MODE_COLOR_TEMPERATURE;
    // light_config.color_control.color_temperature.startup_color_temperature_mireds = nullptr;
    // endpoint_t *endpoint = extended_color_light::create(node, &light_config, ENDPOINT_FLAG_NONE, light_handle);
    on_off_plugin_unit::config_t onoff_light_config;
    onoff_light_config.on_off.on_off = false;
    onoff_light_config.on_off.lighting.start_up_on_off = false;
    endpoint_t *endpoint = on_off_plugin_unit::create(node, &onoff_light_config, ENDPOINT_FLAG_NONE, NULL);

    /* thermostat */
    thermostat::config thermostat_config;
    thermostat_config.thermostat.local_temperature = 24;
    endpoint_t *endpoint_2 = thermostat::create(node, &thermostat_config,
                                                        ENDPOINT_FLAG_NONE, NULL);
    cluster_t *thermostat_cluster = cluster::get(endpoint_2, Thermostat::Id);
    cluster::thermostat::feature::heating::config_t heating_config;
    heating_config.abs_max_heat_setpoint_limit = 3000; // 30 degree
    heating_config.abs_min_heat_setpoint_limit = 1500;
    heating_config.max_heat_setpoint_limit = 2800;
    heating_config.min_heat_setpoint_limit = 2000;
    heating_config.occupied_heating_setpoint = 2300;
    heating_config.pi_heating_demand = 0;
    cluster::thermostat::feature::heating::add(thermostat_cluster, &heating_config);

    cluster::thermostat::feature::cooling::config_t cooling_config;
    cooling_config.abs_max_cool_setpoint_limit = 3200;
    cooling_config.abs_min_cool_setpoint_limit = 1500;
    cooling_config.max_cool_setpoint_limit = 3000;
    cooling_config.min_cool_setpoint_limit = 1700;
    cooling_config.occupied_cooling_setpoint = 2800;
    cooling_config.pi_cooling_demand = 0;
    cluster::thermostat::feature::cooling::add(thermostat_cluster, &cooling_config);

    /* these node and endpoint handles can be used to create/add other endpoints and clusters. */
    if (!node || !endpoint) {
        ESP_LOGE(TAG, "Matter node creation failed");
    }

    light_endpoint_id = endpoint::get_id(endpoint);
    ESP_LOGI(TAG, "Light created with endpoint_id %d", light_endpoint_id);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Matter start failed: %d", err);
    }

    /* Starting driver with default values */
    app_driver_light_set_defaults(light_endpoint_id);

#if CONFIG_ENABLE_ENCRYPTED_OTA
    err = esp_matter_ota_requestor_encrypted_init(s_decryption_key, s_decryption_key_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialized the encrypted OTA, err: %d", err);
    }
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::init();
#endif

    // ----- IR ----- //
     const ir_nec_scan_code_t scan_code = {
                                          .address = 0x6d82,
                                          .command = 0x40bf,
    };
     //ESP_ERROR_CHECK(rmt_transmit_sharpac(2600, MATTER_AC_TYPE_COOLING));
     //ESP_ERROR_CHECK(rmt_transmit_sharpac(2600, MATTER_AC_TYPE_POWER_OFF));

     //ESP_ERROR_CHECK(rmt_transmit(tx_channel, nec_encoder, &scan_code, sizeof(scan_code), &transmit_config));
    //ESP_ERROR_CHECK(rmt_transmit(rx_channel, ));
    // light onoff
    // .address = 0xd729
    // .command = 0x04fb
}
