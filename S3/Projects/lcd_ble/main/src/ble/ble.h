#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"
#include "dptp_mng.h"
/* Declare static functions */
#define GATTC_TAG "BT"
#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
#define MAX_DEVICES 50
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0

extern char *added_devices[MAX_DEVICES]; // Array to store device names
extern int device_count;             // Counter for added devices
extern bool scanning ;

extern esp_ble_scan_params_t ble_scan_params ;
extern lv_obj_t *device_list;
extern lv_obj_t *main_screen;
extern lv_obj_t *device_list_screen;
extern lv_obj_t *intermediate_screen;
extern lv_obj_t *connection_screen;
extern lv_disp_t *disp;

void write_data(int param1,int param2,int param3);  //this function used by ui to update data on screen and send data with ble
void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
void send_parameters_to_device(int param1, int param2,int param3);
void init_ble(void);



