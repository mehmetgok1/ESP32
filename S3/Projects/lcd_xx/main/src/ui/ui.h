#include "dptp_mng.h"
#include "ble.h"




// Static functions for handling LVGL screens
extern char *added_devices[MAX_DEVICES]; // Array to store device names
extern int device_count ;             // Counter for added devices
extern char remote_device_name[MAX_DEVICES];
extern int flag_connect ;
extern esp_ble_scan_params_t ble_scan_params ;
extern bool scanning ;

extern lv_obj_t *device_list;
extern lv_obj_t *main_screen;
extern lv_obj_t *device_list_screen;
extern lv_obj_t *intermediate_screen;
extern lv_obj_t *connection_screen;
extern lv_disp_t *disp;

void lv_main_screen(void);
void show_device_list_screen(void);
void lv_back_button_event_cb(lv_event_t *e);
void lv_connect_button_event_cb(lv_event_t *e);
void update_device_list();
void show_connected_screen();
void lv_connecting_intermediate_page();
void init_ui(void);
