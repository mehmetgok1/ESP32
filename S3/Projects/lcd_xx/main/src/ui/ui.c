#include "ui.h"

char *added_devices[MAX_DEVICES]; // Array to store device names
int device_count = 0;             // Counter for added devices
int flag_connect = 0;
char remote_device_name[MAX_DEVICES];
bool scanning = false;
lv_obj_t *device_list=NULL;
lv_obj_t *main_screen=NULL;
lv_obj_t *device_list_screen = NULL;
lv_obj_t *intermediate_screen=NULL;
lv_obj_t *connection_screen=NULL;
lv_disp_t *disp;

void init_ui(){
    init_ble();
    if (example_lvgl_lock(-1)) {
        lv_main_screen();
        show_device_list_screen();
        lv_connecting_intermediate_page();
        show_connected_screen();

        example_lvgl_unlock();
    }
    lv_scr_load(main_screen);
    
}
// Event callback for Connect button (go to device list screen)
void lv_connect_button_event_cb(lv_event_t *e) {
    esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
    lv_scr_load(device_list_screen);
}
// Main LVGL screen creation
void lv_main_screen(void) {
    main_screen = lv_obj_create(NULL);
    lv_obj_clean(lv_scr_act());  // Clean the current screen
    // Create a circular button
    lv_obj_t *btn_connect = lv_btn_create(main_screen);
    lv_obj_add_event_cb(btn_connect, lv_connect_button_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_size(btn_connect,175,175);  // Adjust size to make it circular
    lv_obj_align(btn_connect, LV_ALIGN_CENTER, 0, 0);  // Center the button
    lv_obj_set_style_radius(btn_connect, LV_RADIUS_CIRCLE, LV_PART_MAIN);  // Make it circular
    lv_obj_set_style_bg_color(btn_connect, lv_color_hex(0x00C8D7), LV_PART_MAIN);  // Set background color (turquoise)
    lv_obj_set_style_shadow_width(btn_connect, 10, LV_PART_MAIN);  // Add shadow for depth
    

    // Add Bluetooth icon to the button
    lv_obj_t *label_icon = lv_label_create(btn_connect);
    lv_label_set_text(label_icon, LV_SYMBOL_BLUETOOTH);  // Use LVGL's built-in Bluetooth symbol
    lv_obj_set_style_text_font(label_icon, &lv_font_montserrat_20, 0);
    lv_obj_align(label_icon, LV_ALIGN_TOP_MID, 0, 30);  // Position near mid

    // Add "Connect" text to the button
    lv_obj_t *label_text = lv_label_create(btn_connect);
    lv_label_set_text(label_text, "Connect");
    lv_obj_set_style_text_font(label_text, &lv_font_montserrat_20, 0);
    lv_obj_align(label_text, LV_ALIGN_CENTER, 0, 0);  // Position near the bottom
}

// Event callback for Back button (go back to main screen)
void lv_back_button_event_cb(lv_event_t *e) {
        esp_ble_gap_set_scan_params(&ble_scan_params);
}
// Callback function for button click
static void btn_list_event_handler(lv_event_t *e) {
    lv_obj_t *btn = lv_event_get_target(e);           // Get the button that triggered the event
    lv_obj_t *list = lv_obj_get_parent(btn);          // Get the parent list
    const char *device_name = lv_list_get_btn_text(list, btn); // Get the button's text
    flag_connect=1;
    strcpy(remote_device_name, device_name);
    printf("Button clicked: %s\n", device_name);
    esp_ble_gap_set_scan_params(&ble_scan_params);
    lv_scr_load(intermediate_screen);
}

// Device List screen creation
void show_device_list_screen(void) {
   
    // Create the device list screen
    device_list_screen = lv_obj_create(NULL);

    // Add title
    lv_obj_t *title = lv_label_create(device_list_screen);
    lv_label_set_text(title, "Available BLE Devices");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, +35);

    // Add back button
    lv_obj_t *btn_back = lv_btn_create(device_list_screen);
    lv_obj_add_event_cb(btn_back, lv_back_button_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_size(btn_back, 60, 30);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_MID, 0, -13);

    lv_obj_t *label_back = lv_label_create(btn_back);
    lv_label_set_text(label_back, "scan");
    lv_obj_center(label_back);

    // Create and store the list object for future use
    device_list = lv_list_create(device_list_screen);
    lv_obj_set_size(device_list, 200, 130);
    lv_obj_align(device_list, LV_ALIGN_CENTER, 0, 10);

    // Switch to the device list screen
    //lv_scr_load(device_list_screen);  // This makes the device list screen the active screen
}

void remove_duplicates(char *added_devices[], int *size) {
    int new_index = 0;

    for (int i = 0; i < *size; i++) {
        bool found = false;

        // Check if the current string already exists in the new list
        for (int j = 0; j < new_index; j++) {
            if (strcmp(added_devices[i], added_devices[j]) == 0) {
                found = true;
                break;
            }
        }

        // If not found, add it to the unique list
        if (!found) {
            added_devices[new_index++] = added_devices[i];
        }
    }

    // Update the size to reflect the number of unique elements
    *size = new_index;
}

void update_device_list() {
    if (device_list == NULL) {
        printf("Device list object not initialized!\n");
        return;
    }
    remove_duplicates(added_devices, &device_count);
    // Clear the current list to avoid duplicates
    lv_obj_clean(device_list);
    
    // Iterate through the added devices and create list items
    for (int i = 0; i < device_count; i++) {
        if (added_devices[i] != NULL) {
            lv_obj_t *btn=lv_list_add_btn(device_list, LV_SYMBOL_BLUETOOTH, added_devices[i]);
            lv_obj_add_event_cb(btn, btn_list_event_handler, LV_EVENT_CLICKED, NULL); // Assign the same callback
            printf("Added device to list: %s\n", added_devices[i]);
        }
    }
    memset(added_devices, 0, sizeof(added_devices));
    device_count=0;
}


void lv_connecting_intermediate_page(void) {
    intermediate_screen = lv_obj_create(NULL);
    lv_obj_clean(intermediate_screen);  // Clean the current screen

    // Create the main container for the screen
    lv_obj_t *connecting = lv_obj_create(intermediate_screen);  // Use lv_scr_act() as the parent
    lv_obj_set_size(connecting, LV_HOR_RES, LV_VER_RES);  // Make it full-screen size

    // Create the label and set its text
    lv_obj_t *label = lv_label_create(connecting);
    lv_label_set_text(label, "Connecting ...");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_26, 0);


    // Center the label within the parent
    lv_obj_center(label);
}


// Device connection creation
// Default parameter values
static int param_pw = 20;
static int param_freq = 20;
static int param_amp = 20;

// Helper function to update the label text
void update_label(lv_obj_t *label, int value) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", value);
    lv_label_set_text(label, buf);
}



// Callback for '+' button click
static void btn_plus_event_handler(lv_event_t *e) {
    lv_obj_t *label = lv_event_get_user_data(e); // Get the associated label
    int *value = lv_obj_get_user_data(label);    // Get the associated parameter value

    (*value)++;                                  // Increment the value
    update_label(label, *value);                 // Update the label
    write_data(param_pw,param_freq,param_amp);
}

// Callback for '-' button click
static void btn_minus_event_handler(lv_event_t *e) {
    lv_obj_t *label = lv_event_get_user_data(e); // Get the associated label
    int *value = lv_obj_get_user_data(label);    // Get the associated parameter value

    if (*value > 0)                              // Prevent negative values
        (*value)--;                              // Decrement the value
    update_label(label, *value);                 // Update the label
    write_data(param_pw,param_freq,param_amp);

}

// Function to create the connected screen with parameters
void show_connected_screen(void) {
    if (connection_screen == NULL) {
        // Create a new screen if not already created
        connection_screen = lv_obj_create(NULL);
    }

    // Clean the current screen and load the new screen
    lv_obj_clean(lv_scr_act());
    //lv_scr_load(connection_screen);

    // Set a clean background style
    lv_obj_set_style_bg_color(connection_screen, lv_color_white(), 0);

    // Create a title at the top
    lv_obj_t *title = lv_label_create(connection_screen);
    lv_label_set_text(title, "Parameters");
    lv_obj_set_style_text_color(title, lv_color_black(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, +25);

    // Parameters array
    struct {
        const char *name;
        int *value;
    } parameters[] = {
        { "PW", &param_pw },
        { "Freq", &param_freq },
        { "Amp", &param_amp }
    };

    // Vertical spacing and alignment
    int y_offset = -30;
    for (int i = 0; i < 3; i++) {
        // Parameter name label
        lv_obj_t *name_label = lv_label_create(connection_screen);
        lv_label_set_text(name_label, parameters[i].name);
        lv_obj_set_style_text_font(name_label, &lv_font_montserrat_18, 0);
        lv_obj_align(name_label, LV_ALIGN_CENTER, -10, y_offset + i * 50);

        // Value display label
        lv_obj_t *value_label = lv_label_create(connection_screen);
        lv_obj_set_user_data(value_label, parameters[i].value); // Associate the value
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", *parameters[i].value);
        lv_label_set_text(value_label, buf);
        lv_obj_set_style_text_font(value_label, &lv_font_montserrat_18, 0);
        lv_obj_align(value_label, LV_ALIGN_CENTER, +25, y_offset + i * 50);

        // '-' Button
        lv_obj_t *btn_minus = lv_btn_create(connection_screen);
        lv_obj_set_size(btn_minus, 40, 40);
        lv_obj_align(btn_minus, LV_ALIGN_CENTER, -70, y_offset + i * 50);
        lv_obj_set_style_radius(btn_minus, 20, 0);
        lv_obj_add_event_cb(btn_minus, btn_minus_event_handler, LV_EVENT_CLICKED, value_label);
        lv_obj_t *btn_minus_label = lv_label_create(btn_minus);
        lv_label_set_text(btn_minus_label, "-");
        lv_obj_set_style_text_font(btn_minus_label, &lv_font_montserrat_16, 0);
        lv_obj_center(btn_minus_label);

        // '+' Button
        lv_obj_t *btn_plus = lv_btn_create(connection_screen);
        lv_obj_set_size(btn_plus, 40, 40);
        lv_obj_align(btn_plus, LV_ALIGN_CENTER, 70, y_offset + i * 50);
        lv_obj_set_style_radius(btn_plus, 20, 0);
        lv_obj_add_event_cb(btn_plus, btn_plus_event_handler, LV_EVENT_CLICKED, value_label);
        lv_obj_t *btn_plus_label = lv_label_create(btn_plus);
        lv_label_set_text(btn_plus_label, "+");
        lv_obj_set_style_text_font(btn_plus_label, &lv_font_montserrat_16, 0);
        lv_obj_center(btn_plus_label);
    }
}
