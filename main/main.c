#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"

// Espressif libraries
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_mac.h"

// Wifi
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_http_client.h"   // HTTP client
#include "lwip/inet.h"        // ip4addr_ntoa_r

// Esp_LCD Library
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"


// LCD
#include "lvgl.h"
#include "lv_demos.h"
#include "esp_lcd_sh8601.h"
#include "lcd_handler.h"

// Touchscreen
#include "esp_lcd_touch_ft5x06.h"
#include "esp_io_expander_tca9554.h"

#include "wifi_details.h" // the Wifi network name and password is in here
#include "ca_cert.h" // TLS cert
int retry_num=0;
static char ip_str[16] = {0};

lv_obj_t * connection_info;
lv_obj_t *feedList;

// RSS
#define RSS_READ_CHUNK 512

void gui_create_list()
{
    feedList = lv_list_create(lv_scr_act());
    lv_obj_set_size(feedList, 350, 350);
    lv_obj_align(feedList, LV_ALIGN_TOP_MID, 0, 20);
}

static void print_rss_titles(const char *xml)
{
    const char *p = xml;
    while ((p = strstr(p, "<item")) != NULL)
    {
        // move to after <item ...>
        const char *item_end = strstr(p, "</item>");
        if (!item_end) break;
        const char *title_start = strstr(p, "<title>");
        if (title_start && title_start < item_end)
        {

            title_start += strlen("<title>");
            const char *title_end = strstr(title_start, "</title>");

            if (title_end && title_end < item_end)
            {
                size_t len = title_end - title_start;
                char *title = malloc(len + 1);

                if (title)
                {
                    memcpy(title, title_start, len);
                    title[len] = '\0';
                    printf("RSS Title: %s\n", title);
                    
                    // Add new item to the list
                    lv_obj_t *row = lv_list_add_btn(feedList, NULL, title);

                    static lv_style_t item_style;
                    lv_style_set_min_height(&item_style, 80);
                    lv_style_set_text_font(&item_style, &lv_font_montserrat_24);
                    lv_obj_add_style(row, &item_style, 0);

                    free(title);
                }
            }
        }
        p = item_end + strlen("</item>");
    }
}


static void fetch_rss_task(void *arg)
{
    const char *url = (const char*) arg;
    
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 5000,
        .cert_pem = ca_cert_pem
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    
    // Open connection manually
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        printf("Failed to open connection\n");
        esp_http_client_cleanup(client);
        vTaskDelete(NULL);
        return;
    }
    
    // Fetch headers
    int content_length = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    printf("HTTP Status: %d\n", status);

    gui_create_list();
    
    // NOW read the response body
    char chunk[RSS_READ_CHUNK];
    char *response = NULL;
    size_t total = 0;
    int read_len = 0;
    
    printf("Reading RSS feed...\n");
    while ((read_len = esp_http_client_read(client, chunk, sizeof(chunk))) > 0)
    {
        char *tmp = realloc(response, total + read_len + 1);
        if (!tmp) {
            printf("Out of memory\n");
            free(response);
            break;
        }
        response = tmp;
        memcpy(response + total, chunk, read_len);
        total += read_len;
    }
    
    if (response) {
        response[total] = '\0';
        printf("Received %u bytes\n", (unsigned)total);
        print_rss_titles(response);
        free(response);
    }
    
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    vTaskDelete(NULL);
}

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    /*lv_obj_t * connection_info = lv_label_create(lv_scr_act());
    //lv_label_set_text(label, "LakyWatch RSS Reader");
    lv_obj_set_style_text_font(connection_info, &lv_font_montserrat_22, 0);
    lv_obj_align(connection_info, LV_ALIGN_CENTER, 0, 0);*/

    if(event_id == WIFI_EVENT_STA_START)
    {
        printf("Connecting to Wifi network...\n");
    }

    else if(event_id == WIFI_EVENT_STA_CONNECTED)
    {
        printf("Connected to Wifi network!\n");
        lv_label_set_text(connection_info, "Connected!");
    }

    else if(event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        printf("Disconnected from Wifi network!\n");
        

        if(retry_num < 5)
        {
            esp_wifi_connect();
            retry_num++;

            printf("Trying to reconnect... %dth attempt\n", retry_num);
            lv_label_set_text_fmt(connection_info, "Connection failed!\nTrying to reconnect...\nAttempt %d\n", retry_num);
        }
    }

    else if(event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ip4addr_ntoa_r((const ip4_addr_t *)&event->ip_info.ip, ip_str, sizeof(ip_str));

        printf("IP address acquired from Wifi network: %s\n\n", ip_str);

        // update your LVGL label if you want
        lv_label_set_text_fmt(connection_info, "Connected!\nIP: %s", ip_str);

        // Read an RSS feed
        xTaskCreate(&fetch_rss_task, "fetch_rss", 8*1024, (void*)"https://hackaday.com/feed/", 5, NULL);
    }
}

void wifi_connection()
{

    connection_info = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_font(connection_info, &lv_font_montserrat_22, 0);
    lv_obj_align(connection_info, LV_ALIGN_CENTER, 0, 0);

    esp_netif_init(); //Network interface init
    esp_event_loop_create_default(); // Event loop for the connection
    esp_netif_create_default_wifi_sta(); // Data structrures for the connection

    // Initialize Wifi
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation);

    // Register Wifi events
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_config_t wifi_configuration ={
        .sta = {
            .ssid = "",
            .password = ""
        }
    };

    // Copy SSID and password into the wifi configuration
    strcpy((char*)wifi_configuration.sta.ssid, ssid);
    strcpy((char*)wifi_configuration.sta.password, password);

    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);

    // Start Wifi and connect to network
    lv_label_set_text_fmt(connection_info, "Connecting to network\n%s...", ssid);
    esp_wifi_start();
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_connect();
}

void app_main(void)
{
    esp_log_level_set("lcd_panel.io.i2c", ESP_LOG_NONE);
    esp_log_level_set("FT5x06", ESP_LOG_NONE);
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    ESP_LOGI(TAG, "Initialize I2C bus");
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_NUM_TOUCH_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = PIN_NUM_TOUCH_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 200 * 1000,
    };
    ESP_ERROR_CHECK(i2c_param_config(TOUCH_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(TOUCH_HOST, i2c_conf.mode, 0, 0, 0));

    esp_io_expander_handle_t io_expander = NULL;
    esp_io_expander_new_i2c_tca9554(TOUCH_HOST, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000, &io_expander);

    ESP_ERROR_CHECK(esp_io_expander_new_i2c_tca9554(TOUCH_HOST, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000, &io_expander));

    esp_io_expander_set_dir(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1 | IO_EXPANDER_PIN_NUM_2, IO_EXPANDER_OUTPUT);
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0, 0);
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_1, 0);
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_2, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0, 1);
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_1, 1);
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_2, 1);

#if PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif

    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(PIN_NUM_LCD_PCLK,
                                                                 PIN_NUM_LCD_DATA0,
                                                                 PIN_NUM_LCD_DATA1,
                                                                 PIN_NUM_LCD_DATA2,
                                                                 PIN_NUM_LCD_DATA3,
                                                                 LCD_H_RES * LCD_V_RES * LCD_BIT_PER_PIXEL / 8);
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(PIN_NUM_LCD_CS,
                                                                                example_notify_lvgl_flush_ready,
                                                                                &disp_drv);
    sh8601_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_LOGI(TAG, "Install SH8601 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if USE_TOUCH

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
    // Attach the TOUCH to the I2C bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)TOUCH_HOST, &tp_io_config, &tp_io_handle));

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = PIN_NUM_TOUCH_RST,
        .int_gpio_num = PIN_NUM_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    ESP_LOGI(TAG, "Initialize touch controller");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp));

#endif

#if PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
#endif

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(LCD_H_RES * LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(LCD_H_RES * LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * LVGL_BUF_HEIGHT);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.rounder_cb = example_lvgl_rounder_cb;
    disp_drv.drv_update_cb = example_lvgl_update_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

#if USE_TOUCH
    static lv_indev_drv_t indev_drv; // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;
    lv_indev_drv_register(&indev_drv);
#endif

    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);
    xTaskCreate(example_lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    nvs_flash_init();

    ESP_LOGI(TAG, "Display LVGL demos");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (example_lvgl_lock(-1))
    {
        wifi_connection();

        lv_obj_t * label = lv_label_create(lv_scr_act());
        lv_label_set_text(label, "LakyWatch RSS Reader");
        lv_obj_set_style_text_font(label, &lv_font_montserrat_28, 0);
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 20);

        // Release the mutex
        example_lvgl_unlock();
    }
}