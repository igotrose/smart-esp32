#include "drv_spi_lcd.h"
#include "drv_i2c_io_expander.h"

static char* TAG = "DRV_SPI_BUS";
static spi_bus_config_t spi_config = { 0 };
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_io_spi_config_t io_config = { 0 };
static esp_lcd_panel_dev_config_t panel_config = { 0 };
static esp_lcd_panel_handle_t panel_handle = NULL;

esp_err_t dev_lcd_backlight_init(void)
{
    const ledc_channel_config_t lcd_backlight_channel = {
        .gpio_num = DEV_LCD_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = DEV_LCD_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = true,
    };
    const ledc_timer_config_t lcd_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = 1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t ret = ledc_timer_config(&lcd_backlight_timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure backlight timer");
        return ret;
    }

    ret = ledc_channel_config(&lcd_backlight_channel);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure backlight channel");
        return ret;
    }
    return ESP_OK;
}

esp_err_t dev_spi_bus_init(void)
{
    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_config.miso_io_num = BSP_SPI_MISO;
    spi_config.mosi_io_num = BSP_SPI_MOSI;
    spi_config.sclk_io_num = BSP_SPI_SCK;
    spi_config.quadwp_io_num = GPIO_NUM_NC;
    spi_config.quadhd_io_num = GPIO_NUM_NC;
    spi_config.max_transfer_sz = DEV_LCD_H_RES * DEV_LCD_V_RES * sizeof(uint16_t);

    esp_err_t ret = spi_bus_initialize(BSP_SPI_HOST, &spi_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return ret;
    }

    return ESP_OK;
}


esp_err_t dev_spi_lcd_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initialize LCD backlight");
    ret = dev_lcd_backlight_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize LCD backlight");
        return ret;
    }

    ESP_LOGI(TAG, "Initialize SPI bus");
    ret = dev_spi_bus_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return ret;
    }

    ESP_LOGI(TAG, "Initialize Panel IO");
     
    io_config.dc_gpio_num = DEV_LCD_DC;
    io_config.cs_gpio_num = BSP_SPI_CS;
    io_config.pclk_hz = 10000000;
    io_config.lcd_cmd_bits = DEV_LCD_CMD_BITS;
    io_config.lcd_param_bits = DEV_LCD_PARAM_BITS;
    io_config.spi_mode = 2;
    io_config.trans_queue_depth = 10;

    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_SPI_HOST, &io_config, &io_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize panel IO");
        return ret;
    }

    ESP_LOGI(TAG, "Initialize LCD driver");
    panel_config.reset_gpio_num = DEV_LCD_RST;
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_config.bits_per_pixel = DEV_LCD_BITS_PER_PIXEL;

    ret = esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize LCD driver");
        return ret;
    }
    esp_lcd_panel_reset(panel_handle);
    vTaskDelay(pdMS_TO_TICKS(10));

    dev_io_expander_set_output_value(LCD_CS, 0);

    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_invert_color(panel_handle, true);
    esp_lcd_panel_swap_xy(panel_handle, true);
    esp_lcd_panel_mirror(panel_handle, true, false);

    return ESP_OK;
}

void dev_lcd_set_color(uint16_t color)
{
    uint16_t* buffer = (uint16_t*)heap_caps_malloc(DEV_LCD_H_RES * sizeof(uint16_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (buffer == NULL)
    {
        ESP_LOGE(TAG, "Memory for bitmap is not enough");
    }
    else
    {
        for (int i = 0; i < DEV_LCD_H_RES; i++)
        {
            buffer[i] = color;
        }
        for (int y = 0; y < 240; y++)
        {
            esp_lcd_panel_draw_bitmap(panel_handle, 0, y, 320, y+1, (void*)buffer);
        }
        free(buffer);
    }
}

void dev_lcd_draw_picture(int x_start, int y_start, int x_end, int y_end, const unsigned char* gImage)
{
    size_t pixels_byte_size = (x_end - x_start) * (y_end - y_start) * sizeof(uint16_t);
    uint16_t* pixels = (uint16_t*)heap_caps_malloc(pixels_byte_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (pixels == NULL)
    {
        ESP_LOGE(TAG, "Memory for bitmap is not enough");
        return;
    }
    memcpy(pixels, gImage, pixels_byte_size);
    esp_lcd_panel_draw_bitmap(panel_handle, x_start, y_start, x_end, y_end, (void*)pixels);
    heap_caps_free(pixels);
}

esp_err_t dev_lcd_backlight_set(int brightness_percent)
{
    esp_err_t ret;

    if (brightness_percent > 100)
    {
        brightness_percent = 100;
    }
    else if (brightness_percent < 0)
    {
        brightness_percent = 0;
    }

    ESP_LOGI(TAG, "Set LCD backlight brightness to %d%%", brightness_percent);

    uint32_t duty_cycle = (1023 * brightness_percent) / 100;

    ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, DEV_LCD_LEDC_CH, duty_cycle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set duty cycle");
        return ret;
    }

    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, DEV_LCD_LEDC_CH);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to update duty cycle");
        return ret;
    }

    return ESP_OK;
}

esp_err_t dev_lcd_backlight_on(void)
{
    return dev_lcd_backlight_set(100);
}

esp_err_t dev_lcd_backlight_off(void)
{
    return dev_lcd_backlight_set(0);
}

void aiot_esp32_s3_05_demo_spi_lcd(void)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Start SPI LCD demo");
    ret = esp_lcd_panel_disp_on_off(panel_handle, true);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to turn on display");
        return;
    }
    ret = dev_lcd_backlight_on();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to turn on backlight");
        return;
    }

    ESP_LOGI(TAG, "Set color");
    dev_lcd_set_color(0xf800);

    // ESP_LOGI(TAG, "Draw picture");
    // dev_lcd_draw_picture(0, 0, 240, 240, (unsigned char*)logo_en_240x240_lcd);
    
}
