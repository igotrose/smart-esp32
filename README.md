# GPIO KEY 使用
更加详细的可以前往官网进行阅读
<https://docs.espressif.com/projects/esp-idf/zh_CN/v5.5/esp32s3/api-reference/peripherals/gpio.html>
## IO 配置
IO可以两钟使用方式
- 简单读取输入输出
- 作为外设信号的输出输出

IDF 外设驱动内部会处理需要应用到引脚上的必要 IO 配置，以便它们可以用作外设信号的输入或输出，使用`gpio_config()`可以配置 I/O 模式、内部上拉/下拉电阻等管脚设置
```c
esp_err_t gpio_config(const gpio_config_t *pGPIOConfig)
```
- `pGPIOConfig` GPIO结构体指针
- 返回值 `ESP_OK` 返回成功，`ESP_ERR_INVALID_ARG` 参数错误
```c
/**
 * @brief Configuration parameters of GPIO pad for gpio_config function
 */
typedef struct {
    uint64_t pin_bit_mask;          /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
    gpio_mode_t mode;               /*!< GPIO mode: set input/output mode                     */
    gpio_pullup_t pull_up_en;       /*!< GPIO pull-up                                         */
    gpio_pulldown_t pull_down_en;   /*!< GPIO pull-down                                       */
    gpio_int_type_t intr_type;      /*!< GPIO interrupt type                                  */
#if SOC_GPIO_SUPPORT_PIN_HYS_FILTER
    gpio_hys_ctrl_mode_t hys_ctrl_mode;       /*!< GPIO hysteresis: hysteresis filter on slope input    */
#endif
} gpio_config_t;
```
这是需要配置的gpio结构体，配置完之后使用`gpio_config()`进行注册
## 外部中断
外部中断可以配置在GPIO上，配置`gpio_config_t`中的`intr_type`变量就可以了；当GPIO发生变化时，可以触发中断，中断处理函数可以注册到中断控制器中，安装注册中断服务程序函数可以使用`gpio_install_isr_service()`；中断控制器可以配置中断优先级、触发类型等。另外使用`gpio_isr_handler_add()`可以将指定的GPIO分配中断处理函数
- `gpio_install_isr_service()` 安装中断服务程序函数
    ```c
    esp_err_t gpio_install_isr_service(int intr_alloc_flags)
    ```
    - `intr_alloc_flags` 外部中断分配标志，可以配置中断的优先级、触发类型等
    - 返回值 `ESP_OK` 返回成功，`ESP_ERR_NO_MEM` 内存不足

- `gpio_isr_handler_add()` 注册中断处理函数
    ```c
    esp_err_t gpio_isr_handler_add(gpio_num_t gpio_num, gpio_isr_t isr_handler, void *args)
    ```
    - `gpio_num` GPIO 引脚号    
    - `isr_handler` 中断处理函数指针
    - `args` 传递给中断处理函数的参数
    - 返回值 `ESP_OK` 返回成功，`ESP_ERR_INVALID_ARG` 参数错误，`ESP_ERR_NOT_FOUND` GPIO 引脚未注册中断服务程序
