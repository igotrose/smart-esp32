# I2C IMU
## I2C 接口API
具体可以参考乐鑫官方文档   
<https://docs.espressif.com/projects/esp-idf/zh_CN/v5.5.1/esp32s3/api-reference/peripherals/i2c.html>
## IMU 
IMU使用的是QMI8658，设备7 位 I2C 地址是 `0x6A`，具体功能可以查阅数据手册   
[QMI8658A Datasheet Rev A.pdf](<QMI8658A Datasheet Rev A.pdf>)   
### 待实现的功能以及接口
- 姿态解算
- 运动检测
- 计步器
- 跌落检测
- 低功耗检测
