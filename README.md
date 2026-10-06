# 综技赛：柔性振动检测模块

ESP32-C3 与 MPU6050 柔性振动检测固件，以及通过串口实时显示三轴加速度和振动频率的网页。

## 项目内容

- `main/`：ESP-IDF 固件，I2C 读取 MPU6050 三轴加速度，并通过串口输出振动状态、载波频率、启停频率和 RMS。
- `web/`：浏览器串口可视化页面，支持实时波形、数据表格和 CSV 导出。

## 硬件连接

- MPU6050 I2C 地址：`0x68`
- ESP32-C3 SDA：GPIO 8
- ESP32-C3 SCL：GPIO 9
- 串口：115200 baud，8N1

## 编译固件

在 ESP-IDF 环境下，于项目根目录执行：

```shell
idf.py set-target esp32c3
idf.py build
idf.py flash monitor
```

固件通过串口输出 CSV，字段为：

```text
ax_g,ay_g,az_g,vibration,carrier_hz,start_stop_hz,rms_g,missed_ticks
```

## 启动网页

在项目根目录执行：

```powershell
py -m http.server 8000 --directory web
```

使用最新版 Chrome 或 Edge 打开 <http://localhost:8000>，连接开发板串口后开始采集。完整说明见 [`web/README.md`](web/README.md)。

> 浏览器显示的载波频率和启停频率由 ESP32-C3 固件计算；网页记录的是固件以串口发送的遥测数据。
