# MPU6050 串口可视化网页

本页面读取当前固件通过 UART 输出的 CSV 数据，默认串口参数为 **115200 baud、8N1**。数据列对应 `ax_g,ay_g,az_g,vibration,carrier_hz,start_stop_hz,rms_g,missed_ticks`。

## 启动

在项目根目录运行：

```powershell
py -m http.server 8000 --directory web
```

然后用最新版 Chrome 或 Edge 打开 <http://localhost:8000>，点击“连接串口”，选择开发板对应的 COM 端口，再点击“开始采集”。浏览器会询问串口访问权限。

## 页面功能

- 实时显示 Ax、Ay、Az 三轴加速度及最近 300 个点的波形。
- 显示固件计算的振动状态、载波频率、启停频率和 RMS。
- 停止采集、清空数据，并将采集记录导出为 CSV。

串口 CSV 中的三轴值是以 g 表示的加速度；载波频率由设备端 1 kHz 采样数据计算后随串口发送。
