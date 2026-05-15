# 安装指南

## 1. 基本说明
televuer这个项目来源于宇树遥操作项目xr_teleoperate的创建者。该项目主要实现的是从Apple Vison Pro处接收数据并实现相应的读取和转发。

但是本人在此项目基础上做了一定更改。具体如下：

（1）通过protobuf这一开源库实现了Apple Vision Pro原始数据的序列化，主要是为了方便后续数据的统一处理。

（2）写了一些脚本放在scripts目录下。其中最关键的是两个脚本，send_offline_data和send_online_data。作用也如他们的命名方式，是实现对数据的离线和在线发送。这两个脚本的代码写的还是比较容易看懂的，脚本前面的部分也有一些基本参数的配置项，基本不用修改。

（3）通过send_online_data发送的数据都会进行相应的存储，放在项目的data目录下，命名的方式是send_online_data脚本运行的时间。

（4）针对send_online_data得到的数据可以在send_offline_data中指定文件路径来离线发送。

## 2. 安装说明
项目目录下的README.md里写的十分清楚，所以这里不再过多赘述。

基本的流程就是：

（1）创建python版本为3.10的conda环境
（2）到项目目录下pip install -e .
（3）执行openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout key.pem -out cert.pem生成许可证。中间要输入信息的话其实可以不用填，一路直接回车即可。