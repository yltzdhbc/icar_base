# icar_base
Icar机器人下位机程序代码



## 与ICAR相关的仓库

[icar](https://github.com/yltzdhbc/icar.git)：icar机器人ros基础功能包

[icar_multi](https://github.com/yltzdhbc/icar_multi.git)：icar多机编队ros功能包

[icar_base](https://github.com/yltzdhbc/icar_base.git)：icar下位机程序（arduino mega2560）

[icar_manufacture](https://github.com/yltzdhbc/icar_manufacture.git)：icar本体制造相关文件（结构、相关硬件）



## 项目介绍

该项目基于 arduino mega2560

使用platformio开发，需要先安装vscode 与 platformio插件

### 编译

使用platform编译，通过串口连接开发板，下载

### 使用

连接ROS，使用rosserial软件包连接

