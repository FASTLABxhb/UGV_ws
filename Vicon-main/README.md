# Vicon动捕避坑指南

## Windows篇

### 连接

1. 如果电脑卡在Windows logo界面一分钟以上，果断重启，若再不行，就开机按F11进入bios，然后啥也不干，F10保存并退出，此时重启应该就可以了。（虽然我也觉得很扯，但它迄今为止就是每次都能work，so just don't ask and make life easier.

2. 检查网线连接并确认：

   ![image-20201003234956825](https://github.com/ZJU-FAST-Lab/Vicon/blob/main/README.assets/image-20201003234956825.png)

   ![image-20201003234937938](https://github.com/ZJU-FAST-Lab/Vicon/blob/main/README.assets/image-20201003234937938.png)

   

3. 如果打开Vicon软件之后有相机无法被加载，就关闭软件并重启Vicon交换机和路由器之后打开软件，若还不行，再来一次。

### 标定

1. 标定完之后画面中可能有许多噪声，先不要把待追踪物体放入动捕有效范围内。先在system界面ctrl+选中所有相机，然后到calibrate界面最上面有个条可以调整mask，调到画面中没有噪声即可。

   ![d742aa091d12b195083ca07936e8787](https://github.com/ZJU-FAST-Lab/Vicon/blob/main/README.assets/d742aa091d12b195083ca07936e8787.png)

2. 两个标定方式：

   ![f0b0ef53dfbf363200279083623b5f6](https://github.com/ZJU-FAST-Lab/Vicon/blob/main/README.assets/f0b0ef53dfbf363200279083623b5f6.png)

3. 常规操作：挥动标定杆。绕着实验室挥动标定杆一周，对着每一个摄像头逐个挥动。摄像头的状态由慢闪变为快闪表示其接收到的帧数越来越多，直到其指示灯变为绿色可换另外一个摄像头。将可能运动到的空间都走一遍。

4. 如果碰到最后一个摄像头十分顽固，无论如何挥动其有效帧都变化不大的情况，则重新标定。

5. 所有灯都变过绿色后即可。查看image error是否小于0.3，若image error为0.3~0.5则重新挥动，若为0.5以上或image error为红色，则先再标定一次，实在不行调整对应摄像头角度之后**对每个相机reset calibration**，或者调整摄像头参数（暂时没用到，也还不知道怎么搞）。

### 设置原点

设置原点之前再去一下噪，然后设置原点。

### 物体选取

把物体放进有效区域之后应该可以看到对应数量的动捕球，按住alt+鼠标左键画矩形框框起来即可，左边选到object最下面输入物体名称后create即可。

## Linux篇

1. 连接到vicon的路由器上并且把ip设置为192.168.10.2（如果网内已有这个ip就设为192.168.10.3.....顺延），网关255.255.254.0。（无论有线无线都设为这个ip）设置完一定要重启网络使其生效，然后用ipconfig/ifconfig看看ip是不是改了

2. 从https://github.com/ZJU-FAST-Lab/Vicon下载Vicon_bridge包。ethz提供的代码中的pose都是TransformStamped格式，这个格式在rviz里面是不可见的，所以我们改了一下消息格式，并且把launch文件里的 "datastream_hostport" 改为了192.168.10.1:801。

3. ```
   roslaunch vicon_bridge vicon.launch
   ```

解决~了

