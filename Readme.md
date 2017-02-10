#xm视觉包配置说明
##Description
xm_view contains xm_object and xm_people,which you can recognize object or face ,detecting gesture or pose.
copyrights by reason_W,E-mail:
charlewander@gmail.com

##How to build in Ubuntu 14.5
###	First:
 保证你的文件路径为英文路径　如　~/Documents		
###	Second:
进入PKG_Prerequisites
```
			sh ./Autorun1.sh
```

1. 这一步如果flann提示找不到hdf5报错，记得先看一遍Autorun1.sh 文件内容
2.  查看hdf5 是否在指定安装到的/usr/local路径下有访问权限。
3.  如果没有权限，就更换hdf5文件夹至指定目录，并更新.bashrc中的变量路径
4.  安装opencv2.4版本及pcl1.7,若为1.8需到xm_vision10_5/xm_object/CmakeLists.txt,line 3 : 修改对应路径
5.  安装ros-indigo

###	Third:
1. 将Recognition 放在主目录下
2. 找到xm_object/include/init_recognize.h :: line 30 :: # mainPath="/home/charle/" # 2222222 your path
3.  找到xm_object/include/vfh.h :: line 12-14 ::# "/home/charle/" # 2222222 your path
4.  将xm_people,xm_object,xm_msgs 放到你的ros目录下  catkin_make
###        Forth
 运行
```
                  roslaunch xm_people tracker.launch 
```
1. 可开启xtion，并同时启动骨架检测，手势检测，人的位置在ros_topic/people中可以看到
2. 如果有手部动作,也可以看到(举手，投降，双手臂交叉，前推，四个动作)。 并同时发布  /camera/depth_registered/points 和/camera/rgb/image_raw 两个topic
3.   也可以运行
```
                  roslaunch openni2_launch openni2.launch 
```
 　开启xtion
###      Fifth
1. 物体数据输入，训练及识别
```
        rosrun xm_object getobject 1
```
 现在一般是　先把物品放在平面桌子上（注意不能是透明玻璃），你会在画面中看到杯子，默认是桌子以上2cm 到25cm高的东西。要保证桌子在画面中平面比例最大，每按一次r就保存一帧点云图和对应的rgb，如果画面中没有点云图显示，不要按，要先调整摄像头角度或物品位置。文件名在Recognition/object_data/photo_name.txt里保存        画面输出点云图的时候还会同时输出物品宽度和高度。退出按q再输入下一个东西                          
所有物品输入完就行。
```
        rosrun xm_object training
```
 按提示输入物品数量，按提示输入是否有保存好的物品名称，宽度和高度.都输入完，若命令行有training down输出，就完成。
```
        rosrun xm_object recognize
```
 (这里一定要先启动摄像头)
如果显示画面，就表示成功。
识别物品或人脸通过rosservice 调用。
```
        rosservicen call Find_Object
```
 会直接返回画面主平面上物品识别结果，保存在Recognition
```
       rosservice call Find_Face "'commond'= (detect ,remember,recognize)    'arguments'= (人名)  " 
```
 会返回结果。保存在Recognition/face_data
###   Sixth
方案

1. xm_people
       - /src/xtion_tracker.h ：利用openni2的库，实施检测人体骨架，位置ａｐｉ只给了四个。标定的时候不
        用做特定动作，但是稍微动一下效果会好一点。姿势api可以获取４个,上面已经说过。可以获取到１５个骨骼点的
        位置，可以跟踪手部轨迹(这两个都注释掉了).
2. xm_object       
       - /src/Filter.cpp
        - removeplane:检测物体的函数。这里找平面思路就是DominantPlaneSegmentation（分割较慢１ｓ左右）,有一个迭代，几个参数设置
        其他找平面思路是：pcl_organized_segmentation（分割快，效果不好），有color,eu,plane,edge四种 方法，颜色主要是有点慢，慢了一倍。
        - vertex4：得到点云在rgb上对应位置处矩形
        - Clusters：分割加识别，识别主要点云特征是our-cvfh,全局描述符，能应对物品倾斜旋转及小部分遮挡。       能返回欧式距离，及１５个以内的可能并排好序。识别这里目前还是通过匹配保存的每一个点云视角。而不是通过     给每一个视角人为加位姿，融合成一个真正的３ｄ图。下一步准备这么做
        - Scale：根据尺度信息过滤待选物品
     - /include/Cvdetect.h:这里是处理rgb的。因为像素很小只有１０００左右，结构信息，角点信息损失严重，使用sift,sur
      f等特征点基本为０。这里使用hsv，hist匹配。一个准备发展的思路是利用局部颜色特征的空间分部，即
       一幅图分16块，每块颜色直方图不一样。　　　　　
      - /src/init_recognize.cpp：
       识别先用点云识别一遍，有６个备选目标，然后根据备选目标的宽度高度去掉一些目标，然后根据颜色直方图那里的识别给出结果。还要根据第一次训练完的识别数据，对单个物体点云和颜色那里的权重策略做一些调整。人脸识别是对每一个人脸都找一遍以及记忆的人脸，不过因为人的不可重复性，加了一个去重的过程。
## Contact Info
If you have any peoblem on xm_vision,please contact me by sending email to
charlewander@gmail.com
