# 功能
实现了基于ESKF的松耦合定位框架kf-fusion，通过松耦合的形式融合IMU、GNSS、外部里程计（VINS）和动捕，输出平滑、高频的定位信息（位置、姿态和速度）。
1. 通用IMU-ESKF框架，不考虑地球自转带来的非惯性系虚拟力，适用于低成本IMU。
2. 支持以松耦合的形式融合外部里程计/动捕、GNSS（后续支持），融合里程计或动捕时支持动态标定外参（杆臂）。
3. 实现了基于IMU预积分的延迟更新，相比于传统延迟更新方式更高效、平滑。

# 使用
1. 配置位于/config目录下的config.yaml参数文件，关键配置信息包括：IMU话题名、里程计/动捕话题名、是否使用动捕、里程计/动捕机体系到IMU系的外参。
2. 运行kf_fusion：
``` plain
roslaunch kf_fusion kf_fusion.launch
```
3. 若需可视化，运行：
``` plain
roslaunch kf_fusion rviz.launch
```
4. 注意，运行kf_fusion后，需静止等待5s左右完成初始化，控制台输出初始化信息后才能运动。

# 代码框架说明
以下介绍程序的关键模块，包括状态管理、初始化、ESKF框架以及系统Pipeline，请需要进行二次开发的同学仔细阅读。
## 状态管理
对每类状态单独定义一个没有继承关系的类，并使用variant对所有状态进行包装实现类型擦除，用一个map存储所有系统状态，使用variant的主要目的是避免使用基于多态的类型擦除，因为这种方法会频繁使用开销较大的dynamic_cast作下行转换，效率较低。  
最底层的状态类（一级状态类）是按照状态的数据定义和更新方式区分的，目前主要有两个状态基类，分别是OriBase（旋转）和VecBase（三维向量），状态基类记录该状态的localSize（对应误差状态的维数），OriBase维护的基本数据格式是Sophus::SO3d，更新方式为右扰动，VecBase的基本数据格式是Vector3d。  
IMU状态以及外参状态（统称为二级状态类）继承自这些状态基类，其中IMU姿态状态继承自OriBase，外参旋转状态继承自VecBase，其余继承自VecBase，二级状态类需要指定状态类型StateType（枚举值）以及排列序号。  
三级状态是利用variant封装的状态类SystemState，也是最终插入到状态池（map）的状态类型。定义用于生成SystemState的模板类，在该类中实现通用的二级状态访问操作（基于visitor）以及工厂函数。
### StateBase以及二级状态类
**重要成员变量**：  
*LocalDim*：局部维度或误差状态维度，在一级状态中定义，编译常量。  
*state_value*: 状态数值（SO3或Vector），在一级状态中定义。  
*is_constant*：是否是常量状态（涉及到是否会在kf过程中对其进行更新），在一级状态中定义。  
*start_index*：在总状态向量中的索引，在一级状态中定义，编译时确定，运行时赋值。  
*Order*：状态排列序号，在二级状态中定义，编译常量。  
**重要成员方法**：  
*update()*：输入后验误差状态，对状态进行更新。  
*localSize() / startIndex()*：返回对应成员变量的数值。  

### SystemState/DefineVariantState类
模板类，基于std::variant包装二级状态，定义了一系列编译时确定的常量成员方法，用于实现快捷简便的系统状态增删。  
**重要成员方法：**  
*orderIndex() / localSize() / update()* 等：基于visitor模式访问枚举体内部二级状态的成员变量或方法。  
*viewAs() / viewConstAs()* ：模板函数，模板参数为二级状态类型，通过std::get方法返回二级状态引用。  
*create()* : 工厂函数，传入状态初始值的地址和二级状态类型的枚举，返回创建好的三级状态SystemState。  
*sumOfLocalSize()*：编译常量成员方法，返回总的状态维数（各状态的local size之和）。  

### StateManager类
**重要成员变量：**  
*state_pool*：map类，系统状态池，键为每个状态的uid，值为variant包装的三级状态类。  
**重要成员方法：**  
*insertState()*：在系统初始化时调用，输入状态uid，状态类型枚举和初始值地址，构造状态并插入到状态池中。  
*updateState()*：kf完成后调用，输入后验误差状态向量，按顺序调用各状态的更新函数进行更新。  
*getXXXState()*：返回特定状态的状态数值引用。  

### 添加新的状态流程
1. 判断该状态是否属于OriBase或VecBase，若不是则需要自定义新的状态基类，定义方式按照已有状态基类。然后定义二级状态继承自状态基类，指定状态类型和排列序号（均为编译时常量）。
2. 在commons.h中添加枚举值，包括状态类型StateType，状态排列序号StateOrder，若是需要定义新的状态基类则还需要添加该状态基类的local_dim(local size)；指定该状态的uid，在commons.h中定义内联函数getxxxStateUid()，返回该状态的uid。
3. 在SystemState模板参数中添加二级状态类。
4. 确定该状态的初始值以及初始协方差，在kf_coordinator的init函数中通过state_manager的接口向状态池中插入该状态。

## 初始化——StateInitializer类
用于初始化kf状态，主要是导航系下的位姿、速度和零偏，为了实现灵活的初始化（基于odom或基于gps），因此单独将初始化过程封装为一个类。  
**重要成员变量：**  
*state*：StateInit类型，存储了状态的初始值，包括姿态、位置、速度、零偏、初始化时刻的imu信息。  
*imu_queue*：imu缓存队列，用于初始化零偏。  
*odom_que*：里程计队列，用于初始化除零偏以外的状态。  
*is_bias_init*：bool类型，是否完成零偏初始化。  
*is_state_init*：bool类型，是否完成所有状态的初始化。  
**重要成员方法：**  
*initState()*：初始化逻辑主要执行函数，输入观测meas，返回是否初始化成功。若输入meas为imu并且没有完成零偏初始化，则调用initGyroBias()进行零偏初始化；若输入meas为里程计并且已完成零偏初始化，则调用initByOdom()完成其他状态初始化。  
*initGyroBias()*：根据固定时间窗口的imu数据初始化陀螺仪零偏。  
*initByOdom()*：接收odom的位姿和速度，根据odom时间戳以及imu缓存队列，将odom的位姿和速度积分到最新的imu时刻，积分结果作为初始位姿和速度。  

## ESKF框架——KfCoordinator类
**重要成员变量：**  
*imu_que*：维护固定时间长度的imu队列，主要用于实现延迟更新。  
*state_manager*：状态管理器，内部维护了系统状态池，kf_coordiantor通过该成员变量获取和操作系统状态信息。  
*state_cov*: 系统状态协方差矩阵，MatrixXd类型，维度取决状态数量和各状态维数。  
*imu_noise_cov*：imu测量的噪声矩阵，包含高斯白噪声和零偏随机游走噪声，由于imu的递推采用中值积分，因此该噪声矩阵维度是18。  
*obv_residual*：观测残差向量，存储VectorXd类型的vector，容器内每个元素对应一个观测信息形成的观测残差。  
*obv_jocabian*：观测雅克比矩阵，存储MatrixXd类型的vector，容器内每个元素存对应一个观测信息对应观测模型的雅克比矩阵，行数为观测残差维度，列数为系统状态维度。注意obv_residual和obv_jocabian里内的元素必须是一一对应关系。  
*obv_R*：观测的噪声矩阵。  
*is_init*：bool类型，是否完成kf的初始化。  
**重要成员方法：**  
*insertStateWithCov()*：插入非常量状态，需指定：该状态uid，状态类型枚举，状态初始值以及初始协方差。  
*insertConstState()*：插入常量状态，不需要输入协方差，默认协方差为极小值。  
*init()*：输入state_initializer的初始化结果，调用insertStateWithCov()或insertConstState()初始化系统状态。  
*imuStatePropagate()*：imu状态递推实现，根据输入的imu信息传播imu状态和协方差，递推方法为中值积分。  
*processImuMeas()*：将新的imu测量push到队列，并控制队列的长度，调用imuStatePropagate()完成imu状态递推。  
*getDelayUpdateInfo()*：输入观测信息的观测时间，返回用于延迟更新的信息（延迟更新雅克比矩阵，延迟预积分结果）。  
*addResidualJacobian()*：输入某次观测信息对应的观测残差、观测雅克比、id_size_pair以及该观测对应的延迟更新雅克比。输入的观测残差直接push到obv_residual；输入的观测雅克比是用vector存储的，每个元素对应该观测残差对某个系统状态的雅克比，通过id_size_pair指明该雅克比关联状态的start_idx和local_size，从而在addResidualJacobian()内部实现对雅克比矩阵的扩展（列扩展为系统状态总维度），最后将扩展后的观测雅克比乘以延迟更新雅克比，push到obv_jacobian中。  
*solveAndUpdate()*：执行卡尔曼滤波的更新流程，计算卡尔曼因子k和后验误差状态，并更新系统（名义）状态和系统协方差。  
*updateWithOdom()*：输入odom观测信息，首先通过getDelayUpdateInfo得到延迟更新信息，然后计算观测残差和观测雅克比并调用addResidualJacobian()添加到obv_res和obv_jacobian中，设置本次观测的噪声矩阵，最后调用solveAndUpdate()实现kf更新。  
*updateWithZUPT()*：输入imu观测，构建零速更新的残差项和雅克比，从而实现零速更新。  

## 系统Pipeline——KfInterface类
**重要成员变量：**  
*thrd_hdl*：C11线程句柄。  
*meas_queue*：缓存队列，存储接收到的观测数据。  
*kf_coordinator*：封装了eskf相关操作。  
*state_initializer*：封装了kf初始化相关操作。  
*static_check*：零速检测器，目前实现了基于imu滑窗的静止判定，后续会被vins零速检测替代。  
*pub_xxx / sub_xxx*：ros发布和订阅句柄。  
**重要成员方法：**  
*receiveImuTopic()*：imu话题订阅回调函数，包装数据push到队列，通过条件变量唤醒处理线程。  
*receiveOdomTopic()*：odom话题订阅回调函数，包装数据push到队列，通过条件变量唤醒处理线程。  
*publishxxx()*：通过ros发布相关处理结果。  
*processMeasurements()*：作为处理线程入口函数，当被唤醒时，处理观测数据(while遍历处理队列里的所有数据)，具体逻辑:

1. 判定是否kf完成初始化，若没有初始化则调用state_initializer.initState()以及kf_coordiantor_.init()方法完成初始化。
2. 若已完成初始化，如果当前测量数据是imu，则调用kf_coordiantor.imuStatePropagate()递推状态和协方差，并调用static_check.inputImu()检测是否检测到静止，若检测到静止则调用kf_coordiantor.updateWithZUPT()进行零速更新；若是odom或gps数据，调用相关update函数实现kf更新。
3. 通过ros广播相关处理结果。

# 外参离线标定工具
kf-fusion融合外部里程计时，考虑了imu坐标系到里程计机体系之间的相对位姿即外参，但是经过实际测试发现在线标定效果较差，原因是目前的观测模型无法使外参变得可观，因此kf-fusion暂时取消在线估计外参，单独开发了一个基于手眼标定法的简易外参离线标定工具，用于标定两个相互固定且独立的里程计之间单独外参。  
## 使用
1. 在config/config.yaml中修改两个里程计的话题名，目前默认为动捕里程计话题名和vins里程计话题名：  
   ```plain
   odom_topic: "/vins_fusion/odometry"
   motion_capture_topic: "/motion_capture/motion_capture_odom"
   ```
2. 依次启动两个里程计后，再启动标定工具：  
   ```plain
   #启动vins
   sh shfiles/rspx4_xp.sh
   #启动动捕
   roslaunch motion_capture motion_capture.launch
   #启动外参标定工具
   roslaunch kf_fusion ext_calib.launch
   ```
3. 启动外参标定工具后注意终端打印的日志，出现提示“Calibration start, please move the drone fully.”后即可移动机体，保证六自由度的充分运动，程序会自动截取运动充分的数据用于结算外参。  
4. 终端会实时打印有效数据的数量，当有效数量大于50时，若持续2秒没有检测到有效数据，则会停止采集数据，进行外参解算。为了保证求解精度，建议持续运动保证有效数据数量在100以上。外参求解结果会打印在终端：  
   ```plain
   [ WARN] [1765373385.708512653]: Collecting data finished, total size is 411.
   [ WARN] [1765373385.710690551]: Start solve external pose.
   [ WARN] [1765373385.753432781]: Solve external rotation successfully!
   External rotation calib result is:
     0.996615 -0.0312618  0.0760409
    0.0341498   0.998732 -0.0369798
   -0.0747885  0.0394514   0.996419
   [ WARN] [1765373385.755121709]: Solve external position successfully!.
   External position calib result is:
   -0.0491892
     0.041708
    0.0242918
   ```