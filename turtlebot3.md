# Turtlebot3机器人的Gazebo仿真

## 打开Gazebo仿真界面

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
rosrun map_server map_saver -f ~/map


roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml

```

## Particle Swarm Optimization插件参数说明

```yaml
## Particle Swarm Optimization(PSO) planner
  # number of particles 粒子群中的粒子数量
  n_particles: 50
  # number of inherited particles (Note: Need to be less than parameter n_ Particles)从上一次迭代继承其位置的粒子数量。它应该小于总粒子数。
  n_inherited: 20
  # number of position points contained in each particle每个粒子包含的位置点数量。看起来每个粒子有5个位置点。
  pointNum: 5
  # The maximum velocity of particle motion粒子移动的最大速度。
  max_speed: 40
  # inertia weight 用于粒子速度更新方程的惯性权重。
  w_inertial: 1.0
  # social weight 用于粒子速度更新方程的社交权重。
  w_social: 2.0
  # cognitive weight 用于粒子速度更新方程的认知权重。
  w_cognitive: 1.2
  # obstacle factor(greater means obstacles) 障碍物因子，影响障碍物对粒子运动的影响。较高的值意味着障碍物的影响更大
  obs_factor: 0.39 
  # Set the generation mode for the initial position points of the particle swarm
  # 1: Randomly generate initial positions of particle swarm within the map range
  # 2: Randomly generate initial particle swarm positions within the circular area of the starting and target points 粒子群初始位置点生成模式。可以是1或2。模式1在地图范围内随机生成初始位置，而模式2在起始点和目标点定义的圆形区域内生成初始位置
  initposmode: 2 
  # Whether to publish particles 是否发布粒子。在这种情况下，粒子不会被发布
  pub_particles: false
  # maximum iterations PSO算法的最大迭代次数。在这种情况下，设置为5。
  pso_max_iter: 5

```

## uuv_sumulator

uuv_simulator/uuv_sensor_plugins/uuv_sensor_ros_plugins/urdf/sonar_snippets.xacro

```xml
<xacro:macro name="forward_multibeam_p900" params="namespace parent_link *origin">
    <xacro:multibeam_sonar
      namespace="${namespace}"
      suffix=""
      parent_link="${parent_link}"
      topic="sonar"
      mass="0.02"
      update_rate="15" # 多波束声纳的更新频率为 15 Hz，即每秒钟发布 15 次声纳数据
      samples="512" # 多波束声纳的采样数为 512，在 FOV 内将会生成 512 条声纳射线。
      fov="1.5708"  # 多波束声纳的视场角为 1.5708 弧度（约合 90 度）
      range_min="1.0" # 最小测距为 1.0 米
      range_max="100.0" # 最大测距为 100.0 米
      range_stddev="0.027" # 多波束声纳测距的标准差为 0.027 米
      mesh="">
      <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001" />
      <xacro:insert_block name="origin" />
      <visual>
        <geometry>
          <mesh filename="file://$(find uuv_sensor_ros_plugins)/meshes/p900.dae" scale="1 1 1"/>
        </geometry>
      </visual>
    </xacro:multibeam_sonar>
  </xacro:macro>
```

```xml
  <xacro:property name="mass" value="69.7"/>
  <xacro:property name="length" value="1.98"/>
  <xacro:property name="diameter" value="0.23"/>
  <xacro:property name="radius" value="${diameter*0.5}"/>
  <xacro:property name="volume" value="0.06799987704121499"/>
  <xacro:property name="cob" value="0 0 0.06"/>
  <xacro:property name="rho" value="1027.0"/>
```

bot

```xml
 <gazebo reference="base_scan">
    <material>Gazebo/FlatBlack</material>
    <sensor type="ray" name="lds_lfcd_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>$(arg laser_visual)</visualize>
      <update_rate>15</update_rate>
      <ray>
        <scan>
          <horizontal>
            <resolution>1</resolution>

            <samples>512</samples>
            <min_angle>-0.7854</min_angle>
            <max_angle>0.7854</max_angle>

            <!-- <samples>360</samples>
            <min_angle>0.00</min_angle>
            <max_angle>6.28319</max_angle> -->


          </horizontal>
        </scan>
        <range>
          <min>1</min>
          <max>100</max>
          <!-- <max>1.5</max> -->
          <resolution>0.015</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <!-- <stddev>0.01</stddev> -->
          <stddev>0.027</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_ros_lds_lfcd_controller" filename="libgazebo_ros_laser.so">
        <topicName>scan</topicName>
        <frameName>base_scan</frameName>
      </plugin>
    </sensor>
  </gazebo>
```

