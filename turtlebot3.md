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

