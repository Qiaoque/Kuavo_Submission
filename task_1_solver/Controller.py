CONTORLLER_PATH = "/TongVerse/biped_challenge/demo/kuavo-ros-controldev"

import sys
import os
import math
sys.path.append("/opt/ros/noetic/lib/python3/dist-packages")
sys.path.append("/usr/lib/python3/dist-packages")
sys.path.append(os.path.join(CONTORLLER_PATH, "devel/lib/python3/dist-packages"))
import time
import rospy
import subprocess
from std_msgs.msg import Float32
from kuavo_msgs.msg import jointCmd, sensorsData
import numpy as np
from typing import Dict, Any, Optional
import os
import signal
from scipy.spatial.transform import Rotation
from std_srvs.srv import SetBool, SetBoolResponse, Trigger
import termios
import tty
import select
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Twist
from ocs2_msgs.msg import mode_schedule
from kuavo_msgs.srv import SetTagPose

class Controller:
    def __init__(self):
        """初始化控制器"""
        # 存储当前action
        self.current_action = None
        
        # 用于计算加速度的历史数据
        self.last_joint_velocities = None
        self.last_time = None
        
        # 添加新的成员变量
        self.last_obs = None
        self.last_published_time = None
        self.control_freq = 500
        self.dt = 1 / self.control_freq
        
        # 添加子进程存储变量
        self.launch_process = None
        
        # 添加初始位姿配置
        self.SCENE_CFG = {
            "point_1": {"position": [11.8283, -8.7758, 0.0], "orientation": [0, 0, -180]}, # 沙地前
            "point_2": {"position": [2.3283, 9.458, -0.051], "orientation": [0, 0, -90]},  # 导航场景
            "point_3": {"position": [9.5283, 8.4758, -0.052], "orientation": [0, 0, 0]},   # 搬运场景
            "point_4": {"position": [0.2283, 8.7758, 0.0], "orientation": [0, 0, 0]},      # 上斜坡前
            "point_5": {"position": [4.9083, -8.7758, 0.0], "orientation": [0, 0, -180]},  # 上楼梯前
        }

        # 定义路径中点位绝对位置 waypoints
        # 每个 waypoint 包含 position (x, y, z) 和 orientation (四元数，可选)
        self.stage = 0
        self.next_stage = 1
        self.raw_dis = 0
        self.waypoints = [
            {'position': (8.60704, -0.59604, 0.0), 'orientation': [0, 0, +180]},  
            {'position': (6.00021, -0.56854, 0.0), 'orientation': [0, 0, +90]},  
            {'position': (5.91221,  0.494, 0.0),  'orientation': [0, 0, +90]},  
            {'position': (7.46749,  2.57522, 0.0), 'orientation': [0, 0, +90]},  
            {'position': (7.46749,  5.15675, 0.0),  'orientation': [0, 0, 0]},  
            {'position': (8.56583,  5.15675, 0.0),  'orientation': [0, 0, 0]},  
            {'position': (10.47594, 3.73898, 0.0), 'orientation': [0, 0, 0]},  # y可以给3.70673
            {'position': (13.07358, 3.53484, 0.0), 'orientation': [0, 0, -90]},  
            {'position': (13.07358, 2.50952, 0.0), 'orientation': [0, 0, -90]},
            {'position': (11.62783, 0.39815, 0.0), 'orientation': [0, 0, -90]},
            {'position': (11.66321, -0.11236, 0.0),'orientation': [0, 0, -90]}, #11.62783 -0.11236
            {'position': (13.24112, -3.03374, 0.0),'orientation': [0, 0, -90]}
        ]
        
        # PID controller参数
        # # 第一版
        # # 速度控制参数（可动态调整）
        # self.max_linear_speed = 1.0
        # self.min_linear_speed = 0.1
        # self.max_angular_speed = 0.8
        # self.pid_kp = 0.8
        # self.pid_ki = 0.02
        # self.pid_kd = 0.10
        # # 新增角度控制参数
        # self.angle_kp = 1.8
        # self.angle_ki = 0.02
        # self.angle_kd = 0.15
        # 最初版，稳定在1.7左右，应该比较稳


        # # 第二版
        # # 速度控制参数（可动态调整）
        # self.max_linear_speed = 1.0
        # self.min_linear_speed = 0.2
        # self.max_angular_speed = 0.9
        # self.pid_kp = 1.0
        # self.pid_ki = 0.005
        # self.pid_kd = 0.40 
        # # 新增角度控制参数
        # self.angle_kp = 1.6
        # self.angle_ki = 0.005
        # self.angle_kd = 0.35
        # # 最开始1.5-1.6,后来稳定在1.8左右


        # # 第三版
        # # 速度控制参数（可动态调整）
        # self.max_linear_speed = 1.0
        # self.min_linear_speed = 0.3
        # self.max_angular_speed = 0.9
        # self.pid_kp = 0.9
        # self.pid_ki = 0.01
        # self.pid_kd = 0.45
        # # 新增角度控制参数
        # self.angle_kp = 1.5
        # self.angle_ki = 0.01
        # self.angle_kd = 0.40
        # 稳定在1.6多一点，一次能跑进1.5,不太稳，6次翻两次


        # # 第四版
        # # 速度控制参数（可动态调整）
        # self.max_linear_speed = 1.0
        # self.min_linear_speed = 0.3
        # self.max_angular_speed = 0.9
        # self.pid_kp = 0.7
        # self.pid_ki = 0.008
        # self.pid_kd = 0.35
        # # 新增角度控制参数
        # self.angle_kp = 1.2
        # self.angle_ki = 0.008
        # self.angle_kd = 0.30
        # # 试了三次，翻一次，1.6左右，能跑进1.5,应该


        # # 第五版
        # # 速度控制参数（可动态调整）
        # self.max_linear_speed = 1.0
        # self.min_linear_speed = 0.6
        # self.max_angular_speed = 0.9
        # self.min_angular_speed = 0.07
        # self.pid_kp = 0.35
        # self.pid_ki = 0.008
        # self.pid_kd = 0.15
        # # 新增角度控制参数
        # self.angle_kp = 1.2
        # self.angle_ki = 0.008
        # self.angle_kd = 0.45
        # # 试了5次,1.7左右，1.70，1.6*，1.77*，1.756，1.671


        # # 第六版
        # # 速度控制参数（可动态调整）
        # self.max_linear_speed = 1.0
        # self.min_linear_speed = 0.6
        # self.max_angular_speed = 1.0
        # self.min_angular_speed = 0.07
        # self.pid_kp = 0.35
        # self.pid_ki = 0.008
        # self.pid_kd = 0.15
        # # 新增角度控制参数
        # self.angle_kp = 1.2
        # self.angle_ki = 0.008
        # self.angle_kd = 0.45
        # # 试了5次,1.7左右，1.671，1.683，1.717，1.718，1.754 1.727


        # # 第七版
        # # 速度控制参数（可动态调整）
        # self.max_linear_speed = 1.0
        # self.min_linear_speed = 0.6
        # self.max_angular_speed = 1.3
        # self.min_angular_speed = 0.07
        # self.pid_kp = 0.35
        # self.pid_ki = 0.008
        # self.pid_kd = 0.15
        # # 新增角度控制参数
        # self.angle_kp = 1.2
        # self.angle_ki = 0.008
        # self.angle_kd = 0.45
        # # 1.704，1.624，第三次死在最后了


        # 第八版
        # 速度控制参数（可动态调整）
        self.max_linear_speed = 1.0
        self.min_linear_speed = 0.6
        self.max_angular_speed = 1.1
        self.min_angular_speed = 0.07
        self.pid_kp = 0.35
        self.pid_ki = 0.008
        self.pid_kd = 0.15
        # 新增角度控制参数
        self.angle_kp = 1.2
        self.angle_ki = 0.008
        self.angle_kd = 0.45
        # 1.1: 1.68, 死一次, 1.68, 


        # 新增状态变量
        self.integral_error = 0.0
        self.derivative_filter = 0.0
        self.angle_integral = 0.0
        self.last_angle_error = 0.0
        self.last_dis = 0.0
        self.position_tolerance = 0.15  # 位置误差阈值（米）
        self.orientation_tolerance = 0.15  # 朝向误差阈值（弧度）

        # 仿真相关变量
        self.sim_running = True
        self.sensor_time = 0
        self.last_sensor_time = 0
        self.is_grab_box_demo = False
        
        # 添加按键监听相关变量
        self.old_settings = termios.tcgetattr(sys.stdin)

    def init_ros(self):
        """初始化ROS相关的组件"""
        # 初始化ROS节点
        rospy.init_node('controller', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # self.rate = rospy.Rate(10)  # 10Hz 发布频率
        rospy.wait_for_service('/humanoid_auto_gait')
        self.auto_gait_client = rospy.ServiceProxy('/humanoid_auto_gait', Trigger)
        self.gait_change_pub = rospy.Publisher('/humanoid_mpc_mode_schedule', mode_schedule, queue_size=10)


        # 发布器和订阅器
        self.sensor_pub = rospy.Publisher('/sensors_data_raw', sensorsData, queue_size=2)
        
        self.joint_cmd_sub = rospy.Subscriber('/joint_cmd', jointCmd, self.joint_cmd_callback)
        
        # 设置发布频率
        self.publish_rate = rospy.Rate(self.control_freq)  # 500Hz的发布频率
        
        # 添加仿真启动服务
        self.sim_start_srv = rospy.Service('sim_start', SetBool, self.sim_start_callback)
        
        # 添加退出处理
        rospy.on_shutdown(self.cleanup)
        
        # 添加频率统计的发布器
        self.freq_pub = rospy.Publisher('/controller_freq', Float32, queue_size=10)

    def start_launch(self, reset_ground: bool = True) -> None:
        """启动指定的命令行命令并保持进程存活
        
        Args:
            reset_ground: 是否重置地面高度，默认为True
        """
        # 使用bash执行命令
        command = f"bash -c 'source {CONTORLLER_PATH}/devel/setup.bash && roslaunch humanoid_controllers load_kuavo_isaac_sim.launch reset_ground:={str(reset_ground).lower()}'"
        print(command)
        try:
            # 使用shell=True允许执行完整的命令字符串，并将输出直接连接到当前终端
            self.launch_process = subprocess.Popen(
                command,
                shell=True,
                stdout=None,  
                stderr=None,
                stdin=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            rospy.loginfo(f"Successfully started command: {command}")
            
            # 检查进程是否立即失败
            if self.launch_process.poll() is not None:
                raise Exception(f"Process failed to start with return code: {self.launch_process.returncode}")
                
        except Exception as e:
            rospy.logerr(f"Failed to start command: {str(e)}")
            if self.launch_process is not None:
                try:
                    os.killpg(os.getpgid(self.launch_process.pid), signal.SIGTERM)
                except:
                    pass
                self.launch_process = None
        # 初始化ROS相关组件
        self.init_ros()
    
    def process_obs(self, obs: Dict[str, Any], republish = False) -> None:
        """处理观测数据并发布传感器数据
        
        Args:
            obs: 观测数据字典，包含IMU和关节状态信息
        """
        sensor_data = sensorsData()
        
        # 设置时间戳
        current_time = rospy.Time.now()
        sensor_data.header.stamp = current_time
        sensor_data.header.frame_id = "world"
        sensor_data.sensor_time = rospy.Duration(self.sensor_time)
        if republish:
            pass
            # self.sensor_time += self.dt
        else:
            self.sensor_time += obs["imu_data"]["imu_time"] - self.last_sensor_time
        self.last_sensor_time = obs["imu_data"]["imu_time"]
        # print(f"sensor_time: {self.sensor_time}")
        # 处理IMU数据
        if "imu_data" in obs:
            imu_data = obs["imu_data"]
            sensor_data.imu_data.acc.x = imu_data["linear_acceleration"][0]
            sensor_data.imu_data.acc.y = imu_data["linear_acceleration"][1]
            sensor_data.imu_data.acc.z = imu_data["linear_acceleration"][2]
            sensor_data.imu_data.gyro.x = imu_data["angular_velocity"][0]
            sensor_data.imu_data.gyro.y = imu_data["angular_velocity"][1]
            sensor_data.imu_data.gyro.z = imu_data["angular_velocity"][2]
            sensor_data.imu_data.quat.w = imu_data["orientation"][0]
            sensor_data.imu_data.quat.x = imu_data["orientation"][1]
            sensor_data.imu_data.quat.y = imu_data["orientation"][2]
            sensor_data.imu_data.quat.z = imu_data["orientation"][3]

        # 处理关节数据
        if "Kuavo" in obs and "joint_state" in obs["Kuavo"]:
            joint_state = obs["Kuavo"]["joint_state"]
            
            # 初始化关节数据数组
            sensor_data.joint_data.joint_q = [0.0] * 28
            sensor_data.joint_data.joint_v = [0.0] * 28
            sensor_data.joint_data.joint_vd = [0.0] * 28
            sensor_data.joint_data.joint_current = [0.0] * 28

            # 处理腿部数据
            if "legs" in joint_state:
                legs_data = joint_state["legs"]
                legs_pos = legs_data["positions"]
                legs_vel = legs_data["velocities"]
                legs_effort = legs_data["applied_effort"]
                
                for i in range(6):
                    # 左腿
                    sensor_data.joint_data.joint_q[i] = legs_pos[i*2]
                    sensor_data.joint_data.joint_v[i] = legs_vel[i*2]
                    sensor_data.joint_data.joint_current[i] = legs_effort[i*2]
                    # 右腿
                    sensor_data.joint_data.joint_q[i+6] = legs_pos[i*2+1]
                    sensor_data.joint_data.joint_v[i+6] = legs_vel[i*2+1]
                    sensor_data.joint_data.joint_current[i+6] = legs_effort[i*2+1]

            # 处理手臂数据
            if "arms" in joint_state:
                arms_data = joint_state["arms"]
                arms_pos = arms_data["positions"]
                arms_vel = arms_data["velocities"]
                arms_effort = arms_data["applied_effort"]
                
                for i in range(7):
                    # 左臂
                    sensor_data.joint_data.joint_q[i+12] = arms_pos[i*2]
                    sensor_data.joint_data.joint_v[i+12] = arms_vel[i*2]
                    sensor_data.joint_data.joint_current[i+12] = arms_effort[i*2]
                    # 右臂
                    sensor_data.joint_data.joint_q[i+19] = arms_pos[i*2+1]
                    sensor_data.joint_data.joint_v[i+19] = arms_vel[i*2+1]
                    sensor_data.joint_data.joint_current[i+19] = arms_effort[i*2+1]

            # 处理头部数据
            if "head" in joint_state:
                head_data = joint_state["head"]
                head_pos = head_data["positions"]
                head_vel = head_data["velocities"]
                head_effort = head_data["applied_effort"]
                
                for i in range(2):
                    sensor_data.joint_data.joint_q[26+i] = head_pos[i]
                    sensor_data.joint_data.joint_v[26+i] = head_vel[i]
                    sensor_data.joint_data.joint_current[26+i] = head_effort[i]

        # 发布传感器数据
        self.sensor_pub.publish(sensor_data)
     
    def joint_cmd_callback(self, msg: jointCmd) -> None:
        """处理关节命令回调
        
        Args:
            msg: 关节命令消息
        """
        # 构建action字典，按照README.md中的格式
        action = {
            "arms": {
                "ctrl_mode": "position",
                "joint_values": np.zeros(14),  # 14 arm joints
                "stiffness": [100.0] * 14 if self.is_grab_box_demo else [200.0] * 14,  # 搬箱子需要更低刚度的手臂
                "dampings": [20.2, 20.2, 20.5, 20.5, 10.2, 10.2, 20.1, 20.1, 10.1, 10.1, 10.1, 10.1, 10.1, 10.1],
            },
            "legs": {
                "ctrl_mode": "effort",
                "joint_values": np.zeros(12),  # 12 leg joints
                "stiffness": [0.0] * 12,  # Not setting stiffness
                "dampings": [0.2] * 12,  # Not setting dampings
            },
            "head": {
                "ctrl_mode": "position",
                "joint_values": np.zeros(2),  # 2 head joints
                "stiffness": None,  # Not setting stiffness
                "dampings": None,  # Not setting dampings
            }
        }

        # 处理腿部力矩数据
        for i in range(6):
            action["legs"]["joint_values"][i*2] = msg.tau[i]        # 左腿
            action["legs"]["joint_values"][i*2+1] = msg.tau[i+6]    # 右腿

        # 处理手臂位置数据
        for i in range(7):
            action["arms"]["joint_values"][i*2] = msg.joint_q[i+12]    # 左臂
            action["arms"]["joint_values"][i*2+1] = msg.joint_q[i+19]  # 右臂
        # action["arms"]["joint_values"][1] = 100
        # print(action["arms"]["joint_values"])
        # 处理头部位置数据（如果有的话）
        if len(msg.joint_q) >= 28:  # 确保消息中包含头部数据
            action["head"]["joint_values"][0] = msg.joint_q[26]  # 头部第一个关节
            action["head"]["joint_values"][1] = msg.joint_q[27]  # 头部第二个关节

        # 更新当前action
        self.current_action = action

    def sim_start_callback(self, req: SetBool) -> SetBoolResponse:
        """仿真启动服务的回调函数
        
        Args:
            req: SetBool请求，data字段为True表示启动仿真，False表示停止仿真
            
        Returns:
            SetBoolResponse: 服务响应
        """
        response = SetBoolResponse()
        
        self.sim_running = req.data
        
        if req.data:
            rospy.loginfo("Simulation started")
        else:
            rospy.loginfo("Simulation stopped")
        
        response.success = True
        response.message = "Simulation control successful"
        
        return response

    def get_action(self, obs: Dict[str, Any]) -> Optional[Dict]:
        """获取当前action，如果没有新的action则等待并持续发布上一次的观测数据
        
        Args:
            obs: 当前的观测数据字典
            
        Returns:
            当前action字典或None
        """

    ###注：先更新last_obs，再更新current_action，最后发布传感器数据
        # 更新最新的观测数据
        self.process_obs(obs)
        
        # 如果没有收到action，持续发布上一次的观测数据
        # self.current_action = None # 清空当前action
        st = time.time()
        while self.current_action is None and not rospy.is_shutdown():
            # 发布传感器数据
            self.process_obs(self.last_obs,republish=True)
            
            # 等待一个发布周期
            self.publish_rate.sleep()
           
        freq = Float32()
        freq.data = 1
        self.freq_pub.publish(freq)
        
        return self.current_action

    def PID_vel(self, dx, dy, dt):
        """PID位置控制"""
        dis = math.sqrt(dx**2 + dy**2)
        
        # 计算比例项
        P = self.pid_kp * dis
        
        # 计算积分项（带积分限幅）
        self.integral_error += dis * dt
        self.integral_error = np.clip(self.integral_error, -0.5, 0.5)  # 积分限幅
        I = self.pid_ki * self.integral_error
        
        # 计算微分项（带低通滤波）
        derivative = (dis - self.last_dis) / dt
        self.derivative_filter = 0.8 * self.derivative_filter + 0.2 * derivative  # 一阶低通滤波
        D = self.pid_kd * self.derivative_filter
        
        # PID输出
        velocity = P + I + D
        
        # 速度限幅
        return np.clip(velocity, self.min_linear_speed, self.max_linear_speed)

    def PID_angle(self, diff_rot, dt):
        """PID角度控制"""
        # 计算比例项
        P = self.angle_kp * diff_rot
        
        # 计算积分项
        self.angle_integral += diff_rot * dt
        self.angle_integral = np.clip(self.angle_integral, -0.5, 0.5)  # 积分限幅
        I = self.angle_ki * self.angle_integral
        
        # 计算微分项
        D = self.angle_kd * (diff_rot - self.last_angle_error) / dt
        
        # PID输出
        angular_vel = P + I + D

        if angular_vel == 0:
            angular_vel = 0
        elif angular_vel > 0:
            angular_vel = np.clip(angular_vel, self.min_angular_speed, self.max_angular_speed)
        else:
            angular_vel = np.clip(angular_vel, -self.max_angular_speed, -self.min_angular_speed)
        
        # 角速度限幅
        return angular_vel

    def vel_position_control(self, dx, dy):
        def get_a(bottom, high):
            return bottom - high     # 根据下限计算a
        bottom = 0.11
        high = 1.15
        dis = math.sqrt(dx**2 + dy**2)
        x = 1 - dis / self.raw_dis
        a = get_a(bottom, high)
        y = a * (x**2) + high
        return y
    
    def vel_rotation_control(self, diff_rot, raw_rot):
        def get_a(bottom, high):
            return bottom - high     # 根据下限计算a
        # if abs(diff_rot) < (15 / 180) * np.pi:
        return ((diff_rot * 1.7) / np.pi) if np.abs((diff_rot * 1.7) / np.pi) > 0.09  else (0.09 if diff_rot > 0 else -0.09) # yaw 方向角速度
        # else:
        # bottom = 0.07
        # high = 1.6
            # x = 1 - (diff_rot / raw_rot) if (diff_rot / raw_rot) > 0 else 1 + (diff_rot / raw_rot)
            # print(f'diff_rot: {diff_rot}, raw_rot: {raw_rot}')
            # z = get_a(bottom, high) * (x**2) + high
            # return abs(z) if diff_rot > 0 else -abs(z)

    def quat_to_euler(self, quat):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw) in radians.
        Quaternion format: [x, y, z, w] (scalar last).
        Euler angle convention: static XYZ (extrinsic), i.e., rotate around X, then Y, then Z.
        
        Args:
            quat (list or tuple): Quaternion [x, y, z, w]
        
        Returns:
            list: Euler angles [roll, pitch, yaw] in radians
        """
        # Extract components
        w, x, y, z = quat
        
        # Normalize quaternion to ensure unit length
        norm = math.sqrt(w**2 + x**2 + y**2 + z**2)
        if norm == 0:
            raise ValueError("Invalid quaternion: zero norm")
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
        
        # Compute roll (rotation about X-axis)
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        
        # Compute pitch (rotation about Y-axis)
        sin_pitch = 2 * (w * y - z * x)
        # Clamp to [-1, 1] to avoid numerical errors
        sin_pitch = max(min(sin_pitch, 1.0), -1.0)
        pitch = math.asin(sin_pitch)
        
        # Compute yaw (rotation about Z-axis)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        
        return [roll, pitch, yaw]

    def PID_run(self):
        """执行移动到当前 stage 对应的目标点"""
        if self.stage >= len(self.waypoints):
            rospy.loginfo("所有目标点已完成！")
            return

        wp = self.waypoints[self.stage]
        target_position = np.array(wp['position'])
        target_orientation = wp['orientation']

        Kuavo = self.last_obs["Kuavo"]
        pos = Kuavo["body_state"]["world_position"]
        quat = Kuavo["body_state"]["world_orient"]
        rpy = self.quat_to_euler(quat)

        # calculate the difference between current position and the goal
        dx = target_position[0] - pos[0]
        dy = target_position[1] - pos[1]

        # calculate the target direction
        target_dir = np.arctan2(dy, dx)

        # calculate the difference between the target direction and the current direction
        diff_rot = target_dir - rpy[2]

        # round the rotation difference, to avoid the agent to rotate more than 180 degrees
        diff_rot = (diff_rot + np.pi) % (2 * np.pi) - np.pi

        # current_time = rospy.get_time()
        # self.last_control_time = current_time
        # dt = current_time - self.last_control_time if self.last_control_time else 0.01

        # 创建Twist消息对象
        cmd_vel_msg = Twist()
        if (self.next_stage == 1):
            self.raw_dis = math.sqrt(dx**2 + dy**2)
            self.raw_rot = diff_rot
            self.next_stage = 0

        current_time = rospy.Time.now()
        if self.last_time is None:
            dt = self.dt / 2  # 首次初始化时用固定值
        else:
            dt = (current_time - self.last_time).to_sec() / 2
        self.last_time = current_time  # 关键：更新上次时间
        # print(f'dt: {dt}')

        # if the agent is nearly facing the goal, move forward
        if math.sqrt(dx**2 + dy**2) < self.position_tolerance:
            self.stage =  self.stage + 1
            self.next_stage = 1
            print(f'进入stage: {self.stage}')
            # print(f'目标点为: x: {self.waypoints[self.stage]["position"][0]}, y: {self.waypoints[self.stage]["position"][1]}')

        elif abs(diff_rot) < np.pi / 45:
            self.last_time = rospy.Time.now()
            if self.stage == 11:
                cmd_vel_msg.linear.x = 0.8  # x 方向速度
            else:
                # 使用PID位置控制
                cmd_vel_msg.linear.x = self.PID_vel(dx, dy, dt)
            self.last_dis = math.sqrt(dx**2 + dy**2)  # 更新距离记录
            self.cmd_vel_pub.publish(cmd_vel_msg)
            # print(f"move xword: {cmd_vel_msg.linear.x}")
        
        else:
            # 使用PID角度控制
            cmd_vel_msg.angular.z = self.PID_angle(diff_rot, dt)
            self.last_angle_error = diff_rot  # 更新角度误差记录
            self.cmd_vel_pub.publish(cmd_vel_msg)
            # print(f"rotate +zword: {cmd_vel_msg.angular.z}")

    def run(self):
        """执行移动到当前 stage 对应的目标点"""
        if self.stage >= len(self.waypoints):
            rospy.loginfo("所有目标点已完成！")
            return

        wp = self.waypoints[self.stage]
        target_position = np.array(wp['position'])
        target_orientation = wp['orientation']

        Kuavo = self.last_obs["Kuavo"]
        pos = Kuavo["body_state"]["world_position"]
        quat = Kuavo["body_state"]["world_orient"]
        # rpy = Rotation.from_quat(quat).as_euler('xyz', degrees=False)  # 返回 [roll, pitch, yaw]（弧度）
        rpy = self.quat_to_euler(quat)
        # rpy = t3d.euler.quat2euler(quat,axes='sxyz')

        # calculate the difference between current position and the goal
        dx = target_position[0] - pos[0]
        dy = target_position[1] - pos[1]

        # calculate the target direction
        target_dir = np.arctan2(dy, dx)

        # calculate the difference between the target direction and the current direction
        diff_rot = target_dir - rpy[2]

        # round the rotation difference, to avoid the agent to rotate more than 180 degrees
        diff_rot = (diff_rot + np.pi) % (2 * np.pi) - np.pi

        # 创建Twist消息对象
        cmd_vel_msg = Twist()
        if (self.next_stage == 1):
            self.raw_dis = math.sqrt(dx**2 + dy**2)
            self.raw_rot = diff_rot
            self.next_stage = 0
        # if the agent is nearly facing the goal, move forward
        if math.sqrt(dx**2 + dy**2) < self.position_tolerance:
            self.stage =  self.stage + 1
            self.next_stage = 1
            print(f'进入stage: {self.stage}')
            # print(f'目标点为: x: {self.waypoints[self.stage]["position"][0]}, y: {self.waypoints[self.stage]["position"][1]}')

        elif abs(diff_rot) < np.pi / 45:
            # call bipedal controller to get joint effort given a target velocity
            if self.stage == 11:
                cmd_vel_msg.linear.x = 1.1  # x 方向速度
            else:
                cmd_vel_msg.linear.x = self.vel_position_control(dx, dy)
                # cmd_vel_msg.linear.x = 1.2 * (math.sqrt(dx**2 + dy**2) / self.raw_dis) if  1.2 * (math.sqrt(dx**2 + dy**2) / self.raw_dis) > 0.25 else 0.2 # x 方向速度
            cmd_vel_msg.linear.y = 0.0  # y 方向速度
            cmd_vel_msg.linear.z = 0.0  # 增量高度
            cmd_vel_msg.angular.z = 0.0  # yaw 方向角速度
            self.cmd_vel_pub.publish(cmd_vel_msg)
            # print(f"move xword: {cmd_vel_msg.linear.x}")
        
        
        else:
            # call bipedal controller to get joint effort given a target angular velocity
            cmd_vel_msg.linear.x = 0.0  # x 方向速度
            cmd_vel_msg.linear.y = 0.0  # y 方向速度
            cmd_vel_msg.linear.z = 0.0  # 增量高度
            # cmd_vel_msg.angular.z = ((diff_rot * 1.8) / np.pi) if np.abs((diff_rot * 1.8) / np.pi) > 0.07  else (0.07 if diff_rot > 0 else -0.07) # yaw 方向角速度
            # cmd_vel_msg.angular.z = ((diff_rot * 1.8) / self.raw_rot) if np.abs((diff_rot * 1.8) / self.raw_rot) > 0.07  else (0.07 if diff_rot > 0 else -0.07) # yaw 方向角速度
            cmd_vel_msg.angular.z = self.vel_rotation_control(diff_rot, self.raw_rot)
            self.cmd_vel_pub.publish(cmd_vel_msg)
            # print(f"rotate +zword: {cmd_vel_msg.angular.z}")

    def cleanup(self):
        """清理资源，在节点关闭时调用"""
        if self.launch_process is not None:
            try:
                rospy.loginfo("Cleaning up launch process...")
                os.killpg(os.getpgid(self.launch_process.pid), signal.SIGTERM)
                self.launch_process.wait()
                self.launch_process = None
                rospy.loginfo("Launch process cleaned up")
            except Exception as e:
                rospy.logerr(f"Error cleaning up launch process: {str(e)}")
            
            # 清理爬楼梯进程
            if hasattr(self, 'stair_process') and self.stair_process is not None:
                try:
                    rospy.loginfo("Cleaning up stair climbing process...")
                    os.killpg(os.getpgid(self.stair_process.pid), signal.SIGTERM)
                    self.stair_process.wait()
                    self.stair_process = None
                    rospy.loginfo("Stair climbing process cleaned up")
                except Exception as e:
                    rospy.logerr(f"Error cleaning up stair climbing process: {str(e)}")

            # 清理抓箱子进程
            if hasattr(self, 'grab_box_process') and self.grab_box_process is not None:
                try:
                    rospy.loginfo("Cleaning up grab box process...")
                    os.killpg(os.getpgid(self.grab_box_process.pid), signal.SIGTERM)
                    self.grab_box_process.wait()
                    self.grab_box_process = None
                    rospy.loginfo("Grab box process cleaned up")
                except Exception as e:
                    rospy.logerr(f"Error cleaning up grab box process: {str(e)}")

    def init_robot_pose(self, robot, init_pos = None, init_yaw: float = 0, height: float = 0.82):
        """初始化机器人姿态
        
        Args:
            robot: 机器人对象
            height: 机器人初始高度，默认为0.82
        """
        # 获取当前机器人位姿
        robot_init_pos, robot_init_orient = robot.get_world_pose()
        print(f"robot_init_pos: {robot_init_pos}")
        print(f"robot_init_orient: {robot_init_orient}")
        # 从二维数组中提取四元数值
        quat = robot_init_orient[0]  # [w, x, y, z]
        current_euler = Rotation.from_quat([quat[0], quat[1], quat[2], quat[3]]).as_euler("xyz", degrees=True)
        
        print(f"current_euler: {current_euler}")
        # 设置roll=0, pitch=3度, 保持原来的yaw角度
        new_euler = [0, 3, init_yaw]  # [roll, pitch+3, yaw], 拿不到正确的yaw
        initial_quat = Rotation.from_euler("xyz", new_euler, degrees=True).as_quat()[[3, 0, 1, 2]]
        
        # 设置位置和姿态，使用传入的高度值
        if init_pos is None:
            init_pos = [robot_init_pos[0][0], robot_init_pos[0][1], height]  # 从二维数组中提取位置值
        else:
            init_pos = [init_pos[0], init_pos[1], height]
        robot.set_world_pose(init_pos, initial_quat)
        
        print("Robot pose set to:", init_pos, initial_quat) 
        return init_pos, initial_quat