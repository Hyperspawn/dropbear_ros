#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class PelvicJTCPublisher(Node):

    def __init__(self):
        super().__init__('pelvic_jtc_publisher_node')
        
        # Display ASCII art and welcome message
        self.display_welcome()
        
        # Get user input for hand and joints
        self.leg:str
        self.joints:str = []

        # Publish topic name
        self.publish_topic:str = "/waist_joint_controller/joint_trajectory"

        self.joints = ["PG_left_leg_roll", "PG_left_leg_pitch", "PG_right_leg_roll", "PG_right_leg_pitch"]

        # Creating the trajectory publisher
        self.trajectory_publisher = self.create_publisher(JointTrajectory, self.publish_topic, 10)
        timer_period = 1.0
        # Creating a timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Display joint information
        self.display_joint_info()
        
        # Prompting user for input for each joint
        self.goal_positions = []
        self.get_logger().info("📝 Enter joint positions:")
        self.get_logger().info("-" * 30)
        for joint in self.joints:
            try:
                position = float(input(f"Enter the target position for {joint}: "))
                self.goal_positions.append(position)
            except ValueError:
                self.get_logger().warn(f"⚠️  Invalid input for {joint}. Defaulting to 0.0")
                self.goal_positions.append(0.0)
    
    def display_welcome(self):
        """Display ASCII art and welcome message"""
        ascii_art = """
                                                  --                                                
                                             ::::......                                             
                                           -::.........:                                            
                                           @%@-....=###=                                            
                                           %%%%=::-*%#%*#                                           
                                            =+%@@@@@%*#@@                                           
                                            ==++++++#%%+=                                           
                                             +=+=--:+#*=                                            
                                              --==+ ===+==                                          
                                     =--=-::-=-==--+##-+-=+#                                        
                                      =+##+=+-:=*+-+#*:---:-=:.                                     
                                       --=*#::====-:-=:-=---=-.::                                   
                                 :::::-==---+::--==----==-:-=-::::                                  
                                -::-::::==----:**+=----==+==*+:::-                                  
                                 --+-:::====++:++#***=::-+**#%+---                                  
                                 ----::-+++#%%%%%%#**#+:*#####+*=-                                  
                                 ----==:==+*#=#*++##=-::-=#+=-----                                  
                                 -:--==-=-=-===++=+#%*-:=++%%+----                                  
                                 -:::::-  +==+*%#**#+-*#+*#+*+=:::                                  
                                 -:::::-   %*%*===+%%%%#%@#=:==-::                                  
                                 --::::    %%%-::-%@%%%# %*--++---                                  
                                 -:::::-    %%*=--*+=%%# %%#=++---                                  
                                  ::----   -:+#+###=-%##%%# =+*#+-                                  
                                  -==-:   =---=-==**=*:+++ ----=--                                  
                                 --+**-- =+=::+=++*+--:-:: --::::-                                  
                                 ::-=-::#%%+=+*:=*#*=--==--------                                   
                                  :::::-+==-:-+**++#*::-=-+#+=++=                                   
                                  -----=+***+#%%#-:-=-::+--#%%##%%                                  
                                   ==*+*=-:=+#***=:-=+==*###%%%%%%                                  
                                  ######  %%####+==++=--=#%%%%%%##                                  
                                  %%%%%% =#%%#%#+::=+++=:=#%#+--*                                   
                                  #%%%%%+=****%%#-:+@%++:*##*=::                                    
                                   +#+==----:*%#*=:++*++=-----:-                                    
                                      ==---+:+==--:=+==-==------=                                   
                                      ======:::----=++=---------                                    
                                        +%+=-------=*+=--------=                                    
                                        ++==------==++==-------+                                    
                                        ==-==-----== ++=-----==                                     
                                        ==-==---===  =+====--=                                      
                                         =--=+====    ==--=-:                                       
                                         +==-+:--     =----::-                                      
                                           -:----    =-=-=---                                       
                                          -:-:--=    =--:-:                                         
                                          -:--=::     -=::::                                        
                                           ::-:::     --=--                                         
                                            -----     --===                                         
                                            ----=    --=+%#                                         
                                           ---+##   =--:-#*                                         
                                          =--:=#*   ==-::+=                                         
                                          =-:::*=-- =-=-=##                                         
                                          ----=#+=- =--:-#=--                                       
                                          =--:=#+-=  --::+===                                       
                                           --::+==   -=-+#+=                                        
                                           -=-+#+=   ----#+=                                        
                                           -=-=#==   --=-=-=                                        
                                           - --+==  =--==--=                                        
                                           - =---=  =-:-----                                        
                                          =-:----=  ++--=::-==                                      
                                          =-.:---=-  *##+-::::::                                    
                                          =*++=-::::-   +=-::::--                                   
                                             **-::::::-   =---=+                                    
                                               =-----=+                                             
        """
        
        welcome_text = """
╔══════════════════════════════════════════════════════════════════════════════╗
║                           🤖 DROPBEAR ROS 2 🤖                              ║
║                        Humanoid Robot Control System                        ║
║                                                                              ║
║  ⚙️  Pelvic Girdle Control Interface                                        ║
║  📡 Publishing to: /waist_joint_controller/joint_trajectory                 ║
║  ⚡ Control Rate: 1Hz trajectory updates                                    ║
║  🔧 Joints: PG_left_leg_roll, PG_left_leg_pitch, PG_right_leg_roll,        ║
║            PG_right_leg_pitch                                               ║
╚══════════════════════════════════════════════════════════════════════════════╝
        """
        
        print(ascii_art)
        print(welcome_text)
    
    def display_joint_info(self):
        """Display joint control information"""
        joint_info = """
╔══════════════════════════════════════════════════════════════════════════════╗
║                        🎯 PELVIC GIRDLE CONTROL GUIDE                       ║
╠══════════════════════════════════════════════════════════════════════════════╣
║                                                                              ║
║  PG_left_leg_roll:    Left leg roll control (side-to-side movement)        ║
║  PG_left_leg_pitch:   Left leg pitch control (forward/backward tilt)        ║
║  PG_right_leg_roll:   Right leg roll control (side-to-side movement)       ║
║  PG_right_leg_pitch:  Right leg pitch control (forward/backward tilt)       ║
║                                                                              ║
║  💡 Purpose:                                                                 ║
║     • High-torque back actuators for leg spreading                          ║
║     • Maintains base position and stability                                 ║
║     • Controls leg orientation and stance                                   ║
║     • Essential for bipedal balance and locomotion                         ║
║                                                                              ║
║  💡 Tips:                                                                    ║
║     • Start with small values (0.1 to 0.5)                                  ║
║     • Use 0 for neutral position                                             ║
║     • Coordinate with leg controls for stability                            ║
║     • Monitor robot balance in Gazebo/RViz                                  ║
║                                                                              ║
║  ⚠️  Safety:                                                                 ║
║     • High-torque system - use conservative values                          ║
║     • Ensure robot is properly grounded                                     ║
║     • Coordinate with other control systems                                 ║
║     • Start with small movements                                             ║
║                                                                              ║
╚══════════════════════════════════════════════════════════════════════════════╝
        """
        print(joint_info)
    
    # Timer callback function
    def timer_callback(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=2)
        trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info("🚀 Trajectory Sent!")

def main(args=None):
    rclpy.init(args=args)
    node = PelvicJTCPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
