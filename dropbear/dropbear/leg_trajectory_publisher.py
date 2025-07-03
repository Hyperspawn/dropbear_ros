#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class LegJTCPublisher(Node):

    def __init__(self):
        super().__init__('leg_jtc_publisher_node')
        
        # Display ASCII art and welcome message
        self.display_welcome()
        
        # Get user input for leg and joints
        self.leg:str
        self.joints:str = []

        # Publish topic name
        self.publish_topic:str

        self.get_logger().info("🦵 DROPBEAR LEG CONTROL INTERFACE")
        self.get_logger().info("=" * 50)
        self.get_logger().info("Enter 'right' or 'left' for the leg to control:")
        self.leg = input().lower()
        if self.leg == 'right':
            self.get_logger().info("✅ Leg: Right Leg")
            self.get_logger().info("🎯 Controlled Joints: RL_hip_joint")
            self.joints = ["RL_hip_joint"]
            # Topic to publish to
            self.publish_topic = "/right_leg_hip_joint_controller/joint_trajectory"
        elif self.leg == 'left':
            self.get_logger().info("✅ Leg: Left Leg")
            self.get_logger().info("🎯 Controlled Joints: LL_hip_joint")
            self.joints = ["LL_hip_joint"]
            # Topic to publish to
            self.publish_topic = "/left_leg_hip_joint_controller/joint_trajectory"
        else:
            self.get_logger().error("❌ Invalid leg selection. Exiting...")
            sys.exit(1)

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
║  🦵 Leg Trajectory Control Interface                                        ║
║  📡 Publishing to: /[right/left]_leg_hip_joint_controller/joint_trajectory ║
║  ⚡ Control Rate: 1Hz trajectory updates                                    ║
║  🔧 Joints: hip_joint                                                       ║
╚══════════════════════════════════════════════════════════════════════════════╝
        """
        
        print(ascii_art)
        print(welcome_text)
    
    def display_joint_info(self):
        """Display joint control information"""
        joint_info = f"""
╔══════════════════════════════════════════════════════════════════════════════╗
║                           🎯 LEG CONTROL GUIDE                              ║
╠══════════════════════════════════════════════════════════════════════════════╣
║                                                                              ║
║  {self.leg.upper()}_hip_joint: Hip joint control for leg movement           ║
║                                                                              ║
║  💡 Tips:                                                                    ║
║     • Start with small values (0.1 to 0.5)                                  ║
║     • Use 0 for neutral position                                             ║
║     • Monitor robot balance in Gazebo/RViz                                  ║
║     • Coordinate with pelvic girdle for stability                           ║
║                                                                              ║
║  ⚠️  Safety:                                                                 ║
║     • Ensure robot is properly grounded                                     ║
║     • Use pelvic girdle control for balance                                ║
║     • Start with conservative movements                                     ║
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
    node = LegJTCPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
