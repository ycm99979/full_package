#!/usr/bin/env python3
"""
Motion Planner - MoveIt을 사용한 장애물 회피 경로 계획 및 실행

목표 Position을 받으면 MoveIt이 장애물을 회피하는 경로를 계획하고 실행합니다.
OMPL planner를 사용하여 collision-free path를 생성합니다.

Usage:
    # Terminal 1: Launch MoveIt demo
    ros2 launch mobile_manipulator_moveit_config demo.launch.py
    
    # Terminal 2: Run this script
    python3 src/mobile_manipulator_ws/src/mobile_manipulator_moveit_config/scripts/motion_planner.py
    
    # Terminal 3: Send target position (x, y, z)
    ros2 topic pub /goal_position geometry_msgs/msg/Point "{x: 0.0, y: 0.2, z: 0.4}" --once
    
    # Add obstacle (optional)
    ros2 topic pub /add_obstacle geometry_msgs/msg/PoseStamped "{header: {frame_id: 'arm_base_link'}, pose: {position: {x: 0.1, y: 0.1, z: 0.3}}}" --once
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint, PositionConstraint, OrientationConstraint,
    MoveItErrorCodes, BoundingVolume, CollisionObject, RobotState, PositionIKRequest
)
from moveit_msgs.srv import GetPositionIK
from shape_msgs.msg import SolidPrimitive

from rclpy.action import ActionClient

import time


class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')
        
        self.callback_group = ReentrantCallbackGroup()
        self._busy = False
        
        # Parameters
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('end_effector_link', 'gripper_base')
        self.declare_parameter('base_frame', 'arm_base_link')
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('num_planning_attempts', 10)
        self.declare_parameter('velocity_scaling', 0.3)
        self.declare_parameter('acceleration_scaling', 0.3)
        
        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.base_frame = self.get_parameter('base_frame').value
        self.planning_time = self.get_parameter('planning_time').value
        self.num_planning_attempts = self.get_parameter('num_planning_attempts').value
        self.velocity_scaling = self.get_parameter('velocity_scaling').value
        self.acceleration_scaling = self.get_parameter('acceleration_scaling').value
        
        # Current joint states
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # MoveGroup action client
        self.move_group_client = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self.callback_group
        )
        
        # IK service client
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik',
            callback_group=self.callback_group
        )
        
        # Collision object publisher
        self.collision_pub = self.create_publisher(
            CollisionObject, '/collision_object', 10
        )
        
        # Wait for action server
        self.get_logger().info('Waiting for MoveGroup action server...')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available!')
            return
        self.get_logger().info('MoveGroup action server ready!')
        
        # Wait for IK service
        self.get_logger().info('Waiting for IK service...')
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('IK service not available!')
            return
        self.get_logger().info('IK service ready!')
        
        # Arm joint names
        self.arm_joints = [
            'link2_to_link1',
            'link3_to_link2', 
            'link4_to_link3',
            'gripper_to_link4'
        ]
        
        # Subscribe to goal position topic
        self.goal_position_sub = self.create_subscription(
            Point, '/goal_position', self.goal_position_callback, 10,
            callback_group=self.callback_group
        )
        
        # Subscribe to add obstacle topic
        self.add_obstacle_sub = self.create_subscription(
            PoseStamped, '/add_obstacle', self.add_obstacle_callback, 10,
            callback_group=self.callback_group
        )
        
        # Subscribe to clear obstacles topic
        self.clear_obstacles_sub = self.create_subscription(
            String, '/clear_obstacles', self.clear_obstacles_callback, 10,
            callback_group=self.callback_group
        )
        
        # Status publisher
        self.status_pub = self.create_publisher(String, '/motion_planner_status', 10)
        
        # Obstacle counter
        self.obstacle_count = 0
        
        self.get_logger().info('='*60)
        self.get_logger().info('Motion Planner with Collision Avoidance Ready!')
        self.get_logger().info('='*60)
        self.get_logger().info('Topics:')
        self.get_logger().info('  /goal_position (Point) - target xyz position')
        self.get_logger().info('  /add_obstacle (PoseStamped) - add box obstacle')
        self.get_logger().info('  /clear_obstacles (String) - clear all obstacles')
        self.get_logger().info('')
        self.get_logger().info('Example commands:')
        self.get_logger().info('  # Move to position')
        self.get_logger().info('  ros2 topic pub /goal_position geometry_msgs/msg/Point \\')
        self.get_logger().info('    "{x: 0.0, y: 0.2, z: 0.4}" --once')
        self.get_logger().info('')
        self.get_logger().info('  # Add obstacle')
        self.get_logger().info('  ros2 topic pub /add_obstacle geometry_msgs/msg/PoseStamped \\')
        self.get_logger().info('    "{header: {frame_id: arm_base_link}, \\')
        self.get_logger().info('     pose: {position: {x: 0.1, y: 0.1, z: 0.3}}}" --once')
        self.get_logger().info('='*60)
    
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg
    
    def publish_status(self, message: str):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(f'[Status] {message}')
    
    def add_obstacle_callback(self, msg: PoseStamped):
        """Add a box obstacle to the planning scene"""
        self.obstacle_count += 1
        obstacle_id = f'obstacle_{self.obstacle_count}'
        
        collision_object = CollisionObject()
        collision_object.header = msg.header
        if not collision_object.header.frame_id:
            collision_object.header.frame_id = self.base_frame
        collision_object.id = obstacle_id
        collision_object.operation = CollisionObject.ADD
        
        # Create box primitive (10cm cube)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.1, 0.1, 0.1]  # 10cm x 10cm x 10cm
        
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(msg.pose)
        
        self.collision_pub.publish(collision_object)
        
        pos = msg.pose.position
        self.publish_status(f'Added obstacle "{obstacle_id}" at [{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}]')
    
    def clear_obstacles_callback(self, msg: String):
        """Clear all obstacles from the planning scene"""
        for i in range(1, self.obstacle_count + 1):
            collision_object = CollisionObject()
            collision_object.header.frame_id = self.base_frame
            collision_object.id = f'obstacle_{i}'
            collision_object.operation = CollisionObject.REMOVE
            self.collision_pub.publish(collision_object)
        
        self.publish_status(f'Cleared {self.obstacle_count} obstacles')
        self.obstacle_count = 0
    
    def goal_position_callback(self, msg: Point):
        """Callback for goal position - plan with collision avoidance"""
        if self._busy:
            self.get_logger().warn('Planner is busy, ignoring new goal')
            return
        
        self._busy = True
        
        try:
            self.publish_status(f'Goal received: [{msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}]')
            
            # Wait for current joint state
            if self.current_joint_state is None:
                self.publish_status('Waiting for joint states...')
                timeout = 5.0
                start = time.time()
                while self.current_joint_state is None and (time.time() - start) < timeout:
                    time.sleep(0.1)
                
                if self.current_joint_state is None:
                    self.publish_status('ERROR: No joint states received')
                    return
            
            # First compute IK to get target joint positions
            self.publish_status('Computing IK...')
            target_joints = self.compute_ik(msg)
            
            if target_joints is None:
                self.publish_status('ERROR: No IK solution found')
                return
            
            self.publish_status(f'IK solution: {[f"{j:.3f}" for j in target_joints]}')
            
            # Create and send MoveGroup goal with joint constraints
            self.publish_status('Planning collision-free path...')
            goal = self.create_joint_goal(target_joints)
            
            future = self.move_group_client.send_goal_async(goal)
            
            # Wait for goal acceptance
            timeout = 10.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.05)
            
            if not future.done():
                self.publish_status('ERROR: Goal send timeout')
                return
            
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.publish_status('ERROR: Goal rejected by MoveGroup')
                return
            
            self.publish_status('Goal accepted, planning and executing...')
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            timeout = self.planning_time + 30.0
            start = time.time()
            while not result_future.done() and (time.time() - start) < timeout:
                time.sleep(0.1)
            
            if not result_future.done():
                self.publish_status('ERROR: Execution timeout')
                return
            
            result = result_future.result()
            error_code = result.result.error_code.val
            
            if error_code == MoveItErrorCodes.SUCCESS:
                self.publish_status('SUCCESS: Motion completed!')
            else:
                error_msg = self.get_error_message(error_code)
                self.publish_status(f'ERROR: {error_msg}')
                
        except Exception as e:
            self.publish_status(f'ERROR: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self._busy = False
    
    def compute_ik(self, target_position: Point) -> list:
        """Compute IK for target position"""
        request = GetPositionIK.Request()
        
        ik_request = PositionIKRequest()
        ik_request.group_name = self.planning_group
        ik_request.robot_state.joint_state = self.current_joint_state
        ik_request.avoid_collisions = True
        ik_request.timeout.sec = 1
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position = target_position
        pose_stamped.pose.orientation.w = 1.0
        
        ik_request.pose_stamped = pose_stamped
        request.ik_request = ik_request
        
        future = self.ik_client.call_async(request)
        
        timeout = 5.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.05)
        
        if not future.done():
            return None
        
        result = future.result()
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        
        # Extract joint positions
        joint_positions = []
        for joint_name in self.arm_joints:
            if joint_name in result.solution.joint_state.name:
                idx = result.solution.joint_state.name.index(joint_name)
                joint_positions.append(result.solution.joint_state.position[idx])
            else:
                return None
        
        return joint_positions
    
    def create_joint_goal(self, target_joints: list) -> MoveGroup.Goal:
        """Create MoveGroup goal with joint constraints"""
        goal = MoveGroup.Goal()
        
        request = goal.request
        request.group_name = self.planning_group
        request.num_planning_attempts = self.num_planning_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling
        
        # Set workspace bounds
        request.workspace_parameters.header.frame_id = self.base_frame
        request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        request.workspace_parameters.min_corner.x = -1.0
        request.workspace_parameters.min_corner.y = -1.0
        request.workspace_parameters.min_corner.z = -0.5
        request.workspace_parameters.max_corner.x = 1.0
        request.workspace_parameters.max_corner.y = 1.0
        request.workspace_parameters.max_corner.z = 1.0
        
        # Create joint constraints
        constraints = Constraints()
        
        for i, joint_name in enumerate(self.arm_joints):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = target_joints[i]
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        request.goal_constraints.append(constraints)
        
        # Planning options
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        goal.planning_options.replan_delay = 0.5
        
        return goal
    
    def get_error_message(self, error_code: int) -> str:
        """Convert MoveIt error code to message"""
        error_messages = {
            MoveItErrorCodes.SUCCESS: "Success",
            MoveItErrorCodes.FAILURE: "General failure",
            MoveItErrorCodes.PLANNING_FAILED: "Planning failed - no collision-free path found",
            MoveItErrorCodes.INVALID_MOTION_PLAN: "Invalid motion plan",
            MoveItErrorCodes.CONTROL_FAILED: "Control failed",
            MoveItErrorCodes.TIMED_OUT: "Planning timed out",
            MoveItErrorCodes.START_STATE_IN_COLLISION: "Start state in collision",
            MoveItErrorCodes.GOAL_IN_COLLISION: "Goal in collision with obstacle",
            MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "Goal constraints violated",
            MoveItErrorCodes.NO_IK_SOLUTION: "No IK solution - position unreachable",
        }
        return error_messages.get(error_code, f"Unknown error ({error_code})")


def main(args=None):
    rclpy.init(args=args)
    
    node = MotionPlanner()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
