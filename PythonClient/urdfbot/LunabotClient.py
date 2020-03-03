import setup_path
import airsim
import airsim.types as at
import math

class LunabotClient():
    def __init__(self):
        self.airsim_client = airsim.UrdfBotClient()
        self.airsim_client.confirmConnection()
        self.airsim_client.enableApiControl(True)

        self.front_right_wheel_joint_pivot_name = 'top_ring_to_front_right_pivot'
        self.front_left_wheel_joint_pivot_name = 'top_ring_to_front_left_pivot'
        self.back_right_wheel_joint_pivot_name = 'top_ring_to_back_right_pivot'
        self.back_left_wheel_joint_pivot_name = 'top_ring_to_back_left_pivot'

        self.front_right_wheel_joint_name = 'front_right_pivot_to_wheel'
        self.front_left_wheel_joint_name = 'front_left_pivot_to_wheel'
        self.back_right_wheel_joint_name = 'back_right_pivot_to_wheel'
        self.back_left_wheel_joint_name = 'back_left_pivot_to_wheel'

        self.hbar_joint_name = 'top_ring_to_hbar'

        self.bucket_joint_name = 'hbar_to_bucket'

        self.pi = 3.1415926535

    def drive(self, magnitude, theta):
        if (abs(magnitude) > 1):
            raise ValueError('Speed must be between 0 and 1, provided value {0}'.format(magnitude))

        left_multiplier = self.drive_angle_multiplier(theta)
        right_multiplier = self.drive_angle_multiplier(theta - (self.pi / 2))

        front_right_update_obj = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.front_right_wheel_joint_name, control_signal_values={'Value': magnitude * right_multiplier})
        front_left_update_obj = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.front_left_wheel_joint_name, control_signal_values={'Value': magnitude * left_multiplier})
        back_right_update_obj = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.back_right_wheel_joint_name, control_signal_values={'Value': magnitude * right_multiplier})
        back_left_update_obj = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.back_left_wheel_joint_name, control_signal_values={'Value': magnitude * left_multiplier})

        self.airsim_client.updateControlledMotionComponentControlSignal(front_right_update_obj)
        self.airsim_client.updateControlledMotionComponentControlSignal(front_left_update_obj)
        self.airsim_client.updateControlledMotionComponentControlSignal(back_right_update_obj)
        self.airsim_client.updateControlledMotionComponentControlSignal(back_left_update_obj)

    def rotate_wheels(self, rotate_val):
        if (rotate_val < 0 or rotate_val > 1):
            raise ValueError('rotate_val must be between 0 and 1, provided {0}'.format(rotate_val))

        front_right_update = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.front_right_wheel_joint_pivot_name, control_signal_values={'Value': rotate_val})
        front_left_update = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.front_left_wheel_joint_pivot_name, control_signal_values={'Value': rotate_val})
        back_right_update = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.back_right_wheel_joint_pivot_name, control_signal_values={'Value': rotate_val})
        back_left_update = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.back_left_wheel_joint_pivot_name, control_signal_values={'Value': rotate_val})

        self.airsim_client.updateControlledMotionComponentControlSignal(front_right_update)
        self.airsim_client.updateControlledMotionComponentControlSignal(front_left_update)
        self.airsim_client.updateControlledMotionComponentControlSignal(back_right_update)
        self.airsim_client.updateControlledMotionComponentControlSignal(back_left_update)

    def rotate_arm(self, rotate_val):
        if (rotate_val < 0 or rotate_val > 1):
            raise ValueError('rotate_val must be between 0 and 1, provided {0}'.format(rotate_val))

        arm_update = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.hbar_joint_name, control_signal_values={'Value': rotate_val})

        self.airsim_client.updateControlledMotionComponentControlSignal(arm_update)

    def rotate_bucket(self, rotate_val):
        if (rotate_val < 0 or rotate_val > 1):
            raise ValueError('rotate_val must be between 0 and 1, provided {0}'.format(rotate_val))

        bucket_update = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.bucket_joint_name, control_signal_values={'Value': rotate_val})

        self.airsim_client.updateControlledMotionComponentControlSignal(bucket_update)

    def drive_angle_multiplier(self, theta):
        while (theta > 2 * self.pi):
            theta -= (2 * self.pi)
        while (theta < 0):
            theta += 2 * self.pi

        if (theta < self.pi / 2):
            return math.cos(theta * 2)
        elif (theta < self.pi):
            return -1.0
        elif (theta < 3 * self.pi / 2):
            return -1.0 * math.cos( (theta - self.pi) * 2)
        else:
            return 1.0
