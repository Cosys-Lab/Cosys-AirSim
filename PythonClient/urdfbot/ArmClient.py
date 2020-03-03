import setup_path
import airsim
import airsim.types as at
import math

class ArmClient():
    def __init__(self):
        self.airsim_client = airsim.UrdfBotClient()
        self.airsim_client.confirmConnection();
        self.airsim_client.enableApiControl(True)

        self.platform_joint_name = 'base_to_platform'
        self.first_joint_name = 'platform_to_base_sphere'
        self.second_joint_name = 'first_link_to_first_link_sphere'
        self.third_joint_name = 'second_link_to_second_link_sphere'
        self.fourth_joint_name = 'second_link_sphere_to_final_link'
        self.manipulator_left_joint_name = 'manipulator_base_to_left_manipulator'
        self.manipulator_right_joint_name = 'manipulator_base_to_right_manipulator'

    def set_arm_pose(self, platform_signal, first_signal, second_signal, third_signal, fourth_signal):
        if (platform_signal < 0 or platform_signal > 1):
            raise ValueError('Platform_signal is {0}, which is outside the expected range of [0, 1]'.format(platform_signal))
        if (first_signal < 0 or first_signal > 1):
            raise ValueError('First_signal is {0}, which is outside the expected range of [0, 1]'.format(first_signal))
        if (second_signal < 0 or second_signal > 1):
            raise ValueError('Second_signal is {0}, which is outside the expected range of [0, 1]'.format(second_signal))
        if (third_signal < 0 or third_signal > 1):
            raise ValueError('Third_signal is {0}, which is outside the expected range of [0, 1]'.format(third_signal))
        if (fourth_signal < 0 or fourth_signal > 1):
            raise ValueError('Fourth_signal is {0}, which is outside the expected range of [0, 1]'.format(fourth_signal))

        platform_update_obj = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.platform_joint_name, control_signal_values={'Value': platform_signal})
        first_update_obj = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.first_joint_name, control_signal_values={'Value': first_signal})
        second_update_obj = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.second_joint_name, control_signal_values={'Value': second_signal})
        third_update_obj = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.third_joint_name, control_signal_values={'Value': third_signal})
        fourth_update_obj = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.fourth_joint_name, control_signal_values={'Value': fourth_signal})

        self.airsim_client.updateControlledMotionComponentControlSignal(platform_update_obj)
        self.airsim_client.updateControlledMotionComponentControlSignal(first_update_obj)
        self.airsim_client.updateControlledMotionComponentControlSignal(second_update_obj)
        self.airsim_client.updateControlledMotionComponentControlSignal(third_update_obj)
        self.airsim_client.updateControlledMotionComponentControlSignal(fourth_update_obj)

    def set_manipulator(self, magnitude):
        if (magnitude < 0 or magnitude > 1):
            raise ValueError('Magnitude is {0}, which is outside the range [0, 1]'.format(magnitude));

        left_update_obj = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.manipulator_left_joint_name, control_signal_values={'Value': magnitude})
        right_update_obj = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.manipulator_right_joint_name, control_signal_values={'Value': magnitude})

        self.airsim_client.updateControlledMotionComponentControlSignal(left_update_obj)
        self.airsim_client.updateControlledMotionComponentControlSignal(right_update_obj)
