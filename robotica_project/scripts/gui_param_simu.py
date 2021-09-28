#!/usr/bin/env python3

import rospy
import subprocess
from kivymd.app import MDApp
from kivy.lang import Builder
from std_msgs.msg import Bool

class SimulationGUI(MDApp):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.screen = Builder.load_file('../gui/gui_param_simu.kv')

    def build(self):

        return self.screen

    def simulate(self, *args):
        accel_drift_x = self.screen.ids.imu_accelDrift_x.text
        accel_drift_y = self.screen.ids.imu_accelDrift_y.text
        accel_drift_z = self.screen.ids.imu_accelDrift_z.text

        accel_gau_noise_x = self.screen.ids.accelGaussianNoise_x.text
        accel_gau_noise_y = self.screen.ids.accelGaussianNoise_y.text
        accel_gau_noise_z = self.screen.ids.accelGaussianNoise_z.text

        rate_drift_x = self.screen.ids.rateDrift_x.text
        rate_drift_y = self.screen.ids.rateDrift_y.text
        rate_drift_z = self.screen.ids.rateDrift_z.text

        rate_gau_noise_x = self.screen.ids.rateGaussianNoise_x.text
        rate_gau_noise_y = self.screen.ids.rateGaussianNoise_y.text
        rate_gau_noise_z = self.screen.ids.rateGaussianNoise_z.text

        yaw_drift = self.screen.ids.yawDrift.text
        yaw_gau_noise = self.screen.ids.yawGaussianNoise.text

        vel_drift_x = self.screen.ids.velocityDrift_x.text
        vel_drift_y = self.screen.ids.velocityDrift_y.text
        vel_drift_z = self.screen.ids.velocityDrift_z.text

        vel_gau_noise_x = self.screen.ids.velocityGaussianNoise_x.text
        vel_gau_noise_y = self.screen.ids.velocityGaussianNoise_y.text
        vel_gau_noise_z = self.screen.ids.velocityGaussianNoise_z.text

        drift_x = self.screen.ids.drift_x.text
        drift_y = self.screen.ids.drift_y.text
        drift_z = self.screen.ids.drift_z.text
        
        gau_noise_x = self.screen.ids.gaussianNoise_x.text
        gau_noise_y = self.screen.ids.gaussianNoise_y.text
        gau_noise_z = self.screen.ids.gaussianNoise_z.text

        subprocess.call('gnome-terminal --execute roslaunch robotica_project display.launch imu_acc_drift_x_arg:={} imu_acc_drift_y_arg:={} imu_acc_drift_z_arg:={} imu_acc_gau_noise_x_arg:={} imu_acc_gau_noise_y_arg:={} imu_acc_gau_noise_z_arg:={} imu_rate_drift_x_arg:={} imu_rate_drift_y_arg:={} imu_rate_drift_z_arg:={} imu_rate_gau_noise_x_arg:={} imu_rate_gau_noise_y_arg:={} imu_rate_gau_noise_z_arg:={} imu_drift_yaw_arg:={} imu_gau_noise_yaw_arg:={} gps_vel_drift_x_arg:={} gps_vel_drift_y_arg:={} gps_vel_drift_z_arg:={} gps_vel_gau_noise_x_arg:={} gps_vel_gau_noise_y_arg:={} gps_vel_gau_noise_z_arg:={} gps_drift_x_arg:={} gps_drift_y_arg:={} gps_drift_z_arg:={} gps_pos_gau_noise_x_arg:={} gps_pos_gau_noise_y_arg:={} gps_pos_gau_noise_z_arg:={}'.format(accel_drift_x, accel_drift_y,accel_drift_z,accel_gau_noise_x,accel_gau_noise_y,accel_gau_noise_z,rate_drift_x,rate_drift_y,rate_drift_z,rate_gau_noise_x,rate_gau_noise_y,rate_gau_noise_z,yaw_drift,yaw_gau_noise,vel_drift_x,vel_drift_y,vel_drift_z,vel_gau_noise_x,vel_gau_noise_y,vel_gau_noise_z,drift_x,drift_y,drift_z,gau_noise_x,gau_noise_y,gau_noise_z),shell=True) 



if __name__ == '__main__':

    pub = rospy.Publisher('/button',Bool,queue_size=1)

    rospy.init_node("simulation_gui_node_", anonymous=True)

    SimulationGUI().run()