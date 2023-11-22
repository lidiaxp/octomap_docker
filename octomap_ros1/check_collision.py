import rospy
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist

import itertools
import numpy as np
import pandas as pd

from privileged_network.expert import prepare_trajectory, lrs_reduce_nodes


class OctomapChecker:
    def __init__(self):
        self.obs_x, self.obs_y, self.obs_z, self.obs_xyz = [], [], [], []
        self.curr_pos_x, self.curr_pos_y, self.curr_pos_z, self.curr_pos_yaw = [], [], [], []
        self.path_x, self.path_y, self.path_z, self.r_speeds = [], [], [], []
        # TODO pass to yaml
        self.goal_x, self.goal_y, self.goal_z = 39, 39, 5

        # Subscribers
        rospy.Timer(rospy.Duration(0.1), self.tick)
        _ = rospy.Subscriber(f"/uav1/odometry/odom_main", Odometry, self.callbackPosition)
        _ = rospy.Subscriber(f"/occupied_cells_vis_array", MarkerArray, self.callbackBuildMap3D_octomap)
        _ = rospy.Subscriber(f"/lrs/last_path", MultiDOFJointTrajectory, self.callbackLastPaths)

        self.new_path_pub = rospy.Publisher("/lrs/new_paths", MultiDOFJointTrajectory, queue_size=1)


    def get_last_speed(self):
        _curr_pos = np.array([self.curr_pos_x, self.curr_pos_y, self.curr_pos_z])
        _path = np.array([self.path_x, self.path_y, self.path_z])

        distances = np.linalg.norm(_curr_pos - _path, axis=1)
        min_index = np.argmin(distances)

        return self.r_speeds[min_index]


    # ---------------------------- Loop :3 ----------------------------------
    def tick(self, data):
        is_collision = self.check_collision()

        if is_collision:
            last_speed = self.get_last_speed()
            times, positions, speeds, reference_speeds = prepare_trajectory(
                start_node=[self.curr_pos_x, self.curr_pos_y, self.curr_pos_z],
                goal_node=[self.goal_x, self.goal_y, self.goal_z],
                use_yaml=False,
                options=['rain_forest', 24],
                plot_graph=0,
                initial_speed=last_speed,
                input_obstacles=self.obs_xyz
            )
            pos_x = positions[0, :]
            pos_y = positions[1, :]
            pos_z = positions[2, :]

            speed_x = speeds[0, :]
            speed_y = speeds[1, :]
            speed_z = speeds[2, :]

            new_path_x, new_path_y, new_path_z, new_speed_x, new_speed_y, new_speed_z, new_r_speed, new_times = lrs_reduce_nodes(
                pos_x, pos_y, pos_z, speed_x, speed_y, speed_z, reference_speeds, times
            )

            traj_msg = MultiDOFJointTrajectory()
            for i in range(new_path_x):
                point = MultiDOFJointTrajectoryPoint()

                trans = Transform()
                speed = Twist()

                trans.translation.x = new_path_x[i]
                trans.translation.y = new_path_y[i]
                trans.translation.z = new_path_z[i]

                speed.linear.x = new_speed_x[i]
                speed.linear.y = new_speed_y[i]
                speed.linear.z = new_speed_z[i]

                speed.angular.x = new_r_speed[i]
                speed.angular.y = new_times[i]

                point.transforms.append(trans)
                point.velocities.append(speed)

            self.new_path_pub.publish(traj_msg)


    def callbackPosition(self, odom):
        _, _, yaw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])        
        
        self.curr_pos_x = odom.pose.pose.position.x
        self.curr_pos_y = odom.pose.pose.position.y 
        self.curr_pos_z = odom.pose.pose.position.z 
        self.curr_pos_yaw = yaw


    def callbackBuildMap3D_octomap(self, data):
        for marker in data.markers:
            positions = marker.points
            for position in positions:
                round_pos_x = round(position.x)
                round_pos_y = round(position.y)
                round_pos_z = round(position.z)
                pos_to_add = [round_pos_x, round_pos_y, round_pos_z]

                if pos_to_add not in self.abc:
                    self.obs_x.append(round_pos_x)
                    self.obs_y.append(round_pos_y)
                    self.obs_z.append(round_pos_z)
                    self.obs_xyz.append(pos_to_add)

    
    def callbackLastPaths(self, data):
        self.path_x, self.path_y, self.path_z, self.r_speeds = [], [], [], []
        for point in data.points:
            for transform in point.transforms:
                self.path_x.append(transform.translation.x)
                self.path_y.append(transform.translation.y)
                self.path_z.append(transform.translation.z)

            for transform in point.velocities:
                self.r_speeds.append(transform.angular.x)

    
    def check_collision(self, collision_distance: float = 1):
        data = {
            'obs_x': self.obs_x,
            'traj_x': self.path_x,
            'obs_y': self.obs_y,
            'traj_y': self.path_y,
            'obs_z': self.obs_z,
            'traj_z': self.path_z 
        }

        df = pd.DataFrame(data)

        comb_x = list(itertools.product(df['traj_x'], df['obs_x']))
        diff_x = np.asarray([abs(item[1] - item[0]) for item in comb_x])

        comb_y = list(itertools.product(df['traj_y'], df['obs_y']))
        diff_y = np.asarray([abs(item[1] - item[0]) for item in comb_y])

        comb_z = list(itertools.product(df['traj_z'], df['obs_z']))
        diff_z = np.asarray([abs(item[1] - item[0]) for item in comb_z])

        all_dists = np.sum([diff_x, diff_y, diff_z], axis=0)
        collision_indexes = np.where(all_dists < collision_distance)[0]

        ##### if it needs to return with indexes #####
        # combinations_x_full = list(itertools.product(enumerate(df['traj_x']), enumerate(df['obs_x'])))
        # comb_x = [((x1, x2)) for (_, x1), (_, x2) in combinations_x_full]
        # indexes_x = np.asarray([((i1, i2)) for (i1, _), (i2, _) in combinations_x_full])
        # diff_x = np.asarray([abs(item[1] - item[0]) for item in comb_x])

        # combinations_y_full = list(itertools.product(enumerate(df['traj_y']), enumerate(df['obs_y'])))
        # comb_y = [((x1, x2)) for (_, x1), (_, x2) in combinations_y_full]
        # indexes_y = np.asarray([((i1, i2)) for (i1, _), (i2, _) in combinations_y_full])
        # diff_y = np.asarray([abs(item[1] - item[0]) for item in comb_y])

        # combinations_z_full = list(itertools.product(enumerate(df['traj_z']), enumerate(df['obs_z'])))
        # comb_z = [((x1, x2)) for (_, x1), (_, x2) in combinations_z_full]
        # indexes_z = np.asarray([((i1, i2)) for (i1, _), (i2, _) in combinations_z_full])
        # diff_z = np.asarray([abs(item[1] - item[0]) for item in comb_z])

        # all_dists = np.sum([diff_x, diff_y, diff_z], axis=0)
        # collision_indexes = np.where(all_dists < collision_distance)[0]

        ##### if it needs the specific point with collision in the trajectory #####
        # # traj_x_collision_indexes = indexes_x[collision_indexes][:, 0]
        # # obs_x_collision_indexes = indexes_x[collision_indexes][:, 1]

        # # traj_y_collision_indexes = indexes_y[collision_indexes][:, 0]
        # # obs_y_collision_indexes = indexes_y[collision_indexes][:, 1]

        # # traj_z_collision_indexes = indexes_z[collision_indexes][:, 0]
        # # obs_z_collision_indexes = indexes_z[collision_indexes][:, 1]

        # # traj_x_collision = self.path_x[traj_x_collision_indexes]
        # # traj_y_collision = self.path_y[traj_y_collision_indexes]
        # # traj_z_collision = self.path_z[traj_z_collision_indexes]

        return True if len(collision_indexes) > 0 else False


def main():
    rospy.init_node("check_octomap_collision")
    OctomapChecker()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    plt.show()


if __name__ == "__main__":
    main()
