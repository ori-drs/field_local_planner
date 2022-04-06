from geometry_msgs.msg import Pose, Quaternion, Point
import data_tools_base.TrajectoryHelper as th

from . import robot_geometry
import rospy
import rospkg

ref_z = 1.0
test_poses = []

# # Simple batlab experiment
# ref_z = 0.7
# test_poses.append(Pose(Point(12.5, -4.3, ref_z), Quaternion(0.000, 0.000, 0.000, 1.000)))
# test_poses.append(Pose(Point(12.5, -7.5 , ref_z), Quaternion(0.000, 0.000, 0.000, 1.000)))


# Simulation environments
ref_z = 1.0
rospack = rospkg.RosPack()
traj_path = rospack.get_path("locally_reactive_controller_evaluation") + "/config/test_trajectories/test_trajectory.csv"

# Read trajectory
traj = th.readTrajectory(traj_path, format="tum")

for t in traj.index:
    test_poses.append(Pose(Point(traj["x"][t],
                                 traj["y"][t],
                                 traj["z"][t]), 
                           Quaternion(traj["qx"][t], 
                                      traj["qy"][t],
                                      traj["qz"][t],
                                      traj["qw"][t])))



# test_poses.append(Pose(Point(-1.00,  3.75, ref_z), Quaternion(0.000, 0.000, -0.694, 0.719)))
# test_poses.append(Pose(Point(-1.93,  1.00, ref_z), Quaternion(0.000, 0.000, -0.714, 0.700)))
# test_poses.append(Pose(Point(-1.05, -1.77, ref_z), Quaternion(0.000, 0.000, -0.714, 0.700)))
# test_poses.append(Pose(Point(-1.29, -2.85, ref_z), Quaternion(0.000, 0.000, -0.839, 0.543)))
# test_poses.append(Pose(Point(-1.30, -3.74, ref_z), Quaternion(0.000, 0.000, -0.229, 0.973)))
# test_poses.append(Pose(Point( 0.30, -3.93, ref_z), Quaternion(0.000, 0.000,  0.692, 0.721)))
# test_poses.append(Pose(Point( 0.56, -2.11, ref_z), Quaternion(0.000, 0.000,  0.328, 0.944)))
# test_poses.append(Pose(Point( 1.99, -0.87, ref_z), Quaternion(0.000, 0.000,  0.697, 0.716)))
# test_poses.append(Pose(Point( 1.40,  1.87, ref_z), Quaternion(0.000, 0.000,  0.690, 0.723)))
# # test_poses.append(Pose(Point( 1.80,  3.85, ref_z), Quaternion(0.000, 0.000, -0.694, 0.719)))
# test_poses.append(Pose(Point( 1.80,  3.85, ref_z), Quaternion(0.000, 0.000, 0.694, 0.719)))

# position: 
#   x: -1.9351747035980225
#   y: 1.0761288404464722
#   z: 0.0
# orientation: 
#   x: 0.0
#   y: 0.0
#   z: -0.7141039147849152
#   w: 0.7000397123655618

# position: 
#   x: -1.0571081638336182
#   y: -1.779447078704834
#   z: 0.0
# orientation: 
#   x: 0.0
#   y: 0.0
#   z: -0.7132259060296519
#   w: 0.7009342386901799

# position: 
#   x: -1.2951122522354126
#   y: -2.84451961517334
#   z: 0.0
# orientation: 
#   x: 0.0
#   y: 0.0
#   z: -0.8393042765654025
#   w: 0.5436619642195197

# position: 
#   x: -1.3043508529663086
#   y: -3.740222454071045
#   z: 0.0
# orientation: 
#   x: 0.0
#   y: 0.0
#   z: -0.22936517447758759
#   w: 0.9733404423616979

# position: 
#   x: -0.19950109720230103
#   y: -3.9383859634399414
#   z: 0.0
# orientation: 
#   x: 0.0
#   y: 0.0
#   z: -0.025438127421399698
#   w: 0.9996763984776736

# position: 
#   x: 0.5653654336929321
#   y: -2.1165287494659424
#   z: 0.0
# orientation: 
#   x: 0.0
#   y: 0.0
#   z: 0.3286934368328982
#   w: 0.9444366704988628

# position: 
#   x: 1.9974874258041382
#   y: -0.8713651895523071
#   z: 0.0
# orientation: 
#   x: 0.0
#   y: 0.0
#   z: 0.6970735452914896
#   w: 0.7169996321161912

# position: 
#   x: 1.4081207513809204
#   y: 1.8749662637710571
#   z: 0.0
# orientation: 
#   x: 0.0
#   y: 0.0
#   z: 0.6902556050324428
#   w: 0.7235656153530904

# position: 
#   x: 1.685299277305603
#   y: 3.7362442016601562
#   z: 0.0
# orientation: 
#   x: 0.0
#   y: 0.0
#   z: -0.6947803866206553
#   w: 0.7192219506989846
