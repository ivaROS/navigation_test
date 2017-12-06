import rosbag
import rospy
import tf2_ros
import numpy as np
import tf.transformations as transformations
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose, PoseStamped, Transform, TransformStamped
from tf2_msgs.msg import TFMessage

bag_dir = '/media/justin/data/bag_files/'
bag_name = 'pips_offboard_2016-09-13-18-36-21.bag'

inbag = bag_dir + bag_name
output = bag_dir + 'pips_paper/early_odom_' + bag_name


#inputbag = rosbag.Bag(inbag)
#outputbag = rosbag.Bag(output, 'w')

#print inputbag.get_message_count()

first_run = True
first_stamp = -1

'''
Steps:
  1. Gather all tf information into tf buffer
  2. Parse rtabmap poses into list of transforms (frame=base_link, child_frame=odom)
  3. For each rtabmap transform, generate transform from corrected_map to map such that corrected_map : base_link = rtab frame : base_link




'''

#Note: these come from 'gazebo_utils/gazebo_world_to_map.py'. Ideally, they should probably be moved into a library

def transformToMat(transform_stamped):
    # numpy arrays to 4x4 transform matrix

    transform = transform_stamped.transform
    t_array = [transform.translation.x, transform.translation.y, transform.translation.z]
    quat_array = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
    trans_mat = transformations.translation_matrix(t_array)
    rot_mat = transformations.quaternion_matrix(quat_array)
    # create a 4x4 matrix
    transf_mat = np.dot(trans_mat, rot_mat)
    return transf_mat


# Based on http://www.cg.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche53.html
def invertMatTransform(mat):
    inv = np.zeros_like(mat)
    R_t = np.transpose(mat[0:3, 0:3])
    inv[0:3, 0:3] = R_t
    inv[3, 3] = 1
    Rt = -R_t.dot(mat[0:3, 3])
    inv[0:3, 3] = Rt
    return inv


def matToTransform(mat):
    trans = TransformStamped()

    # go back to quaternion and 3x1 arrays
    rot_array = transformations.quaternion_from_matrix(mat)
    trans_array = transformations.translation_from_matrix(mat)

    trans.transform.translation.x = trans_array[0]
    trans.transform.translation.y = trans_array[1]
    trans.transform.translation.z = trans_array[2]

    trans.transform.rotation.x = rot_array[0]
    trans.transform.rotation.y = rot_array[1]
    trans.transform.rotation.z = rot_array[2]
    trans.transform.rotation.w = rot_array[3]

    return trans

#Tr_m: Transform from robot to map
#To_r: Transform from odom to robot
def getRelativeTransform(Tr_m, To_r):

        To_r_mat = transformToMat(To_r)
        Tr_m_mat = transformToMat(Tr_m)

        Tm_o_mat = Tr_m_mat.dot(To_r_mat)

        Tm_o = matToTransform(Tm_o_mat)
        Tm_o.header.frame_id = Tr_m.child_frame_id
        Tm_o.header.stamp = To_r.header.stamp #may want to use latest of the 2
        Tm_o.child_frame_id = To_r.child_frame_id

        return Tm_o

def transformFromRtab(pose):

    trans_mat = transformToMat(pose)

    t1 = np.zeros(shape=(3,4))
    t1[0,2] = 1
    t1[2, 0] = 1
    t1[1,1] = -1

    t2 = np.zeros(shape=(3,4))
    t2[0,2] = 1
    t2[2, 1] = -1
    t2[1,0] = -1

    p1 = np.linalg.inv(t1).dot(trans_mat)

    p2 = np.linalg.inv(t2).dot(p1).dot(t2)

    trans = matToTransform(p2)
    trans.header = pose.header
    trans.child_frame_id = pose.child_frame_id

    return trans


def getCorrectiveTransform(tfBuffer, Tr_m, frame_id):

    try:
        To_r = tfBuffer.lookup_transform(Tr_m.header.frame_id, frame_id, Tr_m.header.stamp)

        correction = getRelativeTransform(Tr_m=Tr_m, To_r=To_r)

        return correction

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException), e:
        print e


class OdometryCorrection:

    def __init__(self):
        self.rtab_frame_id = "corrected_map"
        self.global_frame_id = "map"
        self.robot_frame_id = "base_footprint"
        #self.rtab_robot_frame_id = "rtab_base_footprint"

        self.corrections = []


    def openBag(self,path):
        self.source = rosbag.Bag(f=path, mode="r")

        raw_duration = self.source.get_end_time() - self.source.get_start_time()

        duration =rospy.Duration.from_sec(raw_duration)


        self.tfBuffer = tf2_ros.Buffer(cache_time=duration)

    def parseLine(self, line):
        pose = PoseStamped()
        fields = [float(x) for x in line.split()]

        if len(fields) is not 8:
            print "Error! Line does not have 8 fields. Line = \n\t" + str(line)
            return None

        pose.header.frame_id = self.rtab_frame_id
        pose.header.stamp = rospy.Duration.from_sec(fields[0])

        position = fields[1:4]
        orientation = fields[4:8]

        pose.pose.position = Point(*position)
        pose.pose.orientation = Quaternion(*orientation)
        return pose


    def parseRtabPoses(self,path):
        with open(path) as f:
            for line in f:
                pose = self.parseLine(line)
                Tr_m = self.poseToTransform(pose)

                world_transform = transformFromRtab(Tr_m)
                transform = self.getCorrectiveTransform(Tr_m=world_transform)
                self.corrections.append(transform)

    def writeBag(self,destination):
        with rosbag.Bag(f=destination,  mode='w', compression='lz4') as outbag:
            for topic, msg, t in self.source.read_messages():
                outbag.write(topic,msg,t)

            for transform in self.corrections:
                msg = TFMessage()
                msg.transforms.append(transform)
                outbag.write("/tf",msg, transform.header.stamp)



    def poseToTransform(self, pose):
        Tr_m = TransformStamped()
        Tr_m.transform.translation.x = pose.pose.position.x
        Tr_m.transform.translation.y = pose.pose.position.y
        Tr_m.transform.translation.z = pose.pose.position.z
        Tr_m.transform.rotation = pose.pose.orientation
        Tr_m.header.stamp = pose.header.stamp
        Tr_m.header.frame_id = self.rtab_robot_frame_id
        Tr_m.child_frame_id = pose.header.frame_id
        return Tr_m

    def getCorrectiveTransform(self, Tr_m):
        correction = getCorrectiveTransform(self.tfBuffer, Tr_m=Tr_m, frame_id=self.global_frame_id)
        print str(correction)
        return correction


    def loadBagTfs(self):
        for topic, msg, t in self.source.read_messages(topics="/tf"):
            for transform in msg.transforms:
                self.tfBuffer.set_transform(transform, "default_authority")

    def addStaticTf(self):
        transform = TransformStamped()
        transform.header.frame_id = self.robot_frame_id
        transform.child_frame_id = self.rtab_robot_frame_id

        transform.transform.rotation.y = -.707
        transform.transform.rotation.z = .707

        self.tfBuffer.set_transform_static(transform, "default_authority")

        transform = TransformStamped()
        transform.header.frame_id = "odom"

        transform.transform.rotation.z = .707
        transform.transform.rotation.w = .707


if __name__ == "__main__":
    correcter = OdometryCorrection()
    correcter.openBag(path="/media/justin/data/real_testing/test_1/test_4.bag")
    correcter.loadBagTfs()
    #correcter.addStaticTf()
    correcter.parseRtabPoses(path="/media/justin/data/real_testing/rtabmap_records/poses_1_4.txt")
    correcter.writeBag(destination="/media/justin/data/real_testing/rtabmap_records/processed_1_4.bag")
