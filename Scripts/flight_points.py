import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped, Pose
import csv
from visualization_msgs.msg import Marker

pose = PoseStamped()
count = 0

def joyCallback(msg):
	global pose,test_writer ,count
	pose_pub = rospy.Publisher('bebop/captured_pose', PoseStamped, queue_size=10)	
	with open('test.csv', mode='a') as test_file:
		test_writer = csv.writer(test_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		if msg.buttons[2] == 1:
			test_writer.writerow([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
			pose_pub.publish(pose)
			rospy.sleep(0.25)

			markerPub = rospy.Publisher('bebopMarker', Marker, queue_size=1)
			marker = Marker()
			marker.header.frame_id = "/world"
			marker.ns = "basic_shapes";
			marker.id = count;
			count += 1 
			marker.type = Marker.SPHERE; # shapes -> http://wiki.ros.org/rviz/DisplayTypes/Marker
			marker.action = Marker.ADD

			marker.color.r = 0.0
			marker.color.g = 1.0
			marker.color.b = 0.0
			marker.color.a = 1.0

			marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;    
			marker.pose = pose.pose

			markerPub.publish(marker);

def poseCallback(msg):
	global pose
	pose = msg

def main():
	rospy.init_node('spacepoints', anonymous=False)		
	rospy.Subscriber('/vrpn_client_node/bebop/pose',PoseStamped,poseCallback)
	rospy.Subscriber('/bebop/joy',Joy, joyCallback)
	
	global test_writer
	rospy.spin()
	

if __name__ == '__main__':
	main()
