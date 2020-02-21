import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped, Pose
import numpy as np

# TODO: Add capability to capture points from key board using key press

class CapturePoints:
	def __init__(self):
		self.bebopPose = PoseStamped()
		self.geomPose = PoseStamped()
		self.geomPoseTool = dict()
		self.count = 0
		self.prevBebopPoint = None
		self.prevGeomPoint = None
		self.captureBebop = False
		self.captureGeom = False

		rospy.init_node('CapturePoints', anonymous=False)		
		rospy.Subscriber('/bebop/joy',Joy, joyCallback)
		
		if captureBebop:
			self.captureBebop = True
			rospy.Subscriber('/vrpn_client_node/bebop/pose',PoseStamped,pose_callback_bebop)
			self.pose_pub_bebop = rospy.Publisher('bebop/captured_pose', PoseStamped, queue_size=10)
		if captureGeom:
			self.captureGeom = True
			rospy.Subscriber('/vrpn_client_node/GeomTool/pose',PoseStamped,pose_callback_geom)
			self.pose_pub_geom = rospy.Publisher('GeomTool/captured_pose', PoseStamped, queue_size=10)

	def joy_callback(self, msg):
		if msg.buttons[2] == 1:
			pose = self.bebopPose 
			cur_pt = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
			if self.prevBebopPoint is not None:
				dist = np.linalg.norm(cur_pt -  self.prevBebopPoint)
			if self.prevBebopPoint is None or dist > 0.1: 
				self.pose_pub_bebop.publish(pose)
				self.prevBebopPoint = cur_pt
		if msg.buttons[4] == 1:
			pose = self.geomPose
			cur_pt = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
			if self.prevGeomPoint is not None:
				dist = np.linalg.norm(cur_pt -  self.prevGeomPoint)
			if self.prevGeomPoint is None or dist > 0.1: 
				self.pose_pub_geom.publish(pose)
				self.prevGeomPoint = cur_pt
		
	def pose_callback_bebop(self, msg):
		self.bebopPose = msg

	def pose_callback_geom(self, msg):
		self.geomPose = msg

	def pose_callback_geomA(self, msg):
		self.geomPoseTool['A'] = msg
	
	def pose_callback_geomB(self, msg):
		self.geomPoseTool['B'] = msg
	
	def pose_callback_geomC(self, msg):
		self.geomPoseTool['C'] = msg
	
	def pose_callback_geomD(self, msg):
		self.geomPoseTool['D'] = msg
	
	def pose_callback_geomE(self, msg):
		self.geomPoseTool['E'] = msg
	
	def pose_callback_geomF(self, msg):
		self.geomPoseTool['F'] = msg
	
	def pose_callback_geomG(self, msg):
		self.geomPoseTool['G'] = msg
	
	def pose_callback_geomH(self, msg):
		self.geomPoseTool['H'] = msg
	
	def pose_callback_geomI(self, msg):
		self.geomPoseTool['I'] = msg
	
	def pose_callback_geomJ(self, msg):
		self.geomPoseTool['J'] = msg
	
	def pose_callback_geomK(self, msg):
		self.geomPoseTool['K'] = msg
	
	def pose_callback_geomL(self, msg):
		self.geomPoseTool['L'] = msg
	
	def pose_callback_geomM(self, msg):
		self.geomPoseTool['M'] = msg
	
	def pose_callback_geomN(self, msg):
		self.geomPoseTool['N'] = msg

	def pose_callback_geomO(self, msg):
		self.geomPoseTool['O'] = msg
	
	def pose_callback_geomP(self, msg):
		self.geomPoseTool['P'] = msg
	
	def pose_callback_geomQ(self, msg):
		self.geomPoseTool['Q'] = msg
	
	def pose_callback_geomR(self, msg):
		self.geomPoseTool['R'] = msg
	
	def pose_callback_geomS(self, msg):
		self.geomPoseTool['S'] = msg
	
	def pose_callback_geomT(self, msg):
		self.geomPoseTool['T'] = msg
	
	def pose_callback_geomU(self, msg):
		self.geomPoseTool['U'] = msg
	
	def pose_callback_geomV(self, msg):
		self.geomPoseTool['V'] = msg

	def pose_callback_geomW(self, msg):
		self.geomPoseTool['W'] = msg

	def pose_callback_geomX(self, msg):
		self.geomPoseTool['X'] = msg

	def pose_callback_geomY(self, msg):
		self.geomPoseTool['Y'] = msg
	
	def pose_callback_geomZ(self, msg):
		self.geomPoseTool['Z'] = msg



pose = PoseStamped()
count = 0
prev_captured_point = None

def distance(l1, l2):
	return math.sqrt( (l1[0] - l2[0]) **2 + (l1[1] - l2[1]) **2 + (l1[2] - l2[2]) **2 )


def joyCallback(msg):
	global pose,test_writer ,count, prev_captured_point
	pose_pub = rospy.Publisher('bebop/captured_pose', PoseStamped, queue_size=10)
	pose_pub2 = rospy.Publisher('GeomToolA/captured_pose', PoseStamped, queue_size=10)
	dist = float('inf')	
	
	with open('test.csv', mode='a') as test_file:
		test_writer = csv.writer(test_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		if msg.buttons[2] == 1:
			if prev_captured_point is not None:
				cur_point = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
				dist = distance(cur_point, prev_captured_point)
				print(dist)
			if prev_captured_point is None or dist > 0.1: 
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
				prev_captured_point = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
		if msg.buttons[4] == 1:
			test_writer.writerow([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
			pose_pub2.publish(pose)
			rospy.sleep(0.25)

			markerPub = rospy.Publisher('rigidBodyMarker', Marker, queue_size=1)
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
