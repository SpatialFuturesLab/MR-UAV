import sys, select, tty, termios
import numpy as np
import string

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped, Pose, PoseArray

class NonBlockingConsole(object):
    def __enter__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_data(self):
        try:
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                return sys.stdin.read(1)
        except:
            return '[CTRL-C]'
        return False

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

class CapturePoints:
	def __init__(self, list_of_tools = list(string.ascii_uppercase), list_of_points = list(string.digits)):
		self.capturePoints = True
		self.geomPoseTool = dict()
		self.prevGeomToolPoint = dict()
		self.publishGeomToolPose = dict()
		rospy.init_node('CaptureGeomToolPoints', anonymous=False)		
		self.toolsList = list_of_tools
		
		self.snaphotPose = dict()
		self.publishSnapshotPose = dict()
		self.publishSnapshotPoseArray = rospy.Publisher('Snapshot/captured_pose', PoseArray, queue_size=10)
		
		self.pointsList = list_of_points
		self.poseList = PoseArray()
		
		for p in self.pointsList:
			callback = getattr(self, 'pose_callback_point' + p)
			rospy.Subscriber('/vrpn_client_node/SnapPoint{0}/pose'.format(p),PoseStamped,callback)
			self.publishSnapshotPose[p] = rospy.Publisher('SnapPoint{0}/captured_pose'.format(p), PoseStamped, queue_size=10)
			self.snaphotPose[p] = None

		for tool in self.toolsList:
			callback = getattr(self, 'pose_callback_geom' + tool)
			rospy.Subscriber('/vrpn_client_node/GeomTool{0}/pose'.format(tool),PoseStamped,callback)
			self.publishGeomToolPose[tool] = rospy.Publisher('GeomTool{0}/captured_pose'.format(tool), PoseStamped, queue_size=10)
			self.prevGeomToolPoint[tool] = None
			self.geomPoseTool[tool] = None

		self.publishVivePose = rospy.Publisher('vive/captured_pose', PoseStamped, queue_size=10)

	
	def key_reader(self):
		with NonBlockingConsole() as nbc:
			while self.capturePoints:
				c = nbc.get_data()
				if c == '\x1b': # Esc
					self.capturePoints = False
				if c == '\x20': # Space
					try:
						for p in self.pointsList:
							if self.snaphotPose[p] is not None:
								self.poseList.poses.append(self.snaphotPose[p].pose)
						self.publishSnapshotPoseArray.publish(self.poseList)
						print "Publishing list of poses captured"
						self.poseList = PoseArray()
					except:
						print "Cannot send list of poses!!"


				elif c is not False:
					# print c
					lower_case_tools_list = [ch.lower() for ch in self.toolsList]
					if c in lower_case_tools_list:
						tool = c.upper()
						try:
							if tool in self.toolsList:
								pose = self.geomPoseTool[tool]
								cur_pt = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
								if self.prevGeomToolPoint[tool] is not None:
									dist = np.linalg.norm(cur_pt -  self.prevGeomToolPoint[tool])
								if self.prevGeomToolPoint[tool] is None or dist > 0.1:
									self.publishGeomToolPose[tool].publish(pose)
									self.prevGeomToolPoint[tool] = cur_pt
									print "Capturing Point from GeomTool{0}".format(tool)
								else:
									print "\"Exception: Point too close\" Cannot Capture Point from GeomTool{0}".format(tool)
							else:
								print "Invalid Key Press: {0}".tool
						except Exception as e:
							# print(c)
							print "\"Exception: GeomTool{0} not defined\" Cannot Capture Point from GeomTool{0}".format(tool)
					if c in self.pointsList:
						try:
							if str(c) in self.pointsList:
								pose = self.snaphotPose[str(c)]
								self.publishSnapshotPose[str(c)].publish(pose)
								print "Capturing Point from Marker {0}".format(str(c))
							else:
								print "Invalid Key Press: {0}".format(c)
						except Exception as e:
							print "\"Exception: Marker {0} not defined\" Cannot Capture Point from Marker {0}".format(c)
					

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

	def pose_callback_point0(self, msg):
		self.snaphotPose['0'] = msg
	
	def pose_callback_point1(self, msg):
		self.snaphotPose['1'] = msg
	
	def pose_callback_point2(self, msg):
		self.snaphotPose['2'] = msg
	
	def pose_callback_point3(self, msg):
		self.snaphotPose['3'] = msg
	
	def pose_callback_point4(self, msg):
		self.snaphotPose['4'] = msg
	
	def pose_callback_point5(self, msg):
		self.snaphotPose['5'] = msg
	
	def pose_callback_point6(self, msg):
		self.snaphotPose['6'] = msg
	
	def pose_callback_point7(self, msg):
		self.snaphotPose['7'] = msg
	
	def pose_callback_point8(self, msg):
		self.snaphotPose['8'] = msg
	
	def pose_callback_point9(self, msg):
		self.snaphotPose['9'] = msg

	def pose_callback_vive(self, msg):
		self.publishVivePose(msg)

	

if __name__ == '__main__':
	capture_geom_tools = CapturePoints()
	capture_geom_tools.key_reader()
