import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
from tesseract.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo
from tesseract.tesseract_environment import Environment
from tesseract.tesseract_scene_graph import SimpleResourceLocator, SimpleResourceLocatorFn
"""from tesseract.tesseract_command_language import CartesianWaypoint, Waypoint, \
    PlanInstructionType_FREESPACE, PlanInstructionType_START, PlanInstruction, Instruction, \
    CompositeInstruction, flatten
from tesseract.tesseract_process_managers import ProcessPlanningServer, ProcessPlanningRequest, \
    FREESPACE_PLANNER_NAME"""
import os
import re
import traceback
from tesseract_viewer import TesseractViewer
import numpy as np
import time
import sys
import argparse
from typing import List
import threading


_robdef = """
          service experimental.tesseract_viewer

          import com.robotraconteur.robotics.trajectory

          using com.robotraconteur.robotics.trajectory.JointTrajectory

          object TesseractViewer
          
              property string{list} joint_names [readonly]

              function void update_joint_positions(string{list} joint_names, double[] joint_positions)
              function void play_trajectory(JointTrajectory trajectory)

          end

          """


def main():

    parser = argparse.ArgumentParser(description="Tesseract Viewer Standalone Robot Raconteur Service")
    parser.add_argument("--urdf-file", type=argparse.FileType('r'),default=None,required=True,help="URDF file for scene (required)")
    parser.add_argument("--srdf-file", type=argparse.FileType('r'),default=None,required=True,help="SRDF file for scene (required)")
    parser.add_argument("--manipulator-name", type=str, default="manipulator",help="Name of manipulator in SRDF file (default \"manipulator\"")
    parser.add_argument("--z-offset",type=float,default=0.0,help="Z-offset for scene (default 0.0)")
    parser.add_argument("--http-port",type=int,default=8000,help="HTTP TCP/IP Listen Port (default 8000)")

    args, _ = parser.parse_known_args()

    with args.urdf_file:
        urdf_file_text = args.urdf_file.read()

    with args.srdf_file:
        srdf_file_text = args.srdf_file.read()

    t_env = Environment()

    # locator_fn must be kept alive by maintaining a reference
    locator = GazeboModelResourceLocator()
    t_env.init(urdf_file_text, srdf_file_text, locator)

    manip_info = ManipulatorInfo()
    manip_info.manipulator = args.manipulator_name

    viewer = TesseractViewer(server_address=('',args.http_port))

    viewer.update_environment(t_env, [0,0,args.z_offset])

    # TODO: thread lock for updates?
    viewer.start_serve_background()

    joint_names = list(t_env.getActiveJointNames())

    obj=TesseractViewerService(viewer,joint_names)
    RRC.RegisterStdRobDefServiceTypes(RRN)
    RRN.RegisterServiceType(_robdef)

    with RR.ServerNodeSetup("experimental.tesseract_viewer",59712,argv=sys.argv):

        RRN.RegisterService("tesseract_viewer","experimental.tesseract_viewer.TesseractViewer",obj)

        if sys.version_info[0] < 3:
            input("press enter")
        else:
            input("press enter")

class TesseractViewerService:
    def __init__(self, viewer : TesseractViewer, joint_names: List[str]):
        self._viewer = viewer
        self.joint_names = joint_names
        self._lock = threading.Lock()

    def update_joint_positions(self, joint_names : List[str], joint_positions : np.array):
        with self._lock:
            
            assert len(joint_positions) == len(joint_names), "joint_names and joint_positions must have same length"
            for j in joint_names:
                assert j in self.joint_names, f"Invalid joint name: {j}"

            self._viewer.update_joint_positions(joint_names, joint_positions)

    

    

class GazeboModelResourceLocatorFn:
	def __init__(self):
		model_env_path = os.environ["GAZEBO_MODEL_PATH"]
		self.model_paths = model_env_path.split(os.pathsep)
		assert len(self.model_paths) != 0, "No GAZEBO_MODEL_PATH specified!"
		for p in self.model_paths:
			assert os.path.isdir(p), "GAZEBO_MODEL_PATH directory does not exist: %s" % p

	def __call__(self,url):
		try:
			url_match = re.match(r"^model:\/\/(\w+)\/(.+)$",url)
			if (url_match is None):
				assert False, "Invalid Gazebo model resource url %s" % url
			model_name = url_match.group(1)
			resource_path = os.path.normpath(url_match.group(2))

			for p in self.model_paths:

				fname = os.path.join(p, model_name, resource_path )
				if not os.path.isfile(fname):
					continue
				return fname

			assert False, "Could not find requested resource %s" % url
		except:
			traceback.print_exc()
			return ""

def GazeboModelResourceLocator():
	locator_fn = SimpleResourceLocatorFn(GazeboModelResourceLocatorFn())
	locator = SimpleResourceLocator(locator_fn)
	locator_fn.__disown__()
	return locator

if __name__ == "__main__":
    main()