# Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni
import os
import fnmatch

import carb
from omni.isaac.kit import SimulationApp
import numpy as np
import sys

FRANKA_STAGE_PATH = "/yanu"
CONFIG = {"renderer": "RayTracedLighting", "headless": False}

# Example ROS bridge sample demonstrating the manual loading of stages
# and creation of ROS components
simulation_app = SimulationApp(CONFIG)
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import viewports, stage, extensions, prims, rotations, nucleus
from omni.isaac.core.utils.prims import set_targets
from pxr import Gf

import omni.graph.core as og

# enable ROS bridge extension
extensions.enable_extension("omni.isaac.ros_bridge")
extensions.enable_extension("omni.isaac.repl")
extensions.enable_extension("omni.isaac.articulation_inspector")

simulation_app.update()

from pxr import UsdLux, Sdf, Gf, UsdPhysics, PhysicsSchemaTools, Usd
from pxr import UsdGeom, Gf, PhysxSchema

from omni.kit.menu.utils import add_menu_items, remove_menu_items

from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.graph.core as og

import omni.kit.commands
from omni.isaac.kit import SimulationApp
import omni.ext
import omni.appwindow
import asyncio
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription

from omni.isaac.core import PhysicsContext
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.isaac.core import World
from omni.physx.scripts import utils
from omni.isaac.urdf import _urdf


import carb
import asyncio
import omni.isaac.rospy

from omni.isaac.dynamic_control import _dynamic_control

from sim_interaction import SimInteraction

from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import viewports, stage, extensions, prims, rotations, nucleus
from omni.isaac.core.utils.prims import set_targets
from pxr import Gf

import omni.graph.core as og

# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage
import rosgraph

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

world = World(stage_units_in_meters=1.0)


# Function to check if the prim's path matches the wildcard pattern
def path_matches_wildcard(prim, wildcard_path):
    return fnmatch.fnmatchcase(prim.GetPath().pathString, wildcard_path)


def set_collider(prim):
    if utils.hasSchema(prim, "CollisionAPI"):
        carb.log_warn("CollisionAPI is already defined")
        return

    collisionAPI = UsdPhysics.CollisionAPI.Apply(prim)
    PhysxSchema.PhysxCollisionAPI.Apply(prim)
    collisionAPI.CreateCollisionEnabledAttr().Set(True)

    api_name = PhysxSchema.PhysxTriangleMeshCollisionAPI
    collisionApi2 = api_name.Apply(prim)
    collisionApi3 = UsdPhysics.MeshCollisionAPI.Apply(prim)
    collisionApi3.CreateApproximationAttr().Set("convexDecomposition")


def set_robot_joints(joints: np.array):
    dc = _dynamic_control.acquire_dynamic_control_interface()

    # Get the articulation object representing the Franka robot
    art = dc.get_articulation(FRANKA_STAGE_PATH)
    dc.wake_up_articulation(art)

    # Set joint positions
    joint_positions = [joints]

    dc.set_articulation_dof_position_targets(art, joint_positions)
    _dynamic_control.release_dynamic_control_interface(dc)


def reset_robot_home():
    # reset controller to home
    set_robot_joints(np.array([-0.00521, -1.456, 0.02526, -2.3688, 0.1014, 2.515, 0.712]))


def reset_robot_glass_catch():
    # reset controller to home
    set_robot_joints(np.array([0.0070539364, -0.61308086, -3.5514844e-05, -1.832153, -0.04103387, 2.8022785, 0.83300]))


class YanuRobot:
    def __init__(self):
        self.extension_path = os.path.dirname(os.path.abspath(__file__))
        self.interaction = SimInteraction(self.extension_path)
        self.is_ros_init = False
        self._world = world
        self._world.scene.add_default_ground_plane()

    def setup_scene(self):
        print("setup_scene")

        self.create_franka()
        self.create_ros_action_graph(FRANKA_STAGE_PATH)

        # self.interaction = SimInteraction()

        # self.extension_path = os.path.dirname(os.path.abspath(__file__))
        stage = omni.usd.get_context().get_stage()

        # create glass
        glass = add_reference_to_stage(usd_path=self.extension_path + "/data/klaas.usd", prim_path="/World/glass")

        # set scale and translate
        glass_xform = UsdGeom.Xformable(glass)
        for op in glass_xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                op.Set(Gf.Vec3d(0.001, 0.001, 0.001))
            elif op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op.Set((0.435996, -0.0007245, 1.7309921))

        print("enable glass physics")
        # enable physics
        utils.setRigidBody(glass, "none", False)
        mass_api = UsdPhysics.MassAPI.Apply(glass)
        mass_api.CreateMassAttr(0.234)

        # self._franka = self.scene.get_object("robobar_with_fork")
        # self._fancy_cube = self.scene.get_object("random_cube")

        robobar_base = stage.GetPrimAtPath(FRANKA_STAGE_PATH + "/robobar_link/collisions")
        robobar_gripper = stage.GetPrimAtPath(FRANKA_STAGE_PATH + "/gripper/collisions")

        # robobar_with_fork = stage.GetPrimAtPath("/robobar_with_fork")
        # collision_prims = [prim for prim in Usd.PrimRange(robobar_with_fork) if path_matches_wildcard(prim, wildcard_path)]

        # TODO https://forums.developer.nvidia.com/t/add-missing-collision-approximation-option-for-python-omni-physx-scripts-utils-setcollider/250479

        # utils.setColliderSubtree(glass, approximationShape="convexMeshSimplification")

        utils.removeCollider(robobar_base)
        # the below line doesn't actually set the collider to sdfMesh, but I think there are 2 (or more)
        # different bugs interacting in order to get a result that works.
        utils.setCollider(robobar_base, approximationShape="sdfMesh")
        utils.setCollider(robobar_gripper, approximationShape="convexDecomposition")

        # for prim in collision_prims:
        #     utils.setCollider(prim, approximationShape="sdfMesh")

        return

    def state_callback(self, data):
        print("Recieved state: ", data.current_drink_state)  # asd
        return self.interaction.state_callback(data.current_drink_state)

    def feedback_callback(self, data):
        print("recieved feedback: ", data.feedback.feedback)
        return self.interaction.feedback_callback(data.feedback.feedback)

    def setup_post_load(self):
        print("post_load")

        import rospy
        import rosgraph

        async def my_task():
            try:
                print("init ros subscriber")
                import sys

                # msg_pkg_path = rospkg.RosPack().get_path("robobar_msgs")
                # print("msg path: ", msg_pkg_path + "/msg")
                sys.path.append("/home/kris/dev/yanu-workspace/devel/lib/python3/dist-packages/robobar_msgs/msg")

                from _state import state
                from _robobar_engineActionFeedback import robobar_engineActionFeedback

                # rospy.Subscriber("/robobar_state", state, self.state_callback, queue_size=10)
                rospy.Subscriber(
                    "/robobar_engine/feedback", robobar_engineActionFeedback, self.feedback_callback, queue_size=10
                )

                print("finished task")

            except Exception as e:
                print("had an error")
                print(e)

        # check if rosmaster node is running
        # this is to prevent this sample from waiting indefinetly if roscore is not running
        # can be removed in regular usage

        # if not rosgraph.is_master_online():
        #     print("Please run roscore before executing this script")
        #     # simulation_app.close()
        #     # exit()

        try:
            rospy.init_node("isaac_yanu_subscriber", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
            asyncio.ensure_future(my_task())

        except rospy.exceptions.ROSException:
            print("Node has already been initialized, do nothing")

        # reset_robot_home()

        # await self._world.play_async()
        return

    # This function is called after Reset button is pressed
    # Resetting anything in the world should happen here
    async def setup_post_reset(self):
        self.reset_robot_home()
        await self._world.play_async()
        # hmm hmm sdf azsdsa asd asdas
        return

    def create_ros_action_graph(self, franka_stage_path):
        print("create action graph")
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("PublishJointState", "omni.isaac.ros_bridge.ROS1PublishJointState"),
                        ("SubscribeJointState", "omni.isaac.ros_bridge.ROS1SubscribeJointState"),
                        ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                        ("PublishTF", "omni.isaac.ros_bridge.ROS1PublishTransformTree"),
                        ("PublishClock", "omni.isaac.ros_bridge.ROS1PublishClock"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                        ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                        (
                            "SubscribeJointState.outputs:positionCommand",
                            "ArticulationController.inputs:positionCommand",
                        ),
                        (
                            "SubscribeJointState.outputs:velocityCommand",
                            "ArticulationController.inputs:velocityCommand",
                        ),
                        ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        # Setting the /Franka target prim to Articulation Controller node
                        ("ArticulationController.inputs:usePath", True),
                        ("ArticulationController.inputs:robotPath", franka_stage_path),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # Setting the /Franka target prim to Publish JointState node
        set_target_prims(primPath="/ActionGraph/PublishJointState", targetPrimPaths=[franka_stage_path])

        # Setting the /Franka target prim to Publish Transform Tree node
        set_target_prims(
            primPath="/ActionGraph/PublishTF", inputName="inputs:targetPrims", targetPrimPaths=[franka_stage_path]
        )

        print("finished action graph creation")

        pass

    def create_franka(self):
        # done, pending = await asyncio.wait({task})

        if True:  # task in done:
            status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
            import_config.merge_fixed_joints = False
            import_config.fix_base = True
            import_config.make_default_prim = True
            import_config.create_physics_scene = True
            import_config.import_inertia_tensor = False
            import_config.default_drive_strength = 104532.25
            import_config.default_position_drive_damping = 10453.92188
            # import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY

            print("sadsafe")
            new_urdf = "/home/kris/dev/yanu-workspace/src/robobar/yanu_description/urdf/test.urdf"
            old_urdf = "/home/kris/dev/yanu-workspace/src/robobar/robobar_description/urdf" + "/robobar_with_fork.urdf"
            omni.kit.commands.execute(
                "URDFParseAndImportFile",
                urdf_path=new_urdf,
                import_config=import_config,
            )

            stage = omni.usd.get_context().get_stage()

            # enable physics
            self.scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
            # physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
            # physxSceneAPI.CreateEnableCCDAttr(True)
            # physxSceneAPI.CreateEnableStabilizationAttr(True)
            # physxSceneAPI.CreateEnableGPUDynamicsAttr(True)
            # # physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
            # physxSceneAPI.CreateSolverTypeAttr("TGS")

            # set gravity
            self.scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
            self.scene.CreateGravityMagnitudeAttr().Set(9.81)

            # add ground plane
            PhysicsSchemaTools.addGroundPlane(
                stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, -50), Gf.Vec3f(0.5)
            )

            # add lighting
            distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
            distantLight.CreateIntensityAttr(500)

            print("finished franka creation")


# Preparing stage
viewports.set_camera_view(eye=np.array([2, 2, 2]), target=np.array([1, 1, 1.5]))

simulation_app.update()
# need to initialize physics getting any articulation..etc
world.initialize_physics()

yanu = YanuRobot()
yanu.setup_scene()

yanu.setup_post_load()
simulation_app.update()

reset_robot_home()

world.play()
last_glass_i = 0

world.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)

while simulation_app.is_running():
    # Run with a fixed step size
    world.step(render=True)

    # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
    # og.Controller.set(og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)

world.stop()
simulation_app.close()
