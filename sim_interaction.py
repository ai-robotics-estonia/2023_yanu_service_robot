# this is a python module that accepts yanu robot state enum values and
# for some state transitions, reacts inside the sim

from enum import Enum
from dataclasses import dataclass
from typing import Union
import asyncio
from omni.isaac.core.utils.stage import add_reference_to_stage

from omni.physx.scripts import utils
from pxr import Gf, UsdGeom, UsdPhysics


@dataclass
class COCKTAIL_WAITING_ORDER:
    pass


@dataclass
class COCKTAIL_PICK_GLASS:
    pass


@dataclass
class COCKTAIL_PRE_ICE_1:
    pass


@dataclass
class COCKTAIL_PRE_ICE_2:
    pass


@dataclass
class COCKTAIL_ICE:
    pass


@dataclass
class COCKTAIL_POST_ICE:
    pass


@dataclass
class COCKTAIL_SODA_1:
    pass


@dataclass
class COCKTAIL_SODA_2:
    pass


@dataclass
class COCKTAIL_SODA_3:
    pass


@dataclass
class COCKTAIL_SODA_4:
    pass


@dataclass
class COCKTAIL_AREA_OPTION:
    pass


@dataclass
class COCKTAIL_LOOP:
    pass


@dataclass
class COCKTAIL_FILLING:
    pass


@dataclass
class COCKTAIL_POST_FILLING:
    pass


@dataclass
class COCKTAIL_PRE_SERVING:
    pass


@dataclass
class COCKTAIL_SERVING:
    pass


@dataclass
class COCKTAIL_POST_SERVING:
    pass


@dataclass
class COCKTAIL_GLASS_RELEASE:
    pass


@dataclass
class COCKTAIL_HOME:
    pass


@dataclass
class COCKTAIL_FINAL:
    pass


STATES = Union[
    COCKTAIL_WAITING_ORDER,
    COCKTAIL_PICK_GLASS,
    COCKTAIL_PRE_ICE_1,
    COCKTAIL_PRE_ICE_2,
    COCKTAIL_ICE,
    COCKTAIL_POST_ICE,
    COCKTAIL_SODA_1,
    COCKTAIL_SODA_2,
    COCKTAIL_SODA_3,
    COCKTAIL_SODA_4,
    COCKTAIL_AREA_OPTION,
    COCKTAIL_LOOP,
    COCKTAIL_FILLING,
    COCKTAIL_POST_FILLING,
    COCKTAIL_PRE_SERVING,
    COCKTAIL_SERVING,
    COCKTAIL_POST_SERVING,
    COCKTAIL_GLASS_RELEASE,
    COCKTAIL_HOME,
    COCKTAIL_FINAL,
]

states_as_string = set(s.__name__ for s in STATES.__args__)

ALL_FEEDBACKS = {"cup_received",
                 "cup_released",
                 "add_ice_started",
                 "add_ice_finished",
                 "add_mixer_started",
                 "add_mixer_finished",
                 "add_alcohol_started",
                 "add_alcohol_finished",
                 "drink_inserted_to_dock",
                 "drink_finished",
                 "drink_removed_from_dock",
                 "drink_failed",
                 "NEW_ORDER_CAN_BE_SENT",
                 }


class StateNotFoundError(Exception):
    """
    Custom exception class to be raised when the provided state name
    does not correspond to any of the defined state classes.
    """

    pass


def get_state_class(state_name: str) -> STATES:
    """
    Returns the class object corresponding to the provided state name.
    Raises a StateNotFoundError if the state name is not found.
    """
    # Get a dictionary of all the defined classes in the current scope
    classes = globals()

    if state_name not in states_as_string:
        raise StateNotFoundError(f"No state class found with name '{state_name}'")

    # Get the class object corresponding to the provided state name
    state_class = classes.get(state_name, None)

    return state_class


extension_path = "/home/kris/.local/share/ov/pkg/isaac_sim-2022.2.1/standalone_examples/yanu/"
glass_i = 20


class SimInteraction:
    def __init__(self, extension_path) -> None:
        self.current_state: STATES = COCKTAIL_WAITING_ORDER
        self.loop = asyncio.get_event_loop()
        self.glass_i = glass_i
        self.last_glass = None
        self.extension_path = extension_path
        pass

    def spawn_glass_instance(self):
        # Create a glass instance
        glass_instance = add_reference_to_stage(usd_path=self.extension_path + "/data/klaas.usd",
                                                prim_path=f"/World/glass_{self.glass_i}")

        # Set scale and translation
        glass_xform = UsdGeom.Xformable(glass_instance)
        for op in glass_xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                op.Set(Gf.Vec3d(0.001, 0.001, 0.001))
            elif op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                # op.Set((0.435996, -0.0007245, 1.7557921))
                op.Set((0.69177, 0.001117, 1.807921))

        # enable mass modification
        mass_api = UsdPhysics.MassAPI.Apply(glass_instance)
        mass_api.CreateMassAttr(3)

        utils.setRigidBody(glass_instance, "convexHull", False)

        # save glass to enable later mass modifications
        self.last_glass_mass = mass_api
        self.glass_i += 1

    async def callback_impl(self, state_str):
        new_state: STATES = get_state_class(state_str)

        if new_state == self.current_state:
            return

        print("new state ", state_str)
        self.current_state = new_state

        if new_state == COCKTAIL_PICK_GLASS:
            # sleep 1 sec

            print("sleeping in cb1")
            await asyncio.sleep(2)

            # spawn the cup

            print("spawning cup")
            self.spawn_glass_instance()

            pass
        elif new_state == COCKTAIL_ICE:  # TODO same for all the sodas and fillings
            # sleep 1 sec
            print("sleeping in cb")
            await asyncio.sleep(1)
            # increase the weight of the cup

            mass = self.last_glass_mass.GetMassAttr()
            self.last_glass_mass.SetMassAttr(mass + 0.1)
            pass

    def state_callback(self, state_str):
        """
        this currently shouldn't be called
        """
        raise NotImplementedError("state_callback is deprecated")
        # return self.loop.create_task(self.callback_impl(state_str))
    
    async def feedback_handler(self, feedback: str):
        """
             simulates the real world by creating a cup or increasing cup mass

            @param feedback: string in the set {"cup_received",
                     "cup_released",
                     "add_ice_started",
                     "add_ice_finished",
                     "add_mixer_started",
                     "add_mixer_finished",
                     "add_alcohol_started",
                     "add_alcohol_finished",
                     "drink_inserted_to_dock",
                     "drink_finished",
                     "drink_removed_from_dock",
                     "drink_failed",
                     "NEW_ORDER_CAN_BE_SENT",
                     }
        """
        if feedback == "cup_released":
            await asyncio.sleep(0.1)
            self.spawn_glass_instance()

        elif feedback == "add_ice_started":
            await asyncio.sleep(1)
            mass = self.last_glass_mass.GetMassAttr()
            self.last_glass_mass.SetMassAttr(mass + 0.1)
            
        elif feedback == "add_mixer_started":
            await asyncio.sleep(1)
            mass = self.last_glass_mass.GetMassAttr()
            self.last_glass_mass.SetMassAttr(mass + 0.3)
            
        elif feedback == "add_alcohol_started":
            await asyncio.sleep(1)
            mass = self.last_glass_mass.GetMassAttr()
            self.last_glass_mass.SetMassAttr(mass + 0.1)
            
        elif feedback == "add_alcohol_started":
            await asyncio.sleep(1)
            mass = self.last_glass_mass.GetMassAttr()
            self.last_glass_mass.SetMassAttr(mass + 0.1)

    
    def feedback_callback(self, feedback):
        if feedback not in ALL_FEEDBACKS:
            raise RuntimeError("got feedback value that differs from possible feedback values")

        return self.loop.create_task(self.feedback_handler(feedback))
        



si = SimInteraction(extension_path=extension_path)

# state_class = get_state_class("COCKTAIL_PICK_GLASS")
# print(state_class)
# transitions = []
