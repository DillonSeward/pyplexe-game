import sys
import os
import random
import math
from typing import Dict, Tuple, List, Optional

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib
import traci
from plexe import POS_X, POS_Y, ENGINE_MODEL_REALISTIC, ACC, CACC, FAKED_CACC

# vehicle length
LENGTH = 4
# inter-vehicle distance
DISTANCE = 5
# inter-vehicle distance when leaving space for joining
JOIN_DISTANCE = DISTANCE * 2
# cruising speed
SPEED = 120 / 3.6


class Vehicle:
    # MAY NEED TO STORE LEADER INFORMATION
    def __init__(
        self, id: str, front: Optional[str] = None, back: Optional[str] = None
    ):
        self.id = id
        self.front = front
        self.back = back

    def __str__(self):
        return f"Vehicle(id='{self.id}', front='{self.front or 'None'}', back='{self.back or 'None'}')"


class Platoon:
    def __init__(
        self,
        leader_id: str,
        # lane: int,
        vehicles: List[Vehicle],
    ):
        self.leader_id = leader_id
        self.vehicles: List[Vehicle] = vehicles
        # self.lane: int = lane


class Topology:
    def __init__(self):
        """
        Each `platoon` has:
        - a leader ID (str)
        - a list of Vehicle objects
        """
        self.platoons: Dict[int, Platoon] = {}
        """
         Temporary platoons have to be made to execute most maneuvers
         when these are created, we store the id of the temporary platoon as a key with the id of the platoon it branched off of as a value
        """
        self.temporaries: Dict[int, int] = {}

    def __str__(self) -> str:
        output = ["Topology:"]
        for platoon_id, platoon in self.platoons.items():
            output.append(f"  Platoon {platoon_id}:")
            output.append(f"    Leader: {platoon.leader_id}")
            output.append(f"    Vehicles:")
            for v in platoon.vehicles:
                output.append(f"      - ID: {v.id}, Front: {v.front}, Back: {v.back}")
        if self.temporaries:
            output.append("  Temporary Platoon Mappings:")
            for sub_id, og_id in self.temporaries.items():
                output.append(f"    Temporary {sub_id} -> Original {og_id}")
        return "\n".join(output)

    def create_sub_platoon(self, leader_id: str) -> int:
        # Return existing sub-platoon if already made for this leader
        for sub_id, _ in self.temporaries.items():
            if self.platoons[sub_id].leader_id == leader_id:
                return sub_id

        platoon_id, _ = self.get_vehicle(leader_id)
        vehicles = self.platoons[platoon_id].vehicles

        leader_idx = next(
            (i for i, v in enumerate(vehicles) if v.id == leader_id), None
        )
        assert leader_idx is not None, f"{leader_id} not found in platoon {platoon_id}"

        sub_platoon = self.create_platoon(leader_id)
        self.temporaries[sub_platoon] = platoon_id

        to_move = vehicles[leader_idx:]
        self.platoons[platoon_id] = Platoon(
            self.platoons[platoon_id].leader_id,
            vehicles[:leader_idx],
        )

        for v in to_move:
            self.add_vehicle(sub_platoon, v)

        return sub_platoon

    def reset_leaders(self):
        while self.temporaries:
            sub_id, og_id = self.temporaries.popitem()
            vehicles = self.platoons[sub_id].vehicles
            # Use a copy of the list to avoid issues while modifying it
            for v in vehicles:
                self.remove_vehicle(v.id)
                self.add_vehicle(og_id, v)

    def create_platoon(self, leader_id: str) -> int:
        id = len(self.platoons)
        self.platoons[id] = Platoon(leader_id, [])
        return id

    def add_vehicle(self, platoon: int, vehicle: Vehicle):
        if platoon not in self.platoons:
            self.platoons[platoon] = Platoon("", [])
        platoon = self.platoons[platoon]
        platoon.vehicles.append(vehicle)

    def get_leader(self, vehicle_id: str):
        for platoon_id, platoon in self.platoons.items():
            for i, v in enumerate(platoon.vehicles):
                if v.id == vehicle_id:
                    del platoon.vehicles[i]
                    return platoon.leader_id

    def remove_vehicle(self, vehicle_id: str) -> Vehicle:
        for platoon_id, platoon in self.platoons.items():
            for i, v in enumerate(platoon.vehicles):
                if v.id == vehicle_id:
                    return platoon.vehicles.pop(i)

    def get_vehicle(self, vehicle_id: str) -> Tuple[int, Vehicle]:
        for platoon_id, platoon in self.platoons.items():
            for v in platoon.vehicles:
                if v.id == vehicle_id:
                    return platoon_id, v
        raise ValueError(f"Vehicle {vehicle_id} not found")

    def inPlatoon(self, vid: str) -> bool:
        try:
            platoon_id, _ = self.get_vehicle(vid)
            platoon = self.platoons[platoon_id]
            return len(platoon.vehicles) == 1
        except ValueError:
            return True


# Maybe something like this needs to be moved to Topology class??
def init_topology(n_vehicles_per_platoon: List[int]) -> Topology:
    topology = Topology()
    vehicles_count = 0

    for platoon_id, n in enumerate(n_vehicles_per_platoon):
        for i in range(n):
            vid = "v.%d" % vehicles_count
            vehicle = Vehicle(vid)
            if i == 0:
                topology.create_platoon(vid)
            if i > 0:
                vehicle.front = "v.%d" % (vehicles_count - 1)
            if i < n - 1:
                vehicle.back = "v.%d" % (vehicles_count + 1)

            topology.add_vehicle(platoon_id, vehicle)
            vehicles_count += 1
    return topology


def init_simulation(plexe, topology: Topology, real_engine=False):
    for platoon_id, platoon in topology.platoons.items():
        for i, v in enumerate(platoon.vehicles):
            if platoon_id == 0:
                lane = 0
            if platoon_id > 0:
                lane = platoon_id + 0
            add_platooning_vehicle(
                plexe,
                v.id,
                (len(platoon.vehicles) - i + 1 + platoon_id) * (DISTANCE + LENGTH) + 50,
                lane,
                SPEED,
                DISTANCE,
                real_engine,
            )
            plexe.set_fixed_lane(v.id, lane, safe=True)
            traci.vehicle.setLaneChangeMode(v.id, 0)
            traci.vehicle.setSpeedMode(v.id, 0)

            if v.id == platoon.leader_id:
                plexe.set_active_controller(v.id, ACC)
            else:
                plexe.set_active_controller(v.id, CACC)


def get_in_position(plexe, jid, fid, topology):
    """
    Makes the joining vehicle get close to the join position. This is done by
    changing the topology and setting the leader and the front vehicle for
    the joiner. In addition, we increase the cruising speed and we switch to
    the "fake" CACC, which uses a given GPS distance instead of the radar
    distance to compute the control action
    :param plexe: API instance
    :param jid: id of the joiner
    :param fid: id of the vehicle that will become the predecessor of the joiner
    :param topology: the current platoon topology
    :return: the modified topology
    """
    _, joiner = topology.get_vehicle(jid)
    joiner.front = fid
    plexe.set_cc_desired_speed(joiner.id, SPEED + 15)
    plexe.set_active_controller(joiner.id, FAKED_CACC)
    return topology


def open_gap(plexe, vid, jid, join_distance: int, topology: Topology) -> Topology:
    """
    Makes the vehicle that will be behind the joiner open a gap to let the
    joiner in. This is done by creating a temporary platoon, i.e., setting
    the leader of all vehicles behind to the one that opens the gap and then
    setting the front vehicle of the latter to be the joiner. To properly
    open the gap, the vehicle leaving space switches to the "fake" CACC,
    to consider the GPS distance to the joiner
    :param plexe: API instance
    :param vid: vehicle that should open the gap
    :param jid: id of the joiner
    :param topology: the current platoon topology
    :param n: total number of vehicles currently in the platoon
    :return: the modified topology
    """
    topology.create_sub_platoon(vid)
    # the front vehicle if the vehicle opening the gap is the joiner
    _, v = topology.get_vehicle(vid)
    v.front = jid
    # v = topology.get_vehicle(vid)[1]
    # v.front = jid
    # topology[0][vid]["front"] = jid
    plexe.set_active_controller(vid, FAKED_CACC)
    plexe.set_path_cacc_parameters(vid, distance=join_distance)
    fd = plexe.get_vehicle_data(jid)
    plexe.set_front_vehicle_data(vid, fd)
    distance = get_distance(plexe, vid, jid)
    plexe.set_front_vehicle_fake_data(vid, fd, distance)

    return topology


# lane change state bits
bits = {
    0: "LCA_NONE",
    1 << 0: "LCA_STAY",
    1 << 1: "LCA_LEFT",
    1 << 2: "LCA_RIGHT",
    1 << 3: "LCA_STRATEGIC",
    1 << 4: "LCA_COOPERATIVE",
    1 << 5: "LCA_SPEEDGAIN",
    1 << 6: "LCA_KEEPRIGHT",
    1 << 7: "LCA_TRACI",
    1 << 8: "LCA_URGENT",
    1 << 9: "LCA_BLOCKED_BY_LEFT_LEADER",
    1 << 10: "LCA_BLOCKED_BY_LEFT_FOLLOWER",
    1 << 11: "LCA_BLOCKED_BY_RIGHT_LEADER",
    1 << 12: "LCA_BLOCKED_BY_RIGHT_FOLLOWER",
    1 << 13: "LCA_OVERLAPPING",
    1 << 14: "LCA_INSUFFICIENT_SPACE",
    1 << 15: "LCA_SUBLANE",
    1 << 16: "LCA_AMBLOCKINGLEADER",
    1 << 17: "LCA_AMBLOCKINGFOLLOWER",
    1 << 18: "LCA_MRIGHT",
    1 << 19: "LCA_MLEFT",
    1 << 30: "LCA_UNKNOWN",
}


def add_vehicle(plexe, vid, position, lane, speed, vtype="vtypeauto"):
    if plexe.version[0] >= 1:
        traci.vehicle.add(
            vid,
            "platoon_route",
            departPos=str(position),
            departSpeed=str(speed),
            departLane=str(lane),
            typeID=vtype,
        )
    else:
        traci.vehicle.add(
            vid, "platoon_route", pos=position, speed=speed, lane=lane, typeID=vtype
        )


def add_platooning_vehicle(
    plexe,
    vid,
    position,
    lane,
    speed,
    cacc_spacing,
    real_engine=False,
    vtype="vtypeauto",
):
    """
    Adds a vehicle to the simulation
    :param plexe: API instance
    :param vid: vehicle id to be set
    :param position: position of the vehicle
    :param lane: lane
    :param speed: starting speed
    :param cacc_spacing: spacing to be set for the CACC
    :param real_engine: use the realistic engine model or the first order lag
    model
    """
    add_vehicle(plexe, vid, position, lane, speed, vtype)

    plexe.set_path_cacc_parameters(vid, cacc_spacing, 2, 1, 0.5)
    plexe.set_cc_desired_speed(vid, speed)
    plexe.set_acc_headway_time(vid, 1.5)
    if real_engine:
        plexe.set_engine_model(vid, ENGINE_MODEL_REALISTIC)
        plexe.set_vehicles_file(vid, "vehicles.xml")
        plexe.set_vehicle_model(vid, "alfa-147")
    traci.vehicle.setColor(
        vid,
        (random.uniform(0, 255), random.uniform(0, 255), random.uniform(0, 255), 255),
    )


def get_distance(plexe, v1, v2):
    """
    Returns the distance between two vehicles, removing the length
    :param plexe: API instance
    :param v1: id of first vehicle
    :param v2: id of the second vehicle
    :return: distance between v1 and v2
    """
    v1_data = plexe.get_vehicle_data(v1)
    v2_data = plexe.get_vehicle_data(v2)
    return (
        math.sqrt(
            (v1_data[POS_X] - v2_data[POS_X]) ** 2
            + (v1_data[POS_Y] - v2_data[POS_Y]) ** 2
        )
        - 4
    )


def communicate(plexe, topology: Topology):
    """
    Performs data transfer between vehicles, i.e., fetching data from
    leading and front vehicles to feed the CACC algorithm
    :param plexe: API instance
    :param topology: a dictionary pointing each vehicle id to its front
    vehicle and platoon leader. each entry of the dictionary is a dictionary
    which includes the keys "leader" and "front"
    """
    # for vid, l in topology.platoons.items():
    for platoon_id, platoon in topology.platoons.items():
        # get data about platoon leader
        ld = plexe.get_vehicle_data(platoon.leader_id)
        for v in platoon.vehicles:
            # pass leader vehicle data to CACC
            plexe.set_leader_vehicle_data(v.id, ld)
            # pass data to the fake CACC as well, in case it's needed
            plexe.set_leader_vehicle_fake_data(v.id, ld)
            if v.front is not None:
                fd = plexe.get_vehicle_data(v.front)
                # pass front vehicle data to CACC
                plexe.set_front_vehicle_data(v.id, fd)
                # compute GPS distance and pass it to the fake CACC
                distance = get_distance(plexe, v.id, v.front)
                plexe.set_front_vehicle_fake_data(v.id, fd, distance)


def start_sumo(config_file, already_running, gui=True, sublane=True):
    """
    Starts or restarts sumo with the given configuration file
    :param config_file: sumo configuration file
    :param already_running: if set to true then the command simply reloads
    the given config file, otherwise sumo is started from scratch
    :param gui: start GUI or not
    """
    if sublane:
        arguments = ["--lanechange.duration", "3", "-c"]
    else:
        arguments = ["-c"]
    sumo_cmd = [sumolib.checkBinary("sumo-gui" if gui else "sumo")]
    arguments.append(config_file)
    if already_running:
        traci.load(arguments)
    else:
        sumo_cmd.extend(arguments)
        traci.start(sumo_cmd)


def running(demo_mode, step, max_step):
    """
    Returns whether the demo should continue to run or not. If demo_mode is
    set to true, the demo should run indefinitely, so the function returns
    true. Otherwise, the function returns true only if step <= max_step
    :param demo_mode: true if running in demo mode
    :param step: current simulation step
    :param max_step: maximum simulation step
    :return: true if the simulation should continue
    """
    if demo_mode:
        return True
    else:
        return step <= max_step


def get_status(status):
    """
    Returns a human readable representation of the lane change state of a
    vehicle
    :param status: the lane change state returned by getLaneChangeState
    """
    st = ""
    for i in range(32):
        mask = 1 << i
        if status & mask:
            if mask in bits.keys():
                st += " " + bits[mask]
            else:
                st += " 2^" + str(i)
    return st
