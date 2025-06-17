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


class Topology:
    def __init__(self):
        """
        Each `platoon` has:
        - a leader ID (str)
        - a list of Vehicle objects
        """
        self.platoons: Dict[int, Tuple[str, List[Vehicle]]] = {}
        """
         Temporary platoons have to be made to execute most maneuvers
         when these are created, we store the id of the temporary platoon as a key with the id of the platoon it branched off of as a value
        """
        self.temporaries: Dict[int, int] = {}

    def __str__(self) -> str:
        output = ["Topology:"]
        for platoon_id, (leader, vehicles) in self.platoons.items():
            output.append(f"  Platoon {platoon_id}:")
            output.append(f"    Leader: {leader}")
            output.append(f"    Vehicles:")
            for v in vehicles:
                output.append(f"      - ID: {v.id}, Front: {v.front}, Back: {v.back}")
        if self.temporaries:
            output.append("  Temporary Platoon Mappings:")
            for sub_id, og_id in self.temporaries.items():
                output.append(f"    Temporary {sub_id} -> Original {og_id}")
        return "\n".join(output)

    def create_sub_platoon(self, leader_id: str) -> int:
        # Return existing sub-platoon if already made for this leader
        for sub_id, _ in self.temporaries.items():
            if self.platoons[sub_id][0] == leader_id:
                return sub_id

        platoon_id, _ = self.get_vehicle(leader_id)
        vehicles = self.platoons[platoon_id][1]

        leader_idx = next(
            (i for i, v in enumerate(vehicles) if v.id == leader_id), None
        )
        assert leader_idx is not None, f"{leader_id} not found in platoon {platoon_id}"

        sub_platoon = self.create_platoon(leader_id)
        self.temporaries[sub_platoon] = platoon_id

        to_move = vehicles[leader_idx:]
        self.platoons[platoon_id] = (
            self.platoons[platoon_id][0],
            vehicles[:leader_idx],
        )

        for v in to_move:
            self.add_vehicle(sub_platoon, v)

        return sub_platoon

    def reset_leaders(self):
        while self.temporaries:
            sub_id, og_id = self.temporaries.popitem()
            vehicles = self.platoons[sub_id][1][:]
            # Use a copy of the list to avoid issues while modifying it
            for v in vehicles:
                self.remove_vehicle(v.id)
                self.add_vehicle(og_id, v)

    def create_platoon(self, leader_id: str) -> int:
        id = self.platoons.__len__()
        self.platoons[id] = (leader_id, [])
        return id

    def add_vehicle(self, platoon: int, vehicle: Vehicle):
        if platoon not in self.platoons:
            self.platoons[platoon] = ("", [])
        _, vehicles = self.platoons[platoon]
        vehicles.append(vehicle)

    def get_leader(self, vehicle_id: str):
        for platoon_id, (leader, vehicles) in self.platoons.items():
            for i, v in enumerate(vehicles):
                if v.id == vehicle_id:
                    del vehicles[i]
                    return leader

    def remove_vehicle(self, vehicle_id: str) -> Vehicle:
        for platoon_id, (_, vehicles) in self.platoons.items():
            for i, v in enumerate(vehicles):
                if v.id == vehicle_id:
                    return vehicles.pop(i)

    def get_vehicle(self, vehicle_id: str) -> Tuple[int, Vehicle]:
        for platoon_id, (_, vehicles) in self.platoons.items():
            for v in vehicles:
                if v.id == vehicle_id:
                    return platoon_id, v
        raise ValueError(f"Vehicle {vehicle_id} not found")

    def inPlatoon(self, vid: str) -> bool:
        try:
            platoon_id, _ = self.get_vehicle(vid)
            _, vehicles = self.platoons[platoon_id]
            return len(vehicles) == 1
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
    for platoon_id, (leader, vehicles) in topology.platoons.items():
        for i, v in enumerate(vehicles):
            if platoon_id == 0:
                lane = 0
            if platoon_id > 0:
                lane = platoon_id + 1
            add_platooning_vehicle(
                plexe,
                v.id,
                (len(vehicles) - i + 1 + platoon_id) * (DISTANCE + LENGTH) + 50,
                lane,
                SPEED,
                DISTANCE,
                real_engine,
            )
            plexe.set_fixed_lane(v.id, lane, safe=True)
            traci.vehicle.setLaneChangeMode(v.id, 0)
            traci.vehicle.setSpeedMode(v.id, 0)

            if v.id == leader:
                plexe.set_active_controller(v.id, ACC)
            else:
                plexe.set_active_controller(v.id, CACC)


# def leavePlatoon(veh: Vehicle, topology: Topology, plexe):
#     print(f"[leavePlatoon] Starting leave maneuver for {veh.id}")

#     if not topology.inPlatoon(veh.id):
#         return "Error: Vehicle is not in a platoon to leave"

#     topology = open_gap(plexe, veh.back, veh.id, JOIN_DISTANCE, topology)
#     topology = open_gap(plexe, veh.id, veh.front, JOIN_DISTANCE, topology)

#     plexe.set_active_controller(veh.id, ACC)
#     traci.vehicle.setSpeed(veh.id, SPEED)
#     plexe.set_fixed_lane(veh.id, 1, safe=False)

#     while get_distance(plexe, veh.back, veh.front) <= (JOIN_DISTANCE + 1):
#         traci.simulationStep()
#     _, back = topology.get_vehicle(veh.back)
#     _, front = topology.get_vehicle(veh.back)
#     back.front = front.id
#     front.back = back.id

#     # swithing back control scheme
#     plexe.set_active_controller(back.id, CACC)
#     plexe.set_active_controller(front.id, CACC)
#     plexe.set_path_cacc_parameters(back.id, DISTANCE)
#     plexe.set_path_cacc_parameters(front.id, DISTANCE)

#     print(f"[leavePlatoon] {veh.id} successfully left the platoon.")


# def joinPlatoon(veh: Vehicle, target_platoon: int, topology: Topology, plexe):
#     print(f"[joinPlatoon] starting for vehicle: {veh.id} to platoon: {target_platoon}")

#     if topology.inPlatoon(veh.id):
#         leavePlatoon(veh.id, topology, plexe)

#     if not veh.front or not veh.back:
#         print(
#             f"[joinPlatoon] Front or back vehicle not defined for {veh.id}, aborting."
#         )

#     topology = get_in_position(plexe, veh.id, veh.front, topology)

#     while get_distance(plexe, veh.id, veh.front) >= JOIN_DISTANCE + 1:
#         traci.simulationStep()

#     topology = open_gap(plexe, veh.back, veh.id, JOIN_DISTANCE, topology)

#     while get_distance(plexe, veh.id, veh.front) >= 2 * JOIN_DISTANCE + 2:
#         traci.simulationStep()

#     plexe.set_fixed_lane(
#         veh.id,
#         traci.vehicle.getLaneIndex(topology.get_leader(veh.front)),
#         safe=False,
#     )
#     plexe.set_active_controller(veh.id, CACC)
#     plexe.set_path_cacc_parameters(veh.id, distance=DISTANCE)
#     plexe.set_active_controller(veh.back, CACC)
#     plexe.set_path_cacc_parameters(veh.back, distance=DISTANCE)
#     topology.reset_leaders()

#     print(
#         f"[joinPlatoon] vehicle: {veh.id} successfully joined platoon: {target_platoon}."
#     )


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
    # index = int(vid.split(".")[1])
    # sub_platoon = topology.create_platoon(vid)
    # for i in range(index + 1, n):
    #     sub_vid = "v.%d" % i
    #     v = topology.remove_vehicle(sub_vid)[1]
    #     topology.platoons[sub_platoon][1].append(v)
    # temporarily change the leader
    # BAD!!
    # topology.get_vehicle("v.%d" % i)
    # topology.platoons[0][0] = vid

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


# MAY NOT BE SUPER HELPFUL OR NEEDED, GPT CODE
# def reconfigure_platoons(plexe, topology: Topology):
#     for platoon_id, (leader, vehicles) in topology.platoons.items():
#         leader_data = plexe.get_vehicle_data(leader)
#         for idx, v in enumerate(vehicles):
#             if v.id == leader:
#                 plexe.set_active_controller(v.id, ACC)
#             else:
#                 plexe.set_active_controller(v.id, CACC)
#             plexe.set_leader_vehicle_data(v.id, leader_data)
#             plexe.set_leader_vehicle_fake_data(v.id, leader_data)
#             if v.front is not None:
#                 fd = plexe.get_vehicle_data(v.front)
#                 distance = get_distance(plexe, v.id, v.front)
#                 plexe.set_front_vehicle_data(v.id, fd)
#                 plexe.set_front_vehicle_fake_data(v.id, fd, distance)


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
    for platoon_id, (leader, vehicles) in topology.platoons.items():
        # get data about platoon leader
        ld = plexe.get_vehicle_data(leader)
        for v in vehicles:
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


# ✅ Event-driven join-and-leave logic using state tracking per vehicle

# Maneuver status tracking dicts
leave_tasks = {}
join_tasks = {}


def leavePlatoon(veh: Vehicle, topology: Topology, plexe):
    print(f"[leavePlatoon] Initiating leave maneuver for {veh.id}")

    topology = open_gap(plexe, veh.back, veh.id, JOIN_DISTANCE, topology)
    topology = open_gap(plexe, veh.id, veh.front, JOIN_DISTANCE, topology)

    plexe.set_active_controller(veh.id, ACC)
    traci.vehicle.setSpeed(veh.id, SPEED)
    plexe.set_cc_desired_speed(veh.id, SPEED + 5)
    traci.vehicle.setLaneChangeMode(veh.id, 0)
    plexe.set_fixed_lane(veh.id, 1, safe=False)

    leave_tasks[veh.id] = {"veh": veh, "status": "waiting"}


def joinPlatoon(veh: Vehicle, target_platoon: int, topology: Topology, plexe):
    # If the vehicle is still in a platoon (like v.10 in platoon 1), initiate leave first
    if topology.inPlatoon(veh.id):
        print(f"[joinPlatoon] {veh.id} is still in a platoon, triggering leave first")
        leavePlatoon(veh, topology, plexe)
        # Only schedule join once
        if veh.id not in join_tasks:
            join_tasks[veh.id] = {
                "veh": veh,
                "front": veh.front,
                "back": veh.back,
                "status": "waiting_for_leave",
                "target_platoon": target_platoon,
            }
        return
        # If rejoining after leave, don't overwrite join_tasks
    if veh.id not in join_tasks or join_tasks[veh.id]["status"] == "waiting_for_leave":
        join_tasks[veh.id] = {
            "veh": veh,
            "front": veh.front,
            "back": veh.back,
            "status": "moving_to_position",
            "target_platoon": target_platoon,
        }
        return
    print(f"[joinPlatoon] Initiating join maneuver for {veh.id}")

    # Force-remove from current platoon if necessary
    try:
        topology.remove_vehicle(veh.id)
    except ValueError:
        pass

    # Set front/back explicitly if needed (can also be passed in externally)
    if not veh.front or not veh.back:
        print(f"[joinPlatoon] Missing front/back. Assigning defaults for demo.")
        veh.front = "v.5"
        veh.back = "v.4"

    # Add to solo platoon if not yet in one
    try:
        topology.get_vehicle(veh.id)
    except ValueError:
        solo_id = topology.create_platoon(veh.id)
        topology.add_vehicle(solo_id, veh)

    get_in_position(plexe, veh.id, veh.front, topology)
    plexe.set_cc_desired_speed(veh.id, SPEED + 5)

    join_tasks[veh.id] = {
        "veh": veh,
        "front": veh.front,
        "back": veh.back,
        "status": "moving_to_position",
        "target_platoon": target_platoon,
    }


def update_maneuvers(topology: Topology, plexe):
    from utils import communicate

    communicate(plexe, topology)

    # Update leave maneuvers
    for vid, task in list(leave_tasks.items()):
        veh = task["veh"]
        if get_distance(plexe, veh.back, veh.front) > JOIN_DISTANCE + 1:
            _, back = topology.get_vehicle(veh.back)
            _, front = topology.get_vehicle(veh.front)

            back.front = front.id
            front.back = back.id

            topology.remove_vehicle(veh.id)
            new_platoon = topology.create_platoon(veh.id)
            topology.add_vehicle(new_platoon, veh)

            plexe.set_active_controller(back.id, CACC)
            plexe.set_active_controller(front.id, CACC)
            plexe.set_path_cacc_parameters(back.id, DISTANCE)
            plexe.set_path_cacc_parameters(front.id, DISTANCE)

            del leave_tasks[vid]
            print(f"[leavePlatoon] {vid} completed leaving the platoon.")

    # Update join maneuvers
    for vid, task in list(join_tasks.items()):
        veh = task["veh"]
        front_id = task["front"]
        back_id = task["back"]

        print(
            f"[joinPlatoon] Distance to front: {get_distance(plexe, veh.id, front_id):.2f}"
        )

        if task["status"] == "moving_to_position":
            if get_distance(
                plexe, veh.id, front_id
            ) < JOIN_DISTANCE + 1 and traci.vehicle.getLaneID(
                veh.id
            ) == traci.vehicle.getLaneID(front_id):
                topology = open_gap(plexe, back_id, veh.id, JOIN_DISTANCE, topology)

                _, front_v = topology.get_vehicle(front_id)
                _, back_v = topology.get_vehicle(back_id)
                _, veh_v = topology.get_vehicle(veh.id)

                front_v.back = veh.id
                back_v.front = veh.id
                veh_v.front = front_id
                veh_v.back = back_id
                veh_v.leader = topology.platoons[topology.get_vehicle(front_id)[0]][0]

                task["status"] = "gap_opened"

        elif task["status"] == "waiting_for_leave":
            if veh.id not in leave_tasks:
                print(f"[joinPlatoon] {veh.id} has left, continuing join")
                task["status"] = "moving_to_position"

        elif task["status"] == "gap_opened":
            if get_distance(
                plexe, veh.id, front_id
            ) < 2 * JOIN_DISTANCE + 2 and traci.vehicle.getLaneID(
                veh.id
            ) == traci.vehicle.getLaneID(front_id):
                lane_idx = traci.vehicle.getLaneIndex(front_id)
                print(f"[joinPlatoon] Setting lane {lane_idx} for {veh.id}")

                traci.vehicle.setLaneChangeMode(veh.id, 0)
                plexe.set_fixed_lane(veh.id, lane_idx, safe=False)
                traci.vehicle.setSpeed(veh.id, SPEED)
                plexe.set_active_controller(veh.id, CACC)
                plexe.set_path_cacc_parameters(veh.id, distance=DISTANCE)
                plexe.set_active_controller(back_id, CACC)
                plexe.set_path_cacc_parameters(back_id, distance=DISTANCE)

                # Optional: force vehicle position behind front vehicle
                front_pos = plexe.get_vehicle_data(front_id)[POS_X]
                new_pos = front_pos - (DISTANCE + LENGTH)
                lane_id = traci.vehicle.getLaneID(front_id)
                traci.vehicle.moveTo(veh.id, lane_id, new_pos)

                # Reassign vehicle to correct platoon
                topology.remove_vehicle(veh.id)
                target_platoon, _ = topology.get_vehicle(front_id)
                topology.add_vehicle(target_platoon, veh)

                topology.reset_leaders()
                del join_tasks[vid]
                print(
                    f"[joinPlatoon] {vid} successfully joined platoon {task['target_platoon']}."
                )
