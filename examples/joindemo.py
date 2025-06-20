#!/usr/bin/env python
import os
import random
import sys

from utils import (
    communicate,
    get_distance,
    start_sumo,
    running,
    init_simulation,
    init_topology,
    open_gap,
    add_platooning_vehicle,
    Topology,
    Vehicle,
    get_in_position,
)

from enum import Enum
from typing import List

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
from plexe import Plexe, ACC, CACC, RPM, GEAR
from abc import abstractmethod

# vehicle length
LENGTH = 4
# inter-vehicle distance
DISTANCE = 5
# inter-vehicle distance when leaving space for joining
JOIN_DISTANCE = DISTANCE * 2
# cruising speed
SPEED = 120 / 3.6

N_VEHICLES = 8


class Maneuver:
    def __init__(self, v: Vehicle):
        self.vehicle = v

    @abstractmethod
    def update(self, plexe, topology: Topology):
        pass


class JoinManeuver(Maneuver):
    class State(Enum):
        WAITING = 0
        OPENING_GAP = 1
        GOING_TO_POSITION = 2
        COMPLETED = 3

        def __str__(self):
            match self:
                case JoinManeuver.State.WAITING:
                    return "WAITING"
                case JoinManeuver.State.OPENING_GAP:
                    return "OPENING_GAP"
                case JoinManeuver.State.GOING_TO_POSITION:
                    return "GOING_TO_POSITION"
                case JoinManeuver.State.COMPLETED:
                    return "COMPLETED"

    def __init__(
        self, vehicle: Vehicle, target_platoon: int, join_idx: int, topology: Topology
    ):
        super().__init__(vehicle)
        self.target_platoon = target_platoon
        self.join_idx = join_idx
        self.state = JoinManeuver.State.WAITING
        in_platoon, _ = topology.get_vehicle(self.vehicle.id)
        assert in_platoon != self.target_platoon
        platoon_vehicles = topology.platoons[self.target_platoon][1]
        if (front_join := platoon_vehicles[self.join_idx]) is None:
            raise ValueError("Vehicle has no front!")
        # if (back_join := platoon_vehicles[self.join_idx - 1]) is None:
        #     raise ValueError("Vehicle has no back!")
        self.front_join = front_join
        print("front", front_join)
        _, self.back_join = topology.get_vehicle(front_join.back)
        print("back", self.back_join)

    def update(self, plexe, topology: Topology):
        print("STATE: ", self.state)
        if self.state == JoinManeuver.State.WAITING:
            # at 1 second, let the joiner get closer to the platoon
            topology = get_in_position(
                plexe, self.vehicle.id, self.front_join.id, topology
            )
            self.state = JoinManeuver.State.GOING_TO_POSITION

        elif self.state == JoinManeuver.State.GOING_TO_POSITION:
            # when the distance of the joiner is small enough, let the others
            # open a gap to let the joiner enter the platoon
            if (
                get_distance(plexe, self.vehicle.id, self.front_join.id)
                < JOIN_DISTANCE + 1
            ):
                self.state = JoinManeuver.State.OPENING_GAP
                topology = open_gap(
                    plexe, self.back_join.id, self.vehicle.id, JOIN_DISTANCE, topology
                )
        elif self.state == JoinManeuver.State.OPENING_GAP:
            # when the gap is large enough, complete the maneuver
            if (
                get_distance(plexe, self.back_join.id, self.front_join.id)
                > 2 * JOIN_DISTANCE - 2
            ):
                self.state = JoinManeuver.State.COMPLETED
                plexe.set_fixed_lane(self.vehicle.id, 0, safe=False)
                plexe.set_active_controller(self.vehicle.id, CACC)
                plexe.set_path_cacc_parameters(self.vehicle.id, distance=DISTANCE)
                plexe.set_active_controller(self.back_join.id, CACC)
                plexe.set_path_cacc_parameters(self.back_join.id, distance=DISTANCE)
                topology.reset_leaders()


def main(demo_mode, real_engine, setter=None):
    # used to randomly color the vehicles
    random.seed(1)
    start_sumo("cfg/freeway.sumo.cfg", False)
    plexe = Plexe()
    step = 0
    topology = init_topology([N_VEHICLES, 1])
    print(topology)

    joiner = topology.platoons[1][1][0]
    print("GOT JOINER: ", joiner)

    mans: List[Maneuver] = []

    while running(demo_mode, step, 6000):
        # when reaching 60 seconds, reset the simulation when in demo_mode
        if demo_mode and step == 6000:
            start_sumo("cfg/freeway.sumo.cfg", True)
            step = 0
            # state = WAITING
            random.seed(1)

        traci.simulationStep()

        if len(mans) > 0:
            for man in mans:
                man.update(plexe, topology)

        if step == 0:
            # create vehicles and track the joiner
            init_simulation(plexe, topology, real_engine)
            traci.gui.trackVehicle("View #0", joiner.id)
            traci.gui.setZoom("View #0", 20000)

        if step % 10 == 1:
            # simulate vehicle communication every 100 ms
            communicate(plexe, topology)

        if step == 100:
            mans.append(JoinManeuver(joiner, 0, 4, topology))

        if real_engine and setter is not None:
            # if we are running with the dashboard, update its values
            tracked_id = traci.gui.getTrackedVehicle("View #0")
            if tracked_id != "":
                ed = plexe.get_engine_data(tracked_id)
                vd = plexe.get_vehicle_data(tracked_id)
                setter(ed[RPM], ed[GEAR], vd.speed, vd.acceleration)

        step += 1

    traci.close()


if __name__ == "__main__":
    main(True, False)
