from utils import (
    get_distance,
    open_gap,
    Topology,
    Vehicle,
    get_in_position,
    DISTANCE,
    JOIN_DISTANCE,
)

from enum import Enum
from abc import abstractmethod
from plexe import CACC


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
