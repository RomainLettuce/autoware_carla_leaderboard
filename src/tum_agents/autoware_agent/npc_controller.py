import random
from enum import Enum
import carla


LANE_SPEED_KMH = 40.0   # base speed = posted limit
ACCEL_FACTOR   = 1.5    # Accel motion: speed * 1.5
DECEL_KMH      = 5.0    # Decel motion: near-stop target
MAX_SPEED_KMH  = 80.0


class Motion(Enum):
    Cut_in_Follow  = 1
    Cut_in_Accel   = 2
    Cut_in_Decel   = 3
    Cut_out_Follow = 4
    Cut_out_Accel  = 5
    Cut_out_Decel  = 6
    Lane_keep_Follow = 7
    Lane_keep_Accel  = 8
    Lane_keep_Decel  = 9


class RelPos(Enum):
    FRONT_LEFT  = 0
    FRONT       = 1
    FRONT_RIGHT = 2
    LEFT        = 3
    RIGHT       = 4
    REAR_LEFT   = 5
    REAR        = 6
    REAR_RIGHT  = 7


# (lane_offset, s_offset_m)
# lane_offset: -1=left lane, 0=same lane, +1=right lane
# s_offset_m : positive=forward, negative=backward
_POS_DEF = {
    RelPos.FRONT_LEFT:  (-1, +15),
    RelPos.FRONT:       ( 0, +15),
    RelPos.FRONT_RIGHT: (+1, +15),
    RelPos.LEFT:        (-1,   0),
    RelPos.RIGHT:       (+1,   0),
    RelPos.REAR_LEFT:   (-1, -15),
    RelPos.REAR:        ( 0, -15),
    RelPos.REAR_RIGHT:  (+1, -15),
}

# Valid motions per position (mirrors Apollo-LGSVL location_motion_map)
POSITION_MOTION_MAP = {
    RelPos.FRONT_LEFT:  [Motion.Cut_in_Follow,  Motion.Cut_in_Decel,   Motion.Lane_keep_Follow, Motion.Lane_keep_Decel],
    RelPos.FRONT:       [Motion.Cut_out_Follow, Motion.Cut_out_Decel,  Motion.Lane_keep_Follow, Motion.Lane_keep_Decel],
    RelPos.FRONT_RIGHT: [Motion.Cut_in_Follow,  Motion.Cut_in_Decel,   Motion.Lane_keep_Follow, Motion.Lane_keep_Decel],
    RelPos.LEFT:        [Motion.Cut_in_Follow,  Motion.Cut_in_Accel,   Motion.Cut_in_Decel,     Motion.Lane_keep_Follow],
    RelPos.RIGHT:       [Motion.Cut_in_Follow,  Motion.Cut_in_Accel,   Motion.Cut_in_Decel,     Motion.Lane_keep_Follow],
    RelPos.REAR_LEFT:   [Motion.Cut_in_Follow,  Motion.Cut_in_Accel,   Motion.Lane_keep_Follow, Motion.Lane_keep_Accel],
    RelPos.REAR:        [Motion.Cut_out_Follow, Motion.Cut_out_Accel,  Motion.Lane_keep_Follow, Motion.Lane_keep_Accel],
    RelPos.REAR_RIGHT:  [Motion.Cut_in_Follow,  Motion.Cut_in_Accel,   Motion.Lane_keep_Follow, Motion.Lane_keep_Accel],
}


class NPCVehicle:
    """Single NPC vehicle with declarative motion behavior."""

    def __init__(self, rel_pos: RelPos, motion: Motion,
                 base_speed: float = LANE_SPEED_KMH,
                 lane_change_delay: int = 30):
        self.rel_pos = rel_pos
        self.motion = motion
        self.base_speed = base_speed          # km/h
        self.lane_change_delay = lane_change_delay  # ticks until lane change fires

        self._actor = None
        self._tm = None
        self._tick = 0
        self._lane_changed = False
        self._started = False

    # ------------------------------------------------------------------
    def spawn(self, world: carla.World, client: carla.Client,
              ego_wp: carla.Waypoint, tm_port: int) -> bool:
        target_wp = self._resolve_waypoint(ego_wp)
        if target_wp is None:
            print(f'[NPC] No waypoint for {self.rel_pos.name} — skipping')
            return False

        bp = random.choice(world.get_blueprint_library().filter('vehicle.tesla.*'))
        if bp.has_attribute('color'):
            bp.set_attribute('color', '255,0,0')

        tf = target_wp.transform
        tf.location.z += 0.5

        try:
            self._actor = world.spawn_actor(bp, tf)
        except Exception as e:
            print(f'[NPC] spawn_actor failed: {e}')
            return False

        self._tm_port = tm_port
        self._tm = client.get_trafficmanager(tm_port)
        # Physical hold: hand brake ON, autopilot OFF — TM set_desired_speed(0)
        # is unreliable when autopilot is already active.
        self._actor.apply_control(carla.VehicleControl(hand_brake=True))

        print(f'[NPC] Spawned {self.rel_pos.name} | {self.motion.name} | '
              f'held (waiting for ego to move)')
        return True

    # ------------------------------------------------------------------
    def update(self, ego_speed_ms: float = 0.0):
        """Call once per run_step tick. ego_speed_ms: ego velocity in m/s."""
        if self._actor is None:
            return

        if not self._started:
            if ego_speed_ms > 0.5:  # ~1.8 km/h — ego has left standstill
                self._started = True
                # Release hand brake and hand control to TrafficManager
                self._actor.apply_control(carla.VehicleControl(hand_brake=False))
                self._actor.set_autopilot(True, self._tm_port)
                self._tm.auto_lane_change(self._actor, False)
                self._tm.ignore_lights_percentage(self._actor, 100)
                self._tm.ignore_signs_percentage(self._actor, 100)
                self._tm.set_desired_speed(self._actor, self._target_speed_kmh())
                print(f'[NPC] Released — {self._target_speed_kmh():.1f} km/h '
                      f'(ego speed {ego_speed_ms:.1f} m/s)')
            return

        self._tick += 1
        if self._lane_changed or self._tick < self.lane_change_delay:
            return

        name = self.motion.name
        if name.startswith('Cut_in'):
            lane_offset, _ = _POS_DEF[self.rel_pos]
            # NPC on right (lane_offset>0) → change left toward ego lane, and vice versa
            go_left = (lane_offset > 0)
            self._tm.force_lane_change(self._actor, go_left)
            self._lane_changed = True
            print(f'[NPC] Cut-in {"left" if go_left else "right"}')

        elif name.startswith('Cut_out'):
            lane_offset, _ = _POS_DEF[self.rel_pos]
            # NPC in same/left lane → go left (away from ego)
            go_left = (lane_offset <= 0)
            self._tm.force_lane_change(self._actor, go_left)
            self._lane_changed = True
            print(f'[NPC] Cut-out {"left" if go_left else "right"}')

    # ------------------------------------------------------------------
    def destroy(self):
        if self._actor is not None:
            self._actor.set_autopilot(False)
            self._actor.destroy()
            self._actor = None

    # ------------------------------------------------------------------
    def _target_speed_kmh(self) -> float:
        name = self.motion.name
        if 'Accel' in name:
            return min(self.base_speed * ACCEL_FACTOR, MAX_SPEED_KMH)
        elif 'Decel' in name:
            return DECEL_KMH
        return self.base_speed

    def _resolve_waypoint(self, ego_wp: carla.Waypoint):
        lane_offset, s_offset = _POS_DEF[self.rel_pos]
        wp = ego_wp

        for _ in range(abs(lane_offset)):
            next_wp = wp.get_left_lane() if lane_offset < 0 else wp.get_right_lane()
            if next_wp is None or next_wp.lane_type != carla.LaneType.Driving:
                return None
            wp = next_wp

        if s_offset > 0:
            candidates = wp.next(abs(s_offset))
        elif s_offset < 0:
            candidates = wp.previous(abs(s_offset))
        else:
            candidates = [wp]

        if not candidates:
            return None
        result = candidates[0]
        if result.lane_type != carla.LaneType.Driving:
            return None
        return result


class NPCScenario:
    """Container for multiple NPCVehicle instances."""

    def __init__(self, npcs: list):
        self._npcs = npcs
        self._spawned = []

    def spawn_all(self, world: carla.World, client: carla.Client,
                  ego_vehicle: carla.Vehicle, carla_map: carla.Map,
                  tm_port: int = 8000) -> int:
        ego_wp = carla_map.get_waypoint(ego_vehicle.get_location())
        for npc in self._npcs:
            if npc.spawn(world, client, ego_wp, tm_port):
                self._spawned.append(npc)
        print(f'[NPC] {len(self._spawned)}/{len(self._npcs)} vehicles spawned')
        return len(self._spawned)

    def update(self, ego_speed_ms: float = 0.0):
        for npc in self._spawned:
            npc.update(ego_speed_ms)

    def destroy(self):
        for npc in self._spawned:
            npc.destroy()
        self._spawned.clear()

    # ------------------------------------------------------------------
    @staticmethod
    def random(n: int = 1, base_speed: float = LANE_SPEED_KMH,
               allowed_positions=None) -> 'NPCScenario':
        """Generate a random scenario with n NPC vehicles."""
        positions = list(RelPos) if allowed_positions is None else allowed_positions
        npcs = []
        used_pos = set()
        for _ in range(n):
            available = [p for p in positions if p not in used_pos]
            if not available:
                break
            pos = random.choice(available)
            used_pos.add(pos)
            motion = random.choice(POSITION_MOTION_MAP[pos])
            speed = base_speed + random.uniform(-5.0, 5.0)
            npcs.append(NPCVehicle(pos, motion, speed))
        return NPCScenario(npcs)
