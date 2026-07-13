#!/usr/bin/env python3
"""
Standalone scenario runner for the Autoware-CARLA fuzzing framework.

Responsibilities:
  1. Write config/npc_scenario.yaml (picked up by aw_priviliged.py setup()).
  2. Monkey-patch BackgroundBehavior to a no-op BEFORE leaderboard is imported,
     so no random background traffic is spawned.
  3. Run the leaderboard evaluator in-process.

Usage (run from repo root after sourcing carla_envs.sh):
  python scripts/run_scenario.py
  python scripts/run_scenario.py --rel_pos REAR --motion Lane_keep_Follow --speed 30
  python scripts/run_scenario.py --no_npc
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Ensure all leaderboard/srunner packages are importable even if carla_envs.sh
# was not sourced. Prepend so our order matches carla_envs.sh.
for _p in [
    os.path.join(ROOT, 'src', 'external', 'scenario_runner'),
    os.path.join(ROOT, 'src', 'external', 'scenario_runner', 'srunner', 'tests'),
    os.path.join(ROOT, 'src', 'external', 'leaderboard'),
    os.path.join(ROOT, 'src', 'tum_agents'),
    os.path.join(ROOT, 'src', 'tum_agents', 'autoware_agent'),
    os.path.join(ROOT, 'config'),
]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

# ---------------------------------------------------------------------------
# Monkey-patch BackgroundBehavior → no-op BEFORE any leaderboard import.
# route_scenario.py does a module-level import of BackgroundBehavior, so this
# must happen before 'from leaderboard.leaderboard_evaluator import main'.
# ---------------------------------------------------------------------------
try:
    import py_trees as _pt

    class _NoopBG(_pt.behaviour.Behaviour):
        """Drop-in replacement for BackgroundBehavior that spawns nothing."""
        def __init__(self, ego_actor, route, debug=False, name="BackgroundBehavior"):
            super().__init__(name=name)
        def update(self):
            return _pt.common.Status.RUNNING
        def terminate(self, new_status):
            pass

    import srunner.scenarios.background_activity as _bg
    _bg.BackgroundBehavior = _NoopBG
    print('[run_scenario] BackgroundActivity disabled — no random traffic will spawn')
except Exception as _e:
    print(f'[run_scenario] WARNING: could not patch BackgroundBehavior: {_e}')

# ---------------------------------------------------------------------------
# Expose ScenarioManager instance globally so the agent can stop the scenario.
# ---------------------------------------------------------------------------
try:
    import leaderboard.utils.statistics_manager as _stats_mod
    _orig_save_sensors = _stats_mod.StatisticsManager.save_sensors
    def _patched_save_sensors(self, sensors):
        _orig_save_sensors(self, sensors if sensors else ['privileged_map_agent'])
    _stats_mod.StatisticsManager.save_sensors = _patched_save_sensors
except Exception as _e:
    print(f'[run_scenario] WARNING: could not patch StatisticsManager: {_e}')

try:
    import leaderboard.scenarios.scenario_manager as _sm_mod
    _sm_mod._GLOBAL_MANAGER = None
    _orig_sm_init = _sm_mod.ScenarioManager.__init__
    def _patched_sm_init(self, *args, **kwargs):
        _orig_sm_init(self, *args, **kwargs)
        _sm_mod._GLOBAL_MANAGER = self
    _sm_mod.ScenarioManager.__init__ = _patched_sm_init
except Exception as _e:
    print(f'[run_scenario] WARNING: could not patch ScenarioManager: {_e}')

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
REL_POS_CHOICES = [
    'FRONT_LEFT', 'FRONT', 'FRONT_RIGHT',
    'LEFT', 'RIGHT',
    'REAR_LEFT', 'REAR', 'REAR_RIGHT',
]
MOTION_CHOICES = [
    'Cut_in_Follow', 'Cut_in_Accel', 'Cut_in_Decel',
    'Cut_out_Follow', 'Cut_out_Accel', 'Cut_out_Decel',
    'Lane_keep_Follow', 'Lane_keep_Accel', 'Lane_keep_Decel',
]

try:
    import yaml as _yaml
except ImportError:
    sys.exit('PyYAML not found — pip install pyyaml')

NPC_YAML = os.path.join(ROOT, 'config', 'npc_scenario.yaml')
EVALUATOR_MOD = 'leaderboard.leaderboard_evaluator'


def _parse_args():
    p = argparse.ArgumentParser(
        description='Run a 1-NPC Autoware scenario on the Town01 route.',
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument('--rel_pos', default='FRONT', choices=REL_POS_CHOICES,
                   help='NPC start position relative to ego (default: FRONT)')
    p.add_argument('--motion', default='Lane_keep_Decel', choices=MOTION_CHOICES,
                   help='NPC behavior (default: Lane_keep_Decel)')
    p.add_argument('--speed', type=float, default=40.0,
                   help='NPC base speed in km/h (default: 40.0)')
    p.add_argument('--delay', type=int, default=30,
                   help='Ticks before lane-change fires (default: 30)')
    p.add_argument('--no_npc', action='store_true',
                   help='Remove npc_scenario.yaml so no NPCs are spawned')
    p.add_argument('--conf', default='config/config.yaml',
                   help='Leaderboard config path (default: config/config.yaml)')
    p.add_argument('--repeat', type=int, default=None,
                   help='Number of repetitions (overrides config repetitions field)')
    p.add_argument('--reset_time', action='store_true',
                   help='Clear persisted clock offset (use when restarting Autoware)')
    return p.parse_args()


def _write_npc_yaml(args):
    cfg = {
        'npc_vehicles': [{
            'rel_pos': args.rel_pos,
            'motion': args.motion,
            'base_speed': args.speed,
            'lane_change_delay': args.delay,
        }]
    }
    with open(NPC_YAML, 'w') as f:
        _yaml.safe_dump(cfg, f)
    print(f'[run_scenario] NPC config → {NPC_YAML}')
    print(f'               rel_pos={args.rel_pos}  motion={args.motion}'
          f'  speed={args.speed} km/h  delay={args.delay} ticks')


def main():
    args = _parse_args()

    _TIME_OFFSET_FILE = '/tmp/aw_carla_time_offset'
    if args.reset_time:
        try:
            os.remove(_TIME_OFFSET_FILE)
            print('[run_scenario] Clock offset reset — starting from t=0')
        except FileNotFoundError:
            print('[run_scenario] No clock offset file found (already at t=0)')

    if args.no_npc:
        if os.path.isfile(NPC_YAML):
            os.remove(NPC_YAML)
            print('[run_scenario] Removed npc_scenario.yaml — no NPCs')
        else:
            print('[run_scenario] No npc_scenario.yaml — running without NPCs')
    else:
        _write_npc_yaml(args)

    # Build the config path; if --repeat is set, write a patched temp config.
    conf_path = args.conf
    tmp_conf = None
    if args.repeat is not None:
        with open(os.path.join(ROOT, args.conf)) as f:
            cfg = _yaml.safe_load(f)
        cfg['evaluation']['challenge']['repetitions'] = args.repeat
        tmp_conf = os.path.join(ROOT, '.tmp_run_scenario_conf.yaml')
        with open(tmp_conf, 'w') as f:
            _yaml.safe_dump(cfg, f)
        conf_path = os.path.relpath(tmp_conf, ROOT)
        print(f'[run_scenario] Repetitions: {args.repeat}')

    # Override sys.argv so the leaderboard's own argparse sees only its flag.
    sys.argv = [sys.argv[0], f'--conf_file_path={conf_path}']

    print(f'[run_scenario] Starting leaderboard evaluator (conf={conf_path})\n')

    try:
        from leaderboard.leaderboard_evaluator import main as _eval_main
        _eval_main()  # calls sys.exit() internally
    finally:
        if tmp_conf and os.path.exists(tmp_conf):
            os.remove(tmp_conf)


if __name__ == '__main__':
    main()
