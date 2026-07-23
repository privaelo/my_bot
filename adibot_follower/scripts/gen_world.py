#!/usr/bin/env python3
"""
Generate the cluttered world and its occupancy grid from one obstacle spec.

Reads config/obstacles.yaml and writes:
  worlds/cluttered.sdf          - Gazebo world with static obstacles
  config/cluttered.map.pgm      - occupancy grid rasterized from the same
  config/cluttered.map.yaml       placements, in nav2 map_server format

Deterministic: the same spec always produces byte-identical output.

Usage:
  python3 gen_world.py [spec.yaml]
"""

import math
from pathlib import Path
import random
import sys

import yaml

PKG_ROOT = Path(__file__).resolve().parent.parent

FREE = 254
OCCUPIED = 0


class Box:
    """Axis-aligned box obstacle footprint."""

    def __init__(self, x, y, sx, sy, height):
        self.x, self.y, self.sx, self.sy, self.height = x, y, sx, sy, height
        self.footprint_radius = math.hypot(sx, sy) / 2.0
        self.color = '0.5 0.5 0.5'

    def contains(self, px, py):
        return (abs(px - self.x) <= self.sx / 2.0
                and abs(py - self.y) <= self.sy / 2.0)

    def sdf(self, name):
        return MODEL_TEMPLATE.format(
            name=name, x=self.x, y=self.y, z=self.height / 2.0,
            color=self.color,
            geometry=f'<box><size>{self.sx:.3f} {self.sy:.3f} '
                     f'{self.height:.3f}</size></box>')


class Cylinder:
    """Cylindrical obstacle footprint."""

    def __init__(self, x, y, radius, height):
        self.x, self.y, self.radius, self.height = x, y, radius, height
        self.footprint_radius = radius
        self.color = '0.5 0.5 0.5'

    def contains(self, px, py):
        return math.hypot(px - self.x, py - self.y) <= self.radius

    def sdf(self, name):
        return MODEL_TEMPLATE.format(
            name=name, x=self.x, y=self.y, z=self.height / 2.0,
            color=self.color,
            geometry=f'<cylinder><radius>{self.radius:.3f}</radius>'
                     f'<length>{self.height:.3f}</length></cylinder>')


MODEL_TEMPLATE = """\
    <model name='{name}'>
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>{geometry}</geometry>
        </collision>
        <visual name='visual'>
          <geometry>{geometry}</geometry>
          <material>
            <ambient>{color} 1</ambient>
            <diffuse>{color} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

WORLD_TEMPLATE = """\
<?xml version="1.0"?>
<sdf version='1.10'>
  <world name='{name}'>
    <physics name='1ms' type='ignored'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
    <plugin name='gz::sim::systems::UserCommands'
            filename='gz-sim-user-commands-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster'
            filename='gz-sim-scene-broadcaster-system'/>
    <plugin name='gz::sim::systems::Contact'
            filename='gz-sim-contact-system'/>
    <plugin name='gz::sim::systems::Sensors'
            filename='gz-sim-sensors-system'>
      <render_engine>ogre2</render_engine>
    </plugin>
    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <light name='sun' type='directional'>
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>-0.5 0.1 -0.9</direction>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
{models}\
  </world>
</sdf>
"""


def make_walls(world):
    """Four perimeter walls, inner face flush with the arena boundary."""
    h = world['half_extent']
    t = world['wall_thickness']
    z = world['wall_height']
    c = h - t / 2.0
    span = 2.0 * h
    walls = [
        Box(0.0, c, span, t, z),    # north
        Box(0.0, -c, span, t, z),   # south
        Box(c, 0.0, t, span, z),    # east
        Box(-c, 0.0, t, span, z),   # west
    ]
    for w in walls:
        w.color = world['wall_color']
    return walls


def point_segment_distance(px, py, ax, ay, bx, by):
    """Distance from point (px, py) to segment (ax, ay)-(bx, by)."""
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def place_obstacles(spec, rng):
    """Rejection-sample non-overlapping obstacles honoring keep-out zones."""
    world = spec['world']
    obs = spec['obstacles']
    keep_out = spec['keep_out']
    corridor = spec.get('keep_out_corridor')
    corridor_path = corridor['path'] if corridor else []
    corridor_legs = list(zip(corridor_path, corridor_path[1:] + corridor_path[:1]))
    clearance = obs['min_clearance']
    inner = world['half_extent'] - world['wall_thickness']

    placed = []
    attempts = 0
    while len(placed) < obs['count']:
        attempts += 1
        if attempts > 100000:
            sys.exit(f'error: only placed {len(placed)}/{obs["count"]} '
                     'obstacles; relax the spec (fewer/smaller obstacles or '
                     'less clearance)')
        height = rng.uniform(*obs['height_range'])
        if rng.random() < 0.5:
            sx = rng.uniform(*obs['box_side_range'])
            sy = rng.uniform(*obs['box_side_range'])
            proto = Box(0.0, 0.0, sx, sy, height)
        else:
            proto = Cylinder(0.0, 0.0,
                             rng.uniform(*obs['cylinder_radius_range']),
                             height)
        fr = proto.footprint_radius
        bound = inner - clearance - fr
        proto.x = rng.uniform(-bound, bound)
        proto.y = rng.uniform(-bound, bound)

        ok = all(math.hypot(proto.x - k['x'], proto.y - k['y']) > k['r'] + fr
                 for k in keep_out)
        ok = ok and all(
            math.hypot(proto.x - o.x, proto.y - o.y)
            > fr + o.footprint_radius + clearance
            for o in placed)
        if corridor:
            ok = ok and all(
                point_segment_distance(proto.x, proto.y, a['x'], a['y'], b['x'], b['y'])
                > corridor['radius'] + fr
                for a, b in corridor_legs)
        if ok:
            placed.append(proto)
    return placed


def write_world(spec, obstacles, path):
    models = ''.join(o.sdf(f'obstacle_{i:02d}')
                     for i, o in enumerate(obstacles))
    path.write_text(WORLD_TEMPLATE.format(name=spec['world']['name'],
                                          models=models))


def write_map(spec, obstacles, pgm_path, yaml_path):
    h = spec['world']['half_extent']
    res = spec['map']['resolution']
    size = round(2.0 * h / res)

    grid = bytearray([FREE]) * (size * size)
    for o in obstacles:
        fr = o.footprint_radius
        col_lo = max(0, int((o.x - fr + h) / res))
        col_hi = min(size - 1, int((o.x + fr + h) / res))
        row_lo = max(0, int((h - (o.y + fr)) / res))
        row_hi = min(size - 1, int((h - (o.y - fr)) / res))
        for row in range(row_lo, row_hi + 1):
            py = h - (row + 0.5) * res
            for col in range(col_lo, col_hi + 1):
                px = -h + (col + 0.5) * res
                if o.contains(px, py):
                    grid[row * size + col] = OCCUPIED

    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{size} {size}\n255\n'.encode())
        f.write(bytes(grid))

    yaml_path.write_text(
        f'image: {pgm_path.name}\n'
        'mode: trinary\n'
        f'resolution: {res}\n'
        f'origin: [{-h}, {-h}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n')


def main():
    spec_path = (Path(sys.argv[1]) if len(sys.argv) > 1
                 else PKG_ROOT / 'config' / 'obstacles.yaml')
    spec = yaml.safe_load(spec_path.read_text())

    rng = random.Random(spec['seed'])
    placed = place_obstacles(spec, rng)
    # colors are drawn after all placement samples so recoloring (or editing
    # the palette) never moves an obstacle
    for o in placed:
        o.color = rng.choice(spec['obstacles']['palette'])
    obstacles = make_walls(spec['world']) + placed

    world_path = PKG_ROOT / 'worlds' / 'cluttered.sdf'
    world_path.parent.mkdir(exist_ok=True)
    write_world(spec, obstacles, world_path)
    write_map(spec, obstacles,
              PKG_ROOT / 'config' / 'cluttered.map.pgm',
              PKG_ROOT / 'config' / 'cluttered.map.yaml')
    print(f'wrote {world_path} and cluttered.map.pgm/.yaml '
          f'({len(obstacles) - 4} obstacles + 4 walls, seed {spec["seed"]})')


if __name__ == '__main__':
    main()
