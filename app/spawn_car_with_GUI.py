#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Modifications (c) 2026 School of Computer Science, Hangzhou Dianzi University

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    CTRL + W     : toggle constant velocity mode at 60 km/h

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)

    O            : open/close all doors of vehicle
    T            : toggle vehicle's telemetry

    V            : Select next map layer (Shift+V reverse)
    B            : Load current selected map layer (Shift+B to unload)

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit

    Important function:
    P            : toggle autopilot
    Backspace    : restart
"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

# try:
#     # sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
#     #     sys.version_info.major,
#     #     sys.version_info.minor,
#     #     'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
#
#     sys.path.append('D:\CARLA0.9.15\wzq_carla0.9.15_ws\carla\dist\carla-0.9.15-py3.7-win-amd64.egg')
#
#
#
# except IndexError:
#     pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_f
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_o
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_t
    from pygame.locals import K_u
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_y
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import cv2
except ImportError:
    raise RuntimeError('cannot import cv2, make sure opencv-python package is installed')

# 检查 py360convert 是否可用
try:
    import py360convert
    HAS_PY360CONVERT = True
except ImportError:
    HAS_PY360CONVERT = False
    print("Warning: py360convert not found, panorama stitching will use simple concatenation")

import threading
import time
import datetime
import weakref
import os
from concurrent.futures import ThreadPoolExecutor

# ==============================================================================
# -- 我的函数 ----------------------------------------------------------
# ==============================================================================

# 清除车辆，传感器
def delete_all_actor(args):
    client = carla.Client(args.host, args.port)
    # 获取世界对象
    world = client.get_world()
    # 获取所有演员
    actors = world.get_actors()
    for actor in actors:
        try:
            # 不清除 spectator 或 ego vehicle（可选）
            if actor.type_id != 'spectator':
                actor.destroy()
                print(f'已清除 actor: {actor.id} - {actor.type_id}')
        except Exception as e:
            print(f'无法清除 actor: {actor.id}, 原因: {e}')

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.sync = args.sync
        self.args = args  # 保存args对象，用于后续操作
        # 设置车辆名字
        self.actor_role_name = args.rolename
        #设置随机生成
        self.random_spawn = args.random_spawn
        # 若固定位置，设置坐标
        self.spawn_point = args.spawn_point

        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.keyboard_control = None  # 保存键盘控制对象引用
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self._gamma = args.gamma
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False
        self.show_vehicle_telemetry = False
        self.doors_are_open = False
        self.current_map_layer = 0
        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]
        self.args = args

        # 全景图拍摄相关变量
        self.image_size = 1024  # 相机图像尺寸
        self.camera_height = 2.0  # 相机高度
        self.panorama_sensors = {}  # 6个RGB相机
        self.panorama_images = {'front': None, 'back': None, 'left': None, 'right': None, 'top': None, 'bottom': None}
        self.semseg_sensors = {}  # 6个语义分割相机
        self.semseg_images = {'front': None, 'back': None, 'left': None, 'right': None, 'top': None, 'bottom': None}
        self.semseg_raw_images = {'front': None, 'back': None, 'left': None, 'right': None, 'top': None, 'bottom': None}
        self.depth_sensors = {}  # 6个深度图相机
        self.depth_raw_images = {'front': None, 'back': None, 'left': None, 'right': None, 'top': None, 'bottom': None}
        self.panorama_lock = threading.Lock()
        self.panorama_frame_ids = {}
        self.semseg_frame_ids = {}
        self.depth_frame_ids = {}
        
        # 拍摄同步控制
        self._target_frame = None
        self.is_capturing = False
        self._capture_start_time = None  # 拍摄开始时间，用于超时检测
        self.capture_count = 0
        self._capture_folders = []  # 记录所有拍摄文件夹
        self.output_folder = getattr(args, 'output', './panorama_output')  # 输出目录
        self.panorama_width = 8192
        self._manual_capture_mode = True  # 预留字段（开源简化版不启用）
        
        # 自动拍摄/全自动流程相关变量（开源简化版不启用）
        self.auto_capture_mode = False
        self.last_capture_location = None
        self.total_distance_since_capture = 0.0
        self.auto_capture_interval = 10  # 默认间隔 10 米（预留）
        self._prev_location = None  # 用于计算行驶里程
        self.full_auto_workflow_active = False  # 全自动流程状态（预留）
        self.full_auto_target_count = 100  # 目标拍摄数量（预留）
        self.full_auto_start_count = 0  # 流程开始时的拍摄计数
        self._in_retake_phase = False  # 是否在复拍阶段（预留）
        self._pending_autopilot_restore = False  # 是否需要恢复自动驾驶（预留）
        self._capture_locations = []  # 记录拍摄位置信息
        self._capture_retry_count = 0  # 拍摄重试计数
        self._max_capture_retries = 3  # 最大重试次数

    def restart(self):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0


        blueprint_list = get_actor_blueprints(self.world, self._actor_filter, self._actor_generation)
        if not blueprint_list:
            raise ValueError("Couldn't find any blueprints with the specified filters")
        blueprint = random.choice(blueprint_list)

        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('terramechanics'):
            blueprint.set_attribute('terramechanics', 'true')
        # 固定设置为黑色
        if blueprint.has_attribute('color'):
            colors = blueprint.get_attribute('color').recommended_values
            black_color = None
            for c in colors:
                # 黑色通常是 RGB 值较低的颜色
                if '0,0,0' in c or 'black' in c.lower():
                    black_color = c
                    break
            # 如果没找到黑色，选择第一个颜色
            if black_color is None and colors:
                print("Warning: No black color found, using first color instead.")
                black_color = colors[0]
            if black_color:
                blueprint.set_attribute('color', black_color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])

        # Spawn the player.生成车辆（修改）
        if self.player is not None:# 如果已经存在车辆
            self.destroy()
            delete_all_actor(self.args)
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            if self.random_spawn == "False":
                # print("采用固定点生成") #覆盖掉随机点
                spawn_point.location.x = self.spawn_point.x
                spawn_point.location.y = self.spawn_point.y 
                spawn_point.location.z = self.spawn_point.z
                spawn_point.rotation.yaw = self.spawn_point.yaw
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            # self.modify_vehicle_physics(self.player)

        # 如果没有车辆（没改变，应该用不上）
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            if self.random_spawn == "False":
                # print("采用固定点生成") #覆盖掉随机点
                spawn_point.location.x = self.spawn_point.x
                spawn_point.location.y = self.spawn_point.y 
                spawn_point.location.z = self.spawn_point.z
                spawn_point.rotation.yaw = self.spawn_point.yaw
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            # self.modify_vehicle_physics(self.player)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def next_map_layer(self, reverse=False):
        self.current_map_layer += -1 if reverse else 1
        self.current_map_layer %= len(self.map_layer_names)
        selected = self.map_layer_names[self.current_map_layer]
        self.hud.notification('LayerMap selected: %s' % selected)

    def load_map_layer(self, unload=False):
        selected = self.map_layer_names[self.current_map_layer]
        if unload:
            self.hud.notification('Unloading map layer: %s' % selected)
            self.world.unload_map_layer(selected)
        else:
            self.hud.notification('Loading map layer: %s' % selected)
            self.world.load_map_layer(selected)

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()

    def update_distance_traveled(self):
        """计算累计行驶距离（行驶里程）"""
        if self.player is None:
            return
        current_loc = self.player.get_location()
        if self._prev_location is not None:
            # 计算与上一帧的距离（累计行驶里程）
            dist = current_loc.distance(self._prev_location)
            self.total_distance_since_capture += dist
        self._prev_location = current_loc

    def toggle_auto_capture(self):
        """自动拍摄入口（开源简化版已禁用）"""
        self.hud.notification("自动拍摄已在开源简化版中移除", seconds=3.0)
        print("[提示] 自动拍摄已在开源简化版中移除，如需完整功能请联系作者 1409151182@qq.com")
        return

    def capture_at_distance(self):
        """在指定距离处触发全景拍摄"""
        # 使用 setup_panorama_cameras 初始化相机（如果还没初始化）
        if not self.panorama_sensors:
            self.setup_panorama_cameras()
        # 触发全景拍摄
        self.capture_panorama()

    # ===========================================================================
    # -- 全景图拍摄功能 -----------------------------------------------------------
    # ===========================================================================

    def setup_panorama_cameras(self):#初始化全景相机
        """Setup 6 RGB cameras + 6 semantic segmentation cameras"""
        if self.panorama_sensors:#如果全景相机已初始化，跳过
            return  # 已初始化，跳过
        
        camera_configs = [#全景相机配置
            {'name': 'front',  'yaw': 0,   'pitch': 0,   'roll': 0},
            {'name': 'back',   'yaw': 180, 'pitch': 0,   'roll': 0},
            {'name': 'left',   'yaw': -90, 'pitch': 0,   'roll': 0},
            {'name': 'right',  'yaw': 90,  'pitch': 0,   'roll': 0},
            {'name': 'top',    'yaw': 0,   'pitch': 90,  'roll': 0},
            {'name': 'bottom', 'yaw': 0,   'pitch': -90, 'roll': 0},
        ]

        world = self.player.get_world()#获取世界
        bp_library = world.get_blueprint_library() #获取蓝图库
        
        # RGB 相机蓝图
        cam_bp = bp_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(self.image_size))
        cam_bp.set_attribute('image_size_y', str(self.image_size))
        cam_bp.set_attribute('fov', '90')
        cam_bp.set_attribute('gamma', str(self._gamma))
        
        # 语义分割相机蓝图
        semseg_bp = bp_library.find('sensor.camera.semantic_segmentation')#获取语义分割相机蓝图
        semseg_bp.set_attribute('image_size_x', str(self.image_size))
        semseg_bp.set_attribute('image_size_y', str(self.image_size))
        semseg_bp.set_attribute('fov', '90')

        # 深度图相机蓝图
        depth_bp = bp_library.find('sensor.camera.depth')# 获取深度图相机蓝图
        depth_bp.set_attribute('image_size_x', str(self.image_size))
        depth_bp.set_attribute('image_size_y', str(self.image_size))
        depth_bp.set_attribute('fov', '90')

        bb = self.player.bounding_box#获取车辆包围盒
        bb_location = bb.location#获取车辆包围盒位置
        bb_extent = bb.extent#获取车辆包围盒尺寸

        camera_z = bb_location.z + bb_extent.z + 1#计算相机高度
        self.camera_height = camera_z#设置相机高度

        for config in camera_configs:#遍历全景相机配置
            cam_transform = carla.Transform(#计算相机变换
                carla.Location(x=0, y=0, z=camera_z),
                carla.Rotation(yaw=config['yaw'], pitch=config['pitch'], roll=config['roll'])
            )

            # 创建 RGB 相机
            sensor = world.spawn_actor(
                cam_bp,
                cam_transform,
                attach_to=self.player,
                attachment_type=carla.AttachmentType.Rigid
            )
            self.panorama_sensors[config['name']] = sensor
            self.panorama_images[config['name']] = None

            weak_self = weakref.ref(self)
            
            def make_rgb_callback(captured_name):#创建RGB图像回调
                def callback(image):#回调函数
                    self = weak_self()
                    if not self:#如果self为空，返回
                        return
                    # 如果不在拍摄状态，跳过
                    if not self.is_capturing:#如果不在拍摄状态，返回
                        return
                    # 如果没有设置目标帧，先设置它
                    if self._target_frame is None:
                        self._target_frame = image.frame + 2
                        print(f"[全景] {captured_name} 设置目标帧: {self._target_frame}")
                    # 只处理目标帧附近的帧
                    if image.frame < self._target_frame - 2:
                        return
                    if image.frame > self._target_frame + 10:
                        return
                    self._on_panorama_image_impl(image, captured_name)#处理全景图像回调
                return callback
            
            sensor.listen(make_rgb_callback(config['name']))
            
            # 创建语义分割相机
            semseg_sensor = world.spawn_actor(#创建语义分割相机
                semseg_bp,#语义分割相机蓝图
                cam_transform,#相机变换
                attach_to=self.player,#附加到车辆
                attachment_type=carla.AttachmentType.Rigid#附加类型为刚性
            )
            self.semseg_sensors[config['name']] = semseg_sensor#设置语义分割相机
            self.semseg_images[config['name']] = None#设置语义分割图像
            self.semseg_raw_images[config['name']] = None#设置语义分割原始图像
            
            def make_semseg_callback(captured_name):
                def callback(image):
                    self = weak_self()
                    if not self:
                        return
                    if not self.is_capturing:
                        return
                    self._on_semseg_image_impl(image, captured_name)
                return callback
            
            semseg_sensor.listen(make_semseg_callback(config['name']))
            
            # 创建深度图相机
            depth_sensor = world.spawn_actor(
                depth_bp,
                cam_transform,
                attach_to=self.player,
                attachment_type=carla.AttachmentType.Rigid
            )
            self.depth_sensors[config['name']] = depth_sensor
            self.depth_raw_images[config['name']] = None
            
            def make_depth_callback(captured_name):
                def callback(image):
                    self = weak_self()
                    if not self:
                        return
                    if not self.is_capturing:
                        return
                    self._on_depth_image_impl(image, captured_name)
                return callback
            
            depth_sensor.listen(make_depth_callback(config['name']))

        print(f"[全景] Created {len(self.panorama_sensors)} RGB + {len(self.semseg_sensors)} semseg + {len(self.depth_sensors)} depth cameras")
        
        # 验证相机是否都创建成功
        for name in ['front', 'back', 'left', 'right', 'top', 'bottom']:
            rgb = self.panorama_sensors.get(name)
            sem = self.semseg_sensors.get(name)
            depth = self.depth_sensors.get(name)
            print(f"[全景] {name}: RGB={'✓' if rgb else '✗'}, SemSeg={'✓' if sem else '✗'}, Depth={'✓' if depth else '✗'}")

    def _on_panorama_image_impl(self, image, captured_name):
        """处理全景图像回调"""
        image_data = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        image_data = np.reshape(image_data, (image.height, image.width, 4))
        image_data = image_data[:, :, :3]
        image_data = image_data[:, :, ::-1]
        self.panorama_images[captured_name] = image_data
        self.panorama_frame_ids[captured_name] = image.frame
        self._check_all_panorama_received()

    def _on_semseg_image_impl(self, image, captured_name):
        """处理语义分割图像回调"""
        image.convert(cc.CityScapesPalette)
        image_data = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        image_data = np.reshape(image_data, (image.height, image.width, 4))
        image_data = image_data[:, :, :3]
        # CityScapesPalette 转换后是 BGR 格式，需要翻转成 RGB
        image_data = image_data[:, :, ::-1]
        self.semseg_images[captured_name] = image_data
        self.semseg_frame_ids[captured_name] = image.frame

    def _on_depth_image_impl(self, image, captured_name):
        """处理深度图图像回调"""
        image_data = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        image_data = np.reshape(image_data, (image.height, image.width, 4))
        image_data = image_data[:, :, :3]
        self.depth_raw_images[captured_name] = image_data
        self.depth_frame_ids[captured_name] = image.frame

    def _check_all_panorama_received(self):
        """检查是否收到所有6个方向的图像"""
        # 如果不在拍摄状态，跳过
        if not self.is_capturing:
            return
            
        required_directions = ['front', 'back', 'left', 'right', 'top', 'bottom']
        
        # 检查 RGB
        rgb_ok = all(self.panorama_images.get(d) is not None for d in required_directions)
        # 检查语义分割
        semseg_ok = all(self.semseg_images.get(d) is not None for d in required_directions)
        # 检查深度图
        depth_ok = all(self.depth_raw_images.get(d) is not None for d in required_directions)
        
        if rgb_ok and semseg_ok and depth_ok:
            self._save_panorama_data()

    def _save_panorama_data(self):
        """保存全景数据到磁盘"""
        # 防止同一帧多次保存
        if self.is_capturing == 'saving':
            return
            
        self.is_capturing = 'saving'  # 标记为正在保存
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        folder_name = f"capture_{self.capture_count + 1}_{timestamp}"
        folder_path = os.path.join(self.output_folder, folder_name)
        os.makedirs(folder_path, exist_ok=True)
        
        # 创建子文件夹分类保存不同类型的图像
        rgb_folder = os.path.join(folder_path, 'rgb')
        semseg_folder = os.path.join(folder_path, 'semseg')
        depth_folder = os.path.join(folder_path, 'depth')
        os.makedirs(rgb_folder, exist_ok=True)
        os.makedirs(semseg_folder, exist_ok=True)
        os.makedirs(depth_folder, exist_ok=True)
        
        loc = self.player.get_location()
        rot = self.player.get_transform().rotation
        
        # 记录位置信息
        location_info = {
            'folder': folder_path,
            'rgb_folder': rgb_folder,
            'semseg_folder': semseg_folder,
            'depth_folder': depth_folder,
            'location': {'x': loc.x, 'y': loc.y, 'z': loc.z},
            'rotation': {'pitch': rot.pitch, 'yaw': rot.yaw, 'roll': rot.roll}
        }
        if not hasattr(self, '_capture_locations'):
            self._capture_locations = []
        self._capture_locations.append(location_info)
        
        # 保存位置信息到文件
        with open(os.path.join(folder_path, 'location.txt'), 'w') as f:
            f.write(f"X={loc.x}\nY={loc.y}\nZ={loc.z}\n")
            f.write(f"Pitch={rot.pitch}\nYaw={rot.yaw}\nRoll={rot.roll}\n")
            # 添加GNSS数据
            if self.gnss_sensor:
                f.write(f"GNSS_Lat={self.gnss_sensor.lat:.8f}\n")
                f.write(f"GNSS_Lon={self.gnss_sensor.lon:.8f}\n")
                f.write(f"GNSS_Alt={self.gnss_sensor.alt:.4f}\n")
        
        # 保存6个方向的原始图像（分类保存）
        for direction in ['front', 'back', 'left', 'right', 'top', 'bottom']:
            if self.panorama_images.get(direction) is not None:
                cv2.imwrite(
                    os.path.join(rgb_folder, f'{direction}.png'),
                    cv2.cvtColor(self.panorama_images[direction], cv2.COLOR_RGB2BGR)
                )
            # 语义分割图像是RGB格式，直接保存（不要转换）
            if self.semseg_images.get(direction) is not None:
                cv2.imwrite(
                    os.path.join(semseg_folder, f'{direction}.png'),
                    self.semseg_images[direction]
                )
            if self.depth_raw_images.get(direction) is not None:
                cv2.imwrite(
                    os.path.join(depth_folder, f'{direction}.png'),
                    cv2.cvtColor(self.depth_raw_images[direction], cv2.COLOR_RGB2BGR)
                )
        
        self.capture_count += 1
        
        print(f"[全景] Saved {folder_name} ({loc.x:.1f}, {loc.y:.1f})")
        self.hud.notification(f"Capture #{self.capture_count}", seconds=2.0)
        
        # 重置拍摄状态
        self.is_capturing = False
        self._target_frame = None
        self._capture_start_time = None
        
        # 检查全自动流程进度（仅在初始拍摄阶段检查）
        if self.full_auto_workflow_active and not self._in_retake_phase:
            self.check_full_auto_workflow_progress()
        
        # 设置标志，让game_loop在安全的位置恢复自动驾驶
        if self.auto_capture_mode:
            self._pending_autopilot_restore = True

    def capture_panorama(self, disable_autopilot=True):
        """触发全景拍摄
        
        Args:
            disable_autopilot: 是否关闭自动驾驶。如果为False，则不调用set_autopilot(False)，
                            用于在已经刹停的情况下避免RPC超时
        """
        if not self.is_capturing:#如果不在拍摄状态
            # 开源简化版不包含“车辆停止拍照”的核心逻辑，仅保留基础采集流程
            
            # 清除旧数据
            self.panorama_images = {k: None for k in self.panorama_images}#清除全景图像
            self.semseg_images = {k: None for k in self.semseg_images}#清除语义分割图像
            self.depth_raw_images = {k: None for k in self.depth_raw_images}#清除深度图图像
            self.panorama_frame_ids = {}#清除全景帧ID
            self.semseg_frame_ids = {}#清除语义分割帧ID
            self.depth_frame_ids = {}
            
            # 设置目标帧号 - 等待下一次 tick 后获取帧号
            # 在同步模式下，帧号由外部 game_loop 管理
            self._target_frame = None  # 先设置为None，在下一次tick时更新
            
            loc = self.player.get_location()
            print(f"[全景] Capturing at ({loc.x:.1f}, {loc.y:.1f})...")
            
            self.is_capturing = True#设置拍摄状态为正在拍摄
            self._capture_start_time = time.time()#设置拍摄开始时间
            self.hud.notification('Capturing...', seconds=1.0)#显示拍摄通知

    def check_full_auto_workflow_progress(self):
        """检查全自动流程进度"""
        if not self.full_auto_workflow_active:
            return
        
        # 如果已经在复拍阶段，不再检查
        if self._in_retake_phase:
            return
        
        captured_count = self.capture_count - self.full_auto_start_count
        
        if captured_count >= self.full_auto_target_count:
            print(f"\n[全自动] 已完成 {captured_count} 张有障碍物拍摄")
            print("[全自动] 开始无障碍物复拍...")
            self._start_retake_phase()

    def _start_retake_phase(self):#开始无障碍物复拍阶段
        """开始无障碍物复拍阶段"""
        self._in_retake_phase = True
        
        # 从_capture_locations获取需要复拍的位置
        target_count = self.capture_count - self.full_auto_start_count#获取需要复拍的位置数量
        self._retake_locations = list(self._capture_locations[-target_count:]) if hasattr(self, '_capture_locations') and self._capture_locations else []#获取需要复拍的位置列表
        self._retake_index = 0#复拍索引
        
        # 关闭自动拍摄模式
        if self.auto_capture_mode:
            self.auto_capture_mode = False
        
        # 刹停车辆
        self._apply_emergency_brake()#应用紧急刹车
        
        # 开始复拍
        self._do_next_retake()#开始下一个复拍

    def _do_next_retake(self):
        """执行下一个复拍任务"""
        if self._retake_index >= len(self._retake_locations):
            # 所有复拍完成，开始全景拼接
            print("\n[全自动] 所有复拍完成，拼接全景...")
            self.hud.notification("All retakes done, stitching...", seconds=5.0)
            self.batch_stitch_all()

            # === 流程结束，关闭所有自动模式，让用户可以手动控制 ===
            self.full_auto_workflow_active = False
            self._in_retake_phase = False
            self.auto_capture_mode = False  # 关闭自动拍摄模式
            self.player.set_autopilot(False) # 关闭自动驾驶

            # 等待车辆静止后恢复物理模拟，允许用户手动控制
            self._release_vehicle_control()

            print("\n[全自动] 全部完成！车辆已停止。现在可以手动控制车辆。")
            print("[提示] 再次按下 U 键可开始新的数据采集流程")
            return
        
        loc_info = self._retake_locations[self._retake_index]#获取复拍位置
        loc = loc_info['location']#获取位置
        rot = loc_info['rotation']#获取旋转
        
        print(f"\n[全自动] 复拍 {self._retake_index + 1}/{len(self._retake_locations)}: ({loc['x']:.1f}, {loc['y']:.1f})")#打印复拍进度
        
        # 1. 只在第一次清除障碍物（避免重复RPC调用导致超时）
        if self._retake_index == 0:
            self._clear_obstacles()#debug
        else:
            print("[全自动] 跳过障碍物清理（已清理过）")#打印跳过障碍物清理
        
        # 2. 移动车辆到原始拍摄位置
        carla_transform = carla.Transform(
            carla.Location(x=loc['x'], y=loc['y'], z=loc['z']),
            carla.Rotation(pitch=rot['pitch'], yaw=rot['yaw'], roll=rot['roll'])
        )
        self.player.set_transform(carla_transform)#设置车辆位置
        print(f"[全自动] 已移动到位置: ({loc['x']:.1f}, {loc['y']:.1f}, {loc['z']:.1f})")#打印移动到位置
        
        # 3. 延迟后开始拍摄
        threading.Timer(1.0, lambda: self._execute_retake_capture(loc_info)).start()

    def _execute_retake_capture(self, loc_info):
        """执行复拍摄影"""
        loc = loc_info['location']
        print(f"[全自动] 开始拍摄: ({loc['x']:.1f}, {loc['y']:.1f})")
        # 复拍阶段车辆已经被刹停，不需要再关闭自动驾驶
        self.capture_panorama(disable_autopilot=False)
        self._retake_index += 1
        # 延迟开始下一个复拍
        threading.Timer(1.0, self._do_next_retake).start()

    def _clear_obstacles(self):
        """清除所有动态障碍物（只销毁车辆和行人，使用批处理）"""
        print("[全自动] ========== 开始清除障碍物 ==========")
        start_time = time.time()
        try:
            world = self.player.get_world()
            # 使用 args 创建客户端
            client = carla.Client(self.args.host, self.args.port)
            client.set_timeout(10.0)

            # 获取所有actor
            actors = world.get_actors()
            print(f"[全自动] 获取到 {len(actors)} 个actors")

            # 收集需要销毁的车辆和行人ID
            vehicle_ids = []
            walker_ids = []
            controller_ids = []

            for actor in actors:
                # 跳过主角
                if actor.id == self.player.id:
                    continue
                # 跳过传感器
                if actor.type_id.startswith("sensor."):
                    continue

                # 只销毁车辆和行人
                if actor.type_id.startswith("vehicle."):
                    vehicle_ids.append(actor.id)
                elif actor.type_id.startswith("walker."):
                    walker_ids.append(actor.id)
                elif actor.type_id == 'controller.ai.walker':
                    controller_ids.append(actor.id)

            print(f"[全自动] 分类完成: {len(vehicle_ids)} 辆车, {len(walker_ids)} 个行人, {len(controller_ids)} 个控制器")

            # 1. 先停止所有行人AI控制器
            print(f"[全自动] 正在停止 {len(controller_ids)} 个行人控制器...")
            controllers_stopped = 0
            for actor in actors:
                if actor.type_id == 'controller.ai.walker':
                    try:
                        actor.stop()
                        controllers_stopped += 1
                    except:
                        pass
            if controllers_stopped > 0:
                print(f"[全自动] 已停止 {controllers_stopped} 个行人控制器")
            else:
                print(f"[全自动] 无需停止行人控制器")

            # 2. 使用批处理销毁车辆
            if vehicle_ids:
                print(f"[全自动] 正在批量销毁 {len(vehicle_ids)} 辆车...")
                # 使用 carla.command 批量销毁
                commands = []
                for vid in vehicle_ids:
                    commands.append(carla.command.DestroyActor(vid))
                # 一次性发送所有销毁命令
                try:
                    client.apply_batch(commands)
                    print(f"[全自动] ✓ 已发送 {len(vehicle_ids)} 个车辆销毁命令")
                except Exception as e:
                    print(f"[全自动] ✗ 批量销毁车辆失败: {e}")
            else:
                print(f"[全自动] 无需销毁车辆")

            # 3. 使用批处理销毁行人
            if walker_ids:
                print(f"[全自动] 正在批量销毁 {len(walker_ids)} 个行人...")
                commands = []
                for wid in walker_ids:
                    commands.append(carla.command.DestroyActor(wid))
                try:
                    client.apply_batch(commands)
                    print(f"[全自动] ✓ 已发送 {len(walker_ids)} 个行人销毁命令")
                except Exception as e:
                    print(f"[全自动] ✗ 批量销毁行人失败: {e}")
            else:
                print(f"[全自动] 无需销毁行人")

            elapsed = time.time() - start_time
            print(f"[全自动] ========== 清除障碍物完成 (耗时 {elapsed:.1f}秒) ==========")
        except Exception as e:
            print(f"[全自动] ✗ 清除障碍物失败: {e}")
            import traceback
            traceback.print_exc()

    def _apply_emergency_brake(self):
        """应用紧急刹车并关闭自动驾驶"""
        try:
            self.player.set_autopilot(False)
            self.player.set_simulate_physics(False)
            self.player.set_target_velocity(carla.Vector3D(0, 0, 0))
            self.player.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
            # 在同步模式下不使用 sleep，避免超时
            control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, hand_brake=True)
            self.player.apply_control(control)
            print("[全自动] 车辆已刹停")
        except Exception as e:
            print(f"[全自动] 刹车失败: {e}")

    def _release_vehicle_control(self):
        """释放车辆控制权，允许用户手动操控"""
        try:
            # 恢复物理模拟
            self.player.set_simulate_physics(True)
            # 应用空控制（不刹车、不油门），让物理引擎接管
            control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0, hand_brake=False)
            self.player.apply_control(control)
            # 更新 KeyboardControl 状态，允许手动控制
            if self.keyboard_control:
                self.keyboard_control._autopilot_enabled = False
            print("[全自动] 车辆控制已释放，用户可手动操控")
        except Exception as e:
            print(f"[全自动] 释放车辆控制失败: {e}")

    # ===========================================================================
    # -- 扫描已有拍摄数据并自动复拍功能 -----------------------------------------
    # ===========================================================================

    def scan_existing_captures(self):
        """扫描panorama_output文件夹，获取已有的拍摄位置信息
        
        Returns:
            list: 包含位置信息的字典列表，每个字典包含folder, location, rotation
        """
        capture_locations = []
        
        # 确保输出目录存在
        if not os.path.exists(self.output_folder):
            print(f"[扫描] 输出目录不存在: {self.output_folder}")
            return capture_locations
        
        # 获取所有capture文件夹
        capture_folders = []
        for item in os.listdir(self.output_folder):
            if item.startswith('capture_'):
                folder_path = os.path.join(self.output_folder, item)
                if os.path.isdir(folder_path):
                    # 检查是否有location.txt
                    location_file = os.path.join(folder_path, 'location.txt')
                    if os.path.exists(location_file):
                        # 提取编号用于排序
                        number = 0
                        parts = item.split('_')
                        if len(parts) >= 2:
                            try:
                                number = int(parts[1])
                            except:
                                pass
                        capture_folders.append((folder_path, item, number))
        
        # 按编号排序（确保顺序正确）
        capture_folders.sort(key=lambda x: x[2])
        
        # 解析每个location.txt
        for folder_path, folder_name, number in capture_folders:
            try:
                location_info = {'folder': folder_path, 'folder_name': folder_name}
                
                # 读取location.txt
                with open(os.path.join(folder_path, 'location.txt'), 'r') as f:
                    for line in f:
                        line = line.strip()
                        if '=' in line:
                            key, value = line.split('=', 1)
                            if key == 'X':
                                if 'location' not in location_info:
                                    location_info['location'] = {}
                                location_info['location']['x'] = float(value)
                            elif key == 'Y':
                                if 'location' not in location_info:
                                    location_info['location'] = {}
                                location_info['location']['y'] = float(value)
                            elif key == 'Z':
                                if 'location' not in location_info:
                                    location_info['location'] = {}
                                location_info['location']['z'] = float(value)
                            elif key == 'Pitch':
                                if 'rotation' not in location_info:
                                    location_info['rotation'] = {}
                                location_info['rotation']['pitch'] = float(value)
                            elif key == 'Yaw':
                                if 'rotation' not in location_info:
                                    location_info['rotation'] = {}
                                location_info['rotation']['yaw'] = float(value)
                            elif key == 'Roll':
                                if 'rotation' not in location_info:
                                    location_info['rotation'] = {}
                                location_info['rotation']['roll'] = float(value)
                            elif key == 'GNSS_Lat':
                                location_info['gnss_lat'] = float(value)
                            elif key == 'GNSS_Lon':
                                location_info['gnss_lon'] = float(value)
                            elif key == 'GNSS_Alt':
                                location_info['gnss_alt'] = float(value)
                
                # 设置rgb_folder, semseg_folder, depth_folder
                location_info['rgb_folder'] = os.path.join(folder_path, 'rgb')
                location_info['semseg_folder'] = os.path.join(folder_path, 'semseg')
                location_info['depth_folder'] = os.path.join(folder_path, 'depth')
                
                # 检查是否已拼接过
                panorama_dir = os.path.join(folder_path, 'panorama')
                if os.path.exists(os.path.join(panorama_dir, 'panorama_rgb.png')):
                    location_info['already_stitched'] = True
                else:
                    location_info['already_stitched'] = False
                
                if 'location' in location_info and 'rotation' in location_info:
                    capture_locations.append(location_info)
                    status = "已拼接" if location_info['already_stitched'] else "未拼接"
                    print(f"[扫描] {folder_name}: ({location_info['location']['x']:.1f}, {location_info['location']['y']:.1f}) [{status}]")
            except Exception as e:
                print(f"[扫描] 解析 {folder_name} 失败: {e}")
        
        return capture_locations

    def auto_retake_from_existing(self):
        """从已有的panorama_output文件夹自动复拍并拼接
        
        功能流程:
        1. 扫描已有的拍摄位置
        2. 清除所有障碍物
        3. 找出所有未拼接的位置
        4. 创建专门的复拍文件夹
        5. 依次移动到每个未拼接位置复拍
        6. 复拍后自动拼接
        """
        print("\n" + "=" * 60)
        print("[自动复拍] 开始扫描已有的拍摄数据...")
        print("=" * 60)
        
        # 1. 扫描已有拍摄
        all_captures = self.scan_existing_captures()
        if not all_captures:
            print("[自动复拍] 未找到任何已有的拍摄数据")
            self.hud.notification("未找到已有拍摄数据", seconds=3.0)
            return
        
        # 2. 找出所有未拼接的位置
        pending_captures = [cap for cap in all_captures if not cap.get('already_stitched', False)]
        
        if not pending_captures:
            print("[自动复拍] 所有位置都已拼接完成，无需复拍")
            self.hud.notification("所有位置都已拼接完成", seconds=3.0)
            return
        
        # 3. 找出所有capture_XX编号的最大值
        max_number = 0
        for cap in all_captures:
            folder_name = cap['folder_name']
            parts = folder_name.split('_')
            if len(parts) >= 2:
                try:
                    num = int(parts[1])
                    if num > max_number:
                        max_number = num
                except:
                    pass
        
        # 4. 计算复拍的起始编号
        self._retake_next_number = max_number + 1
        print(f"[自动复拍] 最大拍摄编号: {max_number}, 复拍将从 #{self._retake_next_number} 开始")
        
        # 5. 创建专门的复拍文件夹
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        retake_base_folder = os.path.join(self.output_folder, f"retake_{timestamp}")
        self._retake_base_folder = retake_base_folder
        self._retake_index = 0
        
        # 初始化复拍文件夹列表
        self._retake_folders = []  # 存放新生成的复拍文件夹路径
        
        print(f"[自动复拍] 创建复拍文件夹: {os.path.basename(retake_base_folder)}")
        print(f"[自动复拍] 待复拍: {len(pending_captures)} 个位置")
        
        # 6. 初始化全景相机（如果还没初始化）
        if not self.panorama_sensors:
            self.setup_panorama_cameras()
        
        # 7. 清除障碍物
        print("[自动复拍] 清除障碍物...")
        self._clear_obstacles()
        
        # 8. 设置复拍状态
        self._in_retake_phase = True
        self._retake_locations = pending_captures
        
        # 9. 刹停车辆
        self._apply_emergency_brake()
        
        # 10. 开始复拍流程
        print(f"[自动复拍] 开始复拍 {len(pending_captures)} 个位置...")
        self.hud.notification(f"自动复拍: {len(pending_captures)} 个位置", seconds=3.0)
        self._do_next_retake_from_existing()

    def _do_next_retake_from_existing(self):
        """执行下一个从现有数据的复拍任务"""
        if self._retake_index >= len(self._retake_locations):
            # 所有复拍完成，开始全景拼接
            print("\n[自动复拍] 所有复拍完成，开始拼接...")
            self.hud.notification("复拍完成，开始拼接...", seconds=3.0)
            
            # 拼接新生成的复拍文件夹
            if hasattr(self, '_retake_folders') and self._retake_folders:
                print(f"[自动复拍] 拼接 {len(self._retake_folders)} 个复拍文件夹...")
                for folder_path in self._retake_folders:
                    self._stitch_single_folder(folder_path)
            
            # 扫描并拼接所有未拼接的历史数据（包括原始的capture_xx_xxxxxxx）
            print("\n[自动复拍] 扫描所有历史数据...")
            all_captures = self.scan_existing_captures()
            unstitched_count = 0
            
            for cap in all_captures:
                folder_path = cap.get('folder')
                if folder_path and not cap.get('already_stitched', False):
                    # 检查是否已经在_retake_folders中（避免重复拼接）
                    already_stitched = False
                    folder_path_norm = os.path.normpath(folder_path)
                    for rf in self._retake_folders:
                        if os.path.normpath(rf) == folder_path_norm:
                            already_stitched = True
                            break
                    
                    if not already_stitched:
                        print(f"[自动复拍] 拼接历史数据: {os.path.basename(folder_path)}")
                        self._stitch_single_folder(folder_path)
                        unstitched_count += 1
            
            print(f"\n[自动复拍] 复拍完成！")
            print(f"[自动复拍] 新复拍: {len(self._retake_folders)} 个位置")
            print(f"[自动复拍] 历史拼接: {unstitched_count} 个位置")

            self._in_retake_phase = False
            self.player.set_autopilot(False)  # 关闭自动驾驶
            self._release_vehicle_control()   # 恢复用户手动控制
            print("\n[自动复拍] 全部完成！车辆已停止。现在可以手动控制车辆。")
            self.hud.notification("自动复拍完成！", seconds=5.0)
            return
        
        loc_info = self._retake_locations[self._retake_index]
        loc = loc_info['location']
        rot = loc_info.get('rotation', {'pitch': 0, 'yaw': 0, 'roll': 0})
        
        print(f"\n[自动复拍] [{self._retake_index + 1}/{len(self._retake_locations)}] ({loc['x']:.1f}, {loc['y']:.1f})")
        
        # 移动车辆到位置
        carla_transform = carla.Transform(
            carla.Location(x=loc['x'], y=loc['y'], z=loc.get('z', 0)),
            carla.Rotation(pitch=rot.get('pitch', 0), yaw=rot.get('yaw', 0), roll=rot.get('roll', 0))
        )
        self.player.set_transform(carla_transform)
        print(f"[自动复拍] 已移动到位置: ({loc['x']:.1f}, {loc['y']:.1f})")
        
        # 延迟后开始拍摄
        threading.Timer(1.0, lambda: self._execute_retake_from_existing(loc_info)).start()

    def _execute_retake_from_existing(self, loc_info):
        """执行从现有数据的复拍摄影"""
        loc = loc_info['location']
        rot = loc_info.get('rotation', {'pitch': 0, 'yaw': 0, 'roll': 0})
        print(f"[自动复拍] 开始拍摄: ({loc['x']:.1f}, {loc['y']:.1f})")
        
        # 使用累加编号命名
        capture_number = self._retake_next_number
        self._retake_next_number += 1
        
        # 创建capture_XX文件夹（与原始拍摄格式一致）
        folder_name = f"capture_{capture_number}"
        folder_path = os.path.join(self.output_folder, folder_name)
        os.makedirs(folder_path, exist_ok=True)
        
        # 创建子文件夹
        rgb_folder = os.path.join(folder_path, 'rgb')
        semseg_folder = os.path.join(folder_path, 'semseg')
        depth_folder = os.path.join(folder_path, 'depth')
        os.makedirs(rgb_folder, exist_ok=True)
        os.makedirs(semseg_folder, exist_ok=True)
        os.makedirs(depth_folder, exist_ok=True)
        
        # 先清空旧数据，确保收到新数据
        self.panorama_images = {k: None for k in self.panorama_images}
        self.semseg_images = {k: None for k in self.semseg_images}
        self.depth_raw_images = {k: None for k in self.depth_raw_images}
        
        # 触发全景拍摄
        self.capture_panorama(disable_autopilot=False)
        
        # 等待直到收到所有6个方向的图像
        # 注意：保持 is_capturing = True，让语义分割回调能正常执行
        print(f"[自动复拍] 等待图像...")
        wait_count = 0
        max_wait = 50  # 最多等待5秒
        while wait_count < max_wait:
            rgb_ok = all(self.panorama_images.get(d) is not None for d in ['front', 'back', 'left', 'right', 'top', 'bottom'])
            semseg_ok = all(self.semseg_images.get(d) is not None for d in ['front', 'back', 'left', 'right', 'top', 'bottom'])
            depth_ok = all(self.depth_raw_images.get(d) is not None for d in ['front', 'back', 'left', 'right', 'top', 'bottom'])
            
            if rgb_ok and semseg_ok and depth_ok:
                break
            
            time.sleep(0.1)
            wait_count += 1
        
        # 停止拍摄状态，防止自动保存
        self.is_capturing = False
        
        if not rgb_ok:
            print(f"[自动复拍] 警告: RGB图像不完整")
        
        # 保存6个方向的图像
        saved_count = 0
        saved_semseg_count = 0
        for direction in ['front', 'back', 'left', 'right', 'top', 'bottom']:
            if self.panorama_images.get(direction) is not None:
                cv2.imwrite(
                    os.path.join(rgb_folder, f'{direction}.png'),
                    cv2.cvtColor(self.panorama_images[direction], cv2.COLOR_RGB2BGR)
                )
                saved_count += 1
            # 语义分割图像是RGB格式，直接保存（不要转换）
            if self.semseg_images.get(direction) is not None:
                cv2.imwrite(
                    os.path.join(semseg_folder, f'{direction}.png'),
                    self.semseg_images[direction]
                )
                saved_semseg_count += 1
            if self.depth_raw_images.get(direction) is not None:
                cv2.imwrite(
                    os.path.join(depth_folder, f'{direction}.png'),
                    cv2.cvtColor(self.depth_raw_images[direction], cv2.COLOR_RGB2BGR)
                )
        
        print(f"[自动复拍] 保存了 {saved_count}/6 RGB, {saved_semseg_count}/6 Semseg 图像")
        
        # 保存位置信息到文件
        with open(os.path.join(folder_path, 'location.txt'), 'w') as f:
            f.write(f"X={loc['x']}\nY={loc['y']}\nZ={loc.get('z', 0)}\n")
            f.write(f"Pitch={rot.get('pitch', 0)}\nYaw={rot.get('yaw', 0)}\nRoll={rot.get('roll', 0)}\n")
            # 添加GNSS数据
            if 'gnss_lat' in loc_info:
                f.write(f"GNSS_Lat={loc_info['gnss_lat']:.8f}\n")
            if 'gnss_lon' in loc_info:
                f.write(f"GNSS_Lon={loc_info['gnss_lon']:.8f}\n")
            if 'gnss_alt' in loc_info:
                f.write(f"GNSS_Alt={loc_info['gnss_alt']:.4f}\n")
        
        # 记录新生成的复拍文件夹
        self._retake_folders.append(folder_path)
        print(f"[自动复拍] 已保存: {folder_name}")
        
        self._retake_index += 1
        # 延迟开始下一个复拍
        threading.Timer(0.5, self._do_next_retake_from_existing).start()

    def _stitch_single_folder(self, folder_path):
        """拼接单个文件夹的全景图"""
        folder_name = os.path.basename(folder_path)
        
        # 检查是否已拼接过
        panorama_dir = os.path.join(folder_path, 'panorama')
        if os.path.exists(os.path.join(panorama_dir, 'panorama_rgb.png')):
            print(f"[全景] {folder_name}: 已拼接过，跳过")
            return False
        
        print(f"[全景] 正在拼接: {folder_name}")
        
        # 读取6个方向的图像
        images = {}
        semseg_images = {}
        depth_images = {}
        
        rgb_folder = os.path.join(folder_path, 'rgb')
        semseg_folder = os.path.join(folder_path, 'semseg')
        depth_folder = os.path.join(folder_path, 'depth')
        
        for direction in ['front', 'back', 'left', 'right', 'top', 'bottom']:
            rgb_path = os.path.join(rgb_folder, f'{direction}.png')
            semseg_path = os.path.join(semseg_folder, f'{direction}.png')
            depth_path = os.path.join(depth_folder, f'{direction}.png')
            
            if os.path.exists(rgb_path):
                images[direction] = cv2.imread(rgb_path)
            if os.path.exists(semseg_path):
                semseg_images[direction] = cv2.imread(semseg_path)
            if os.path.exists(depth_path):
                depth_images[direction] = cv2.imread(depth_path)
        
        # 创建输出目录
        os.makedirs(panorama_dir, exist_ok=True)
        
        # 并行拼接
        if images:
            def save_rgb_panorama():
                rgb_panorama = self._stitch_panorama(images, 'nearest')
                if rgb_panorama is not None:
                    cv2.imwrite(os.path.join(panorama_dir, 'panorama_rgb.png'), rgb_panorama)
                    print(f"[全景] 已保存: {panorama_dir}/panorama_rgb.png")
            
            def save_semseg_panorama():
                semseg_panorama = self._stitch_panorama(semseg_images, 'nearest')
                if semseg_panorama is not None:
                    cv2.imwrite(os.path.join(panorama_dir, 'panorama_semseg.png'), semseg_panorama)
                    print(f"[全景] 已保存: {panorama_dir}/panorama_semseg.png")
            
            def save_depth_panorama():
                depth_panorama = self._stitch_panorama(depth_images, 'linear')
                if depth_panorama is not None:
                    cv2.imwrite(os.path.join(panorama_dir, 'panorama_depth.png'), depth_panorama)
                    print(f"[全景] 已保存: {panorama_dir}/panorama_depth.png")
            
            from concurrent.futures import ThreadPoolExecutor
            with ThreadPoolExecutor(max_workers=3) as executor:
                executor.submit(save_rgb_panorama)
                executor.submit(save_semseg_panorama)
                executor.submit(save_depth_panorama)
            
            return True
        else:
            print(f"[全景] {folder_name}: 缺少图像，跳过")
            return False

    def batch_stitch_remaining(self):
        """批量拼接所有未拼接的透视图为全景"""
        if not hasattr(self, '_retake_locations') or not self._retake_locations:
            print("[全景] 没有待拼接的数据")
            return
        
        from concurrent.futures import ThreadPoolExecutor
        
        stitched_count = 0
        skipped_count = 0
        
        for loc_info in self._retake_locations:
            folder_path = loc_info['folder']
            folder_name = os.path.basename(folder_path)
            
            # 检查是否已拼接过
            panorama_dir = os.path.join(folder_path, 'panorama')
            if os.path.exists(os.path.join(panorama_dir, 'panorama_rgb.png')):
                print(f"[全景] {folder_name}: 已拼接过，跳过")
                skipped_count += 1
                continue
            
            print(f"[全景] 正在拼接: {folder_name}")
            
            # 读取6个方向的图像（从子文件夹读取）
            images = {}
            semseg_images = {}
            depth_images = {}
            
            rgb_folder = loc_info.get('rgb_folder', os.path.join(folder_path, 'rgb'))
            semseg_folder = loc_info.get('semseg_folder', os.path.join(folder_path, 'semseg'))
            depth_folder = loc_info.get('depth_folder', os.path.join(folder_path, 'depth'))
            
            for direction in ['front', 'back', 'left', 'right', 'top', 'bottom']:
                rgb_path = os.path.join(rgb_folder, f'{direction}.png')
                semseg_path = os.path.join(semseg_folder, f'{direction}.png')
                depth_path = os.path.join(depth_folder, f'{direction}.png')
                
                if os.path.exists(rgb_path):
                    images[direction] = cv2.imread(rgb_path)
                if os.path.exists(semseg_path):
                    semseg_images[direction] = cv2.imread(semseg_path)
                if os.path.exists(depth_path):
                    depth_images[direction] = cv2.imread(depth_path)
            
            # 创建输出目录
            os.makedirs(panorama_dir, exist_ok=True)
            
            # 并行拼接
            if images:
                def save_rgb_panorama():
                    rgb_panorama = self._stitch_panorama(images, 'nearest')
                    if rgb_panorama is not None:
                        cv2.imwrite(os.path.join(panorama_dir, 'panorama_rgb.png'), rgb_panorama)
                        print(f"[全景] 已保存: {panorama_dir}/panorama_rgb.png")
                
                def save_semseg_panorama():
                    semseg_panorama = self._stitch_panorama(semseg_images, 'nearest')
                    if semseg_panorama is not None:
                        cv2.imwrite(os.path.join(panorama_dir, 'panorama_semseg.png'), semseg_panorama)
                        print(f"[全景] 已保存: {panorama_dir}/panorama_semseg.png")
                
                def save_depth_panorama():
                    depth_panorama = self._stitch_panorama(depth_images, 'linear')
                    if depth_panorama is not None:
                        cv2.imwrite(os.path.join(panorama_dir, 'panorama_depth.png'), depth_panorama)
                        print(f"[全景] 已保存: {panorama_dir}/panorama_depth.png")
                
                with ThreadPoolExecutor(max_workers=3) as executor:
                    executor.submit(save_rgb_panorama)
                    executor.submit(save_semseg_panorama)
                    executor.submit(save_depth_panorama)
                
                stitched_count += 1
            else:
                print(f"[全景] {folder_name}: 缺少图像，跳过")
        
        print(f"\n[全景] 拼接完成: 新拼接 {stitched_count} 个, 跳过 {skipped_count} 个")

    def batch_stitch_all(self):
        """批量拼接所有保存的透视图为全景"""
        if not hasattr(self, '_capture_locations') or not self._capture_locations:
            print("[全景] 没有待拼接的数据")
            return
        
        from concurrent.futures import ThreadPoolExecutor
        
        for loc_info in self._capture_locations:
            folder_path = loc_info['folder']
            print(f"[全景] 正在拼接: {os.path.basename(folder_path)}")
            
            # 读取6个方向的图像（从子文件夹读取）
            images = {}
            semseg_images = {}
            depth_images = {}
            
            rgb_folder = loc_info.get('rgb_folder', os.path.join(folder_path, 'rgb'))
            semseg_folder = loc_info.get('semseg_folder', os.path.join(folder_path, 'semseg'))
            depth_folder = loc_info.get('depth_folder', os.path.join(folder_path, 'depth'))
            
            for direction in ['front', 'back', 'left', 'right', 'top', 'bottom']:
                rgb_path = os.path.join(rgb_folder, f'{direction}.png')
                semseg_path = os.path.join(semseg_folder, f'{direction}.png')
                depth_path = os.path.join(depth_folder, f'{direction}.png')
                
                if os.path.exists(rgb_path):
                    images[direction] = cv2.imread(rgb_path)
                if os.path.exists(semseg_path):
                    semseg_images[direction] = cv2.imread(semseg_path)
                if os.path.exists(depth_path):
                    depth_images[direction] = cv2.imread(depth_path)
            
            # 创建输出目录
            panorama_dir = os.path.join(folder_path, 'panorama')
            os.makedirs(panorama_dir, exist_ok=True)
            
            # 并行拼接
            if images:
                def save_rgb_panorama():
                    rgb_panorama = self._stitch_panorama(images, 'nearest')
                    if rgb_panorama is not None:
                        cv2.imwrite(os.path.join(panorama_dir, 'panorama_rgb.png'), rgb_panorama)
                        print(f"[全景] 已保存: {panorama_dir}/panorama_rgb.png")
                
                def save_semseg_panorama():
                    semseg_panorama = self._stitch_panorama(semseg_images, 'nearest')
                    if semseg_panorama is not None:
                        cv2.imwrite(os.path.join(panorama_dir, 'panorama_semseg.png'), semseg_panorama)
                        print(f"[全景] 已保存: {panorama_dir}/panorama_semseg.png")
                
                def save_depth_panorama():
                    depth_panorama = self._stitch_panorama(depth_images, 'linear')
                    if depth_panorama is not None:
                        cv2.imwrite(os.path.join(panorama_dir, 'panorama_depth.png'), depth_panorama)
                        print(f"[全景] 已保存: {panorama_dir}/panorama_depth.png")
                
                with ThreadPoolExecutor(max_workers=3) as executor:
                    executor.submit(save_rgb_panorama)
                    executor.submit(save_semseg_panorama)
                    executor.submit(save_depth_panorama)
            
            print(f"[全景] 已拼接: {os.path.basename(folder_path)}")

    def _stitch_panorama(self, images, interpolation='nearest'):
        """
        从6张图像拼接全景图（使用等距长方投影）
        
        Args:
            images: 6个方向的图像字典
            interpolation: 插值方法，'nearest'用于标签，'linear'用于深度图
        
        Args:
            images: 6个方向的图像字典
            interpolation: 插值方法
        """
        if not all(d in images for d in ['front', 'back', 'left', 'right', 'top', 'bottom']):
            print("[全景] 缺少方向图像，跳过拼接")
            return None
        
        # 转换为RGB格式
        for key in images:
            if images[key] is not None and len(images[key].shape) == 3:
                images[key] = cv2.cvtColor(images[key], cv2.COLOR_BGR2RGB)
        
        # 创建dice格式立方体贴图
        cube_size = images['front'].shape[0]
        cube_w = cube_size * 4
        cube_h = cube_size * 3
        
        cube_map = np.zeros((cube_h, cube_w, 3), dtype=np.uint8)
        
        # 放置6个面到dice格式
        #      top
        # left front right back
        #     bottom
        
        # front (中心)
        cube_map[cube_size:2*cube_size, cube_size:2*cube_size] = images['front']
        # right (右边第二格)
        cube_map[cube_size:2*cube_size, 2*cube_size:3*cube_size] = images['right']
        # back (右边)
        cube_map[cube_size:2*cube_size, 3*cube_size:4*cube_size] = images['back']
        # left (左边)
        cube_map[cube_size:2*cube_size, 0:cube_size] = images['left']
        # top (上边)
        cube_map[0:cube_size, cube_size:2*cube_size] = images['top']
        # bottom (下边)
        cube_map[2*cube_size:3*cube_size, cube_size:2*cube_size] = images['bottom']
        
        # 使用py360convert转换为全景图
        try:
            if HAS_PY360CONVERT:
                # py360convert.c2e(cubemap, h, w, mode='bilinear', cube_format='dice')
                # h = 输出高度, w = 输出宽度
                h = cube_size * 2  # 全景图高度 = cube_size * 2
                w = cube_size * 4  # 全景图宽度 = cube_size * 4
                equirectangular = py360convert.c2e(cube_map, h, w, mode='bilinear')
            else:
                # 简单拼接方法（水平展开）
                equirectangular = self._simple_concat(cube_map, cube_size)
        except Exception as e:
            print(f"[全景] py360convert转换失败: {e}")
            import traceback
            traceback.print_exc()
            equirectangular = self._simple_concat(cube_map, cube_size)
        
        return cv2.cvtColor(equirectangular, cv2.COLOR_RGB2BGR) if len(equirectangular.shape) == 3 else equirectangular

    def _simple_concat(self, cube_map, cube_size):
        """简单拼接方法 - 水平展开所有方向"""
        # 重新排列为水平环绕格式
        order = ['left', 'front', 'right', 'back']
        h, w = cube_map.shape[:2]
        result = np.zeros((cube_size, cube_size * 4, 3), dtype=np.uint8)
        
        for i, name in enumerate(order):
            start_col = i * cube_size
            if name in ['left', 'front', 'right', 'back']:
                # 从cube_map提取正确位置
                if name == 'front':
                    result[:, start_col:start_col+cube_size] = cube_map[cube_size:2*cube_size, cube_size:2*cube_size]
                elif name == 'back':
                    result[:, start_col:start_col+cube_size] = cube_map[cube_size:2*cube_size, 2*cube_size:3*cube_size]
                elif name == 'left':
                    result[:, start_col:start_col+cube_size] = cube_map[cube_size:2*cube_size, 0:cube_size]
                elif name == 'right':
                    result[:, start_col:start_col+cube_size] = cube_map[cube_size:2*cube_size, 3*cube_size:4*cube_size]
        
        return result


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        self._ackermann_enabled = False
        self._ackermann_reverse = 1
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._ackermann_control = carla.VehicleAckermannControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self, client, world, clock, sync_mode):
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    if self._autopilot_enabled:
                        world.player.set_autopilot(False)
                        world.restart()
                        world.player.set_autopilot(True)
                    else:
                        world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_v and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_map_layer(reverse=True)
                elif event.key == K_v:
                    world.next_map_layer()
                elif event.key == K_b and pygame.key.get_mods() & KMOD_SHIFT:
                    world.load_map_layer(unload=True)
                elif event.key == K_b:
                    world.load_map_layer()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_g:
                    world.toggle_radar()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key == K_n:
                    world.camera_manager.next_sensor()
                elif event.key == K_w and (pygame.key.get_mods() & KMOD_CTRL):
                    if world.constant_velocity_enabled:
                        world.player.disable_constant_velocity()
                        world.constant_velocity_enabled = False
                        world.hud.notification("Disabled Constant Velocity Mode")
                    else:
                        world.player.enable_constant_velocity(carla.Vector3D(17, 0, 0))
                        world.constant_velocity_enabled = True
                        world.hud.notification("Enabled Constant Velocity Mode at 60 km/h")
                elif event.key == K_o:
                    try:
                        if world.doors_are_open:
                            world.hud.notification("Closing Doors")
                            world.doors_are_open = False
                            world.player.close_door(carla.VehicleDoor.All)
                        else:
                            world.hud.notification("Opening doors")
                            world.doors_are_open = True
                            world.player.open_door(carla.VehicleDoor.All)
                    except Exception:
                        pass
                elif event.key == K_t:
                    if world.show_vehicle_telemetry:
                        world.player.show_debug_telemetry(False)
                        world.show_vehicle_telemetry = False
                        world.hud.notification("Disabled Vehicle Telemetry")
                    else:
                        try:
                            world.player.show_debug_telemetry(True)
                            world.show_vehicle_telemetry = True
                            world.hud.notification("Enabled Vehicle Telemetry")
                        except Exception:
                            pass
                elif event.key > K_0 and event.key <= K_9:
                    index_ctrl = 0
                    if pygame.key.get_mods() & KMOD_CTRL:
                        index_ctrl = 9
                    world.camera_manager.set_sensor(event.key - 1 - K_0 + index_ctrl)
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.camera_manager.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    if (world.recording_enabled):
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    current_index = world.camera_manager.index
                    world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    world.camera_manager.set_sensor(current_index)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_f:
                        # Toggle ackermann controller
                        self._ackermann_enabled = not self._ackermann_enabled
                        world.hud.show_ackermann_info(self._ackermann_enabled)
                        world.hud.notification("Ackermann Controller %s" %
                                               ("Enabled" if self._ackermann_enabled else "Disabled"))
                    if event.key == K_q:
                        if not self._ackermann_enabled:
                            self._control.gear = 1 if self._control.reverse else -1
                        else:
                            self._ackermann_reverse *= -1
                            # Reset ackermann control
                            self._ackermann_control = carla.VehicleAckermannControl()
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                        if not self._autopilot_enabled and not sync_mode:
                            print("WARNING: You are currently in asynchronous mode and could "
                                  "experience some issues with the traffic simulation")
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification(
                            'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                        current_lights ^= carla.VehicleLightState.Special1
                    elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                        current_lights ^= carla.VehicleLightState.HighBeam
                    elif event.key == K_l:
                        # Use 'L' key to switch between lights:
                        # closed -> position -> low beam -> fog
                        if not self._lights & carla.VehicleLightState.Position:
                            world.hud.notification("Position lights")
                            current_lights |= carla.VehicleLightState.Position
                        else:
                            world.hud.notification("Low beam lights")
                            current_lights |= carla.VehicleLightState.LowBeam
                        if self._lights & carla.VehicleLightState.LowBeam:
                            world.hud.notification("Fog lights")
                            current_lights |= carla.VehicleLightState.Fog
                        if self._lights & carla.VehicleLightState.Fog:
                            world.hud.notification("Lights off")
                            current_lights ^= carla.VehicleLightState.Position
                            current_lights ^= carla.VehicleLightState.LowBeam
                            current_lights ^= carla.VehicleLightState.Fog
                    elif event.key == K_i:
                        current_lights ^= carla.VehicleLightState.Interior
                    elif event.key == K_z:
                        current_lights ^= carla.VehicleLightState.LeftBlinker
                    elif event.key == K_x:
                        current_lights ^= carla.VehicleLightState.RightBlinker
                    elif event.key == K_y:
                        # Y键: 自动拍摄（开源简化版移除）
                        world.hud.notification("自动拍摄已移除，联系作者获取完整功能", seconds=3.0)
                    elif event.key == K_u:
                        # U键: 全自动流程（开源简化版移除）
                        world.hud.notification("全自动流程已移除，联系作者获取完整功能", seconds=3.0)
                    elif event.key == pygame.K_F12:
                        # F12键: 自动复拍与拼接（开源简化版移除）
                        world.hud.notification("自动复拍已移除，联系作者获取完整功能", seconds=3.0)
                    
                    elif event.key == pygame.K_e:
                        # E键: 提前结束自动拍摄（开源简化版移除）
                        world.hud.notification("复拍流程已移除，联系作者获取完整功能", seconds=3.0)

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._control.reverse = self._control.gear < 0
                # Set automatic control-related vehicle lights
                if self._control.brake:
                    current_lights |= carla.VehicleLightState.Brake
                else: # Remove the Brake flag
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    current_lights |= carla.VehicleLightState.Reverse
                else: # Remove the Reverse flag
                    current_lights &= ~carla.VehicleLightState.Reverse
                if current_lights != self._lights: # Change the light state only if necessary
                    self._lights = current_lights
                    world.player.set_light_state(carla.VehicleLightState(self._lights))
                # Apply control
                if not self._ackermann_enabled:
                    world.player.apply_control(self._control)
                else:
                    world.player.apply_ackermann_control(self._ackermann_control)
                    # Update control to the last one applied by the ackermann controller.
                    self._control = world.player.get_control()
                    # Update hud with the newest ackermann control
                    world.hud.update_ackermann_control(self._ackermann_control)

            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)
                world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[K_UP] or keys[K_w]:
            if not self._ackermann_enabled:
                self._control.throttle = min(self._control.throttle + 0.1, 1.00)
            else:
                self._ackermann_control.speed += round(milliseconds * 0.005, 2) * self._ackermann_reverse
        else:
            if not self._ackermann_enabled:
                self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            if not self._ackermann_enabled:
                self._control.brake = min(self._control.brake + 0.2, 1)
            else:
                self._ackermann_control.speed -= min(abs(self._ackermann_control.speed), round(milliseconds * 0.005, 2)) * self._ackermann_reverse
                self._ackermann_control.speed = max(0, abs(self._ackermann_control.speed)) * self._ackermann_reverse
        else:
            if not self._ackermann_enabled:
                self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        if not self._ackermann_enabled:
            self._control.steer = round(self._steer_cache, 1)
            self._control.hand_brake = keys[K_SPACE]
        else:
            self._ackermann_control.steer = round(self._steer_cache, 1)

    def _parse_walker_keys(self, keys, milliseconds, world):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = world.player_max_speed_fast if pygame.key.get_mods() & KMOD_SHIFT else world.player_max_speed
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

        self._show_ackermann_info = False
        self._ackermann_control = carla.VehicleAckermannControl()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            'Accelero: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.accelerometer),
            'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
            if self._show_ackermann_info:
                self._info_text += [
                    '',
                    'Ackermann Controller:',
                    '  Target speed: % 8.0f km/h' % (3.6*self._ackermann_control.speed),
                ]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def show_ackermann_info(self, enabled):
        self._show_ackermann_info = enabled

    def update_ackermann_control(self, ackermann_control):
        self._ackermann_control = ackermann_control

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            self.hud = hud
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0  # 添加高度
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude
        self.alt = event.altitude


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================


class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z

        self.velocity_range = 7.5 # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35))
        bp.set_attribute('vertical_fov', str(20))
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z+0.05),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))

    @staticmethod
    def _Radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
        # points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        # points = np.reshape(points, (len(radar_data), 4))

        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / self.velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        Attachment = carla.AttachmentType

        if not self._parent.type_id.startswith("walker.pedestrian"):
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=+0.8*bound_x, y=+0.0*bound_y, z=1.3*bound_z)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=+1.9*bound_x, y=+1.0*bound_y, z=1.2*bound_z)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-2.8*bound_x, y=+0.0*bound_y, z=4.6*bound_z), carla.Rotation(pitch=6.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-1.0, y=-1.0*bound_y, z=0.4*bound_z)), Attachment.Rigid)]
        else:
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=2.5, y=0.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-4.0, z=2.0), carla.Rotation(pitch=6.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=0, y=-2.5, z=-0.0), carla.Rotation(yaw=90.0)), Attachment.Rigid)]

        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.instance_segmentation', cc.CityScapesPalette, 'Camera Instance Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.instance_segmentation', cc.Raw, 'Camera Instance Segmentation (Raw)', {}],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {'range': '50'}],
            ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
                {'lens_circle_multiplier': '3.0',
                'lens_circle_falloff': '3.0',
                'chromatic_aberration_intensity': '0.5',
                'chromatic_aberration_offset': '0'}],
            ['sensor.camera.optical_flow', cc.Raw, 'Optical Flow', {}],
            ['sensor.camera.normals', cc.Raw, 'Camera Normals', {}],
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                self.lidar_range = 50

                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == 'range':
                        self.lidar_range = float(attr_value)

            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / (2.0 * self.lidar_range)
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        elif self.sensors[self.index][0].startswith('sensor.camera.optical_flow'):
            image = image.get_color_coded_flow()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    original_settings = None
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(20.0)
        # 获取当前地图的名称
        current_map = client.get_world().get_map().name

        if args.map in current_map:
            print(f"当前地图已是目标地图 {args.map}，无需重新加载。")
            sim_world = client.get_world()
        else:
            print(f"当前地图为 {current_map}，将加载目标地图 {args.map}...")
            sim_world = client.load_world(args.map)

        if args.sync: # 如果配置文件为同步模式。事实上不会在本程序这样设置，忽略
            original_settings = sim_world.get_settings()#获取原始设置
            settings = sim_world.get_settings()#获取当前设置
            if not settings.synchronous_mode:#如果当前不是同步模式
                settings.synchronous_mode = True#设置为同步模式
                settings.fixed_delta_seconds = 0.05  # 50 FPS (1/50=0.02)#debug
            sim_world.apply_settings(settings)#应用设置

            traffic_manager = client.get_trafficmanager()#获取交通管理器
            traffic_manager.set_synchronous_mode(True)#设置为同步模式

        display = pygame.display.set_mode(#设置显示窗口
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))#填充背景色    
        pygame.display.flip()#刷新显示

        hud = HUD(args.width, args.height)#创建HUD对象
        world = World(sim_world, hud, args)#创建World对象
        controller = KeyboardControl(world, args.autopilot)#创建KeyboardControl对象
        world.keyboard_control = controller  # 保存键盘控制对象引用


        clock = pygame.time.Clock()#创建时钟对象
        while True:
            current_sync_mode = sim_world.get_settings().synchronous_mode
            # clock.tick_busy_loop(60)
            clock.tick(60) # 该刷新方法不会导致同步模式延迟，这是游戏循环的核心，60表示60帧每秒
            
            # 在同步模式下，先处理待恢复的操作，再等待下一帧
            if world._pending_autopilot_restore and not world.is_capturing: #如果需要恢复自动驾驶且不在拍摄状态
                world._pending_autopilot_restore = False
                if world.auto_capture_mode: #如果处于自动拍摄模式
                    try:
                        # 恢复物理模拟
                        world.player.set_simulate_physics(True)
                        world.player.set_target_velocity(carla.Vector3D(0, 0, 0))
                        world.player.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
                        # 恢复自动驾驶
                        world.player.set_autopilot(True)
                        print("[全景] 车辆已恢复自动驾驶")
                    except Exception as e:
                        print(f"[全景] 恢复失败: {e}")
            
            if current_sync_mode == True: # 监听carla世界是否处于同步模式
                sim_world.wait_for_tick() #等待下一帧


            if controller.parse_events(client, world, clock, args.sync):#处理事件
                return

            world.tick(clock)#处理世界
            world.render(display)#渲染画面
            pygame.display.flip()

            # 自动拍摄模式: 检查行驶距离并触发拍摄
            if world.auto_capture_mode and not world.is_capturing:
                world.update_distance_traveled()
                if world.total_distance_since_capture >= world.auto_capture_interval:#如果行驶距离大于等于自动拍摄间隔
                    world.capture_panorama()#触发拍摄
                    world.total_distance_since_capture = 0.0
                    print(f"[自动拍摄] 已行驶 {world.auto_capture_interval:.0f}米, 触发拍摄")
            
            # 超时检测：如果5秒内没有完成拍摄，重置状态并重试
            if world.is_capturing and world._capture_start_time:#如果正在拍摄且拍摄开始时间不为空
                if time.time() - world._capture_start_time > 5.0:
                    world._capture_retry_count += 1
                    if world._capture_retry_count >= world._max_capture_retries:
                        # 超过最大重试次数，放弃这张
                        print(f"[全景] ⚠️ 拍摄失败，已重试 {world._max_capture_retries} 次，跳过")
                        world.is_capturing = False
                        world._capture_start_time = None
                        world._capture_retry_count = 0
                        # 设置标志，让下一次循环恢复自动驾驶
                        if world.auto_capture_mode:
                            world._pending_autopilot_restore = True
                        world.hud.notification('Capture failed, skipping...', seconds=2.0)
                    else:
                        # 重试拍摄
                        print(f"[全景] ⚠️ 拍摄超时，重试 {world._capture_retry_count}/{world._max_capture_retries}")
                        world.is_capturing = False
                        world.capture_panorama()

    finally:

        if original_settings:#
            sim_world.apply_settings(original_settings) #应用原始设置

        if (world and world.recording_enabled): #如果存在世界且记录状态为开启
            client.stop_recorder() #停止录制

        if world is not None: #如果存在世界
            world.destroy() #销毁世界

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        default='True',  # 默认为同步模式
        help='Activate synchronous mode execution')
    
    ## wzq自定义参数
    # 地图名字
    argparser.add_argument(
        '--map',
        default='gaoshu3800',
        help='choose map(default: Town06)')
    # 是否随机位置？
    argparser.add_argument(
        '--random_spawn',
        default='False',
        help='Random spawn setting,True is random(default:False)')
    # 出生位置（固定时）
    argparser.add_argument(
        '--spawn_point',
        # default='-280.7,245.6,0.05,0', #Town06
        default='-1930, 48.25, 0.3, 0', #gaoshu3800 起点
        help='spawn point(x,y,z,yaw)')
    # 车辆选型
    argparser.add_argument(
        '--filter',
        # metavar='PATTERN',
        default='vehicle.tesla.model3', # 注意，model3是单踏板模式，纵向动力学不一样
        # default='vehicle.dodge.charger_2020',
        # default='vehicle.*',
        help='Vehicle selection')
    # 车辆名字
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='ego_car',
        help='actor role name (default: "ego_car")')
    
    # 全景图输出目录
    argparser.add_argument(
        '--output',
        default='./panorama_output',
        help='Output folder for panorama images (default: ./panorama_output)')
    
    # 解析参数
    args = argparser.parse_args()
    # args参数处理，处理画面大小
    args.width, args.height = [int(x) for x in args.res.split('x')]
    # 处理生成位置
    spawn_coords = list(map(float, args.spawn_point.split(',')))
    if len(spawn_coords) != 4:
        raise ValueError("Spawn point must contain exactly 4 values (x, y, z, yaw)!")
    args.spawn_point = argparse.Namespace(
        x=spawn_coords[0],
        y=spawn_coords[1],
        z=spawn_coords[2],
        yaw=spawn_coords[3],
    )

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\n用户尝试退出中....')
    finally:
        # 清理所有车辆、传感器
        delete_all_actor(args)
        print('退出成功')


if __name__ == '__main__':

    main()
