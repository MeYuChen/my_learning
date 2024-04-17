#!/usr/bin/env python
#
# Copyright (c) 2022, Unity-Drive Inc. All rights reserved.
# Author: Zhenyu Li lizhenyu@unity-drive.com


import glob
import os
import sys
try:
    sys.path.append(glob.glob('/home/udrive/carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import json

def parse_json_file(json_file_path):
    global_sensors = []
    vehicles = []
    with open(json_file_path, 'r', encoding='utf-8') as json_file:
        json_files = json.loads(json_file.read())

    for actor in json_files["objects"]:
        actor_type = actor["type"].split('.')[0]
        if actor_type == "sensor":
            global_sensors.append(actor)
        elif actor_type == "vehicle" or actor_type == "walker":
            vehicles.append(actor)
    return global_sensors, vehicles, json_files["config"]     

try:
    my_client = carla.Client('localhost', 2000)
    my_client.set_timeout(30.0)
    map_name = sys.argv[1]
    world = my_client.load_world(map_name)
    world.unload_map_layer(carla.MapLayer.Buildings)
    world.unload_map_layer(carla.MapLayer.Decals)
    world.unload_map_layer(carla.MapLayer.Foliage)
    # world.unload_map_layer(carla.MapLayer.Ground)
    world.unload_map_layer(carla.MapLayer.ParkedVehicles)
    world.unload_map_layer(carla.MapLayer.Particles)
    # world.unload_map_layer(carla.MapLayer.Props)
    world.unload_map_layer(carla.MapLayer.StreetLights)
    # world.unload_map_layer(carla.MapLayer.Walls)
    print("Unload useless feature of map")
except KeyboardInterrupt: 
    pass
