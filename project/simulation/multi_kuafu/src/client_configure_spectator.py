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

import random

npc_list = []

try:
    my_client = carla.Client('localhost', 2000)
    my_client.set_timeout(30.0)
    world = my_client.get_world()

    while True:
        actor_list = world.get_actors()
        ego_vehicle = actor_list.filter("vehicle.kuafu.kuafu")
        if len(ego_vehicle):
            ego_vehicle = ego_vehicle[0]
            break
    print("Obtain the kuafu vehicle")

    # spawn npc
    if sys.argv[1] == '-npc':
        for _ in range(0, 10):
            transform_obs = ego_vehicle.get_transform()
            transform_obs.location.x += 5
            transform_obs.location.z += 5
            bp = random.choice(world.get_blueprint_library().filter('walker'))

            # This time we are using try_spawn_actor. If the spot is already
            # occupied by another object, the function will return None.
            npc = world.try_spawn_actor(bp, transform_obs)
            if npc is not None:
                npc_list.append(npc)
                # npc.set_autopilot(True)
                print('created %s' % npc.type_id) 

    while True:
        spectator = world.get_spectator()
        transform = ego_vehicle.get_transform()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20),
            carla.Rotation(pitch=-90)))
except KeyboardInterrupt: 
    print("destroying all npc")
    my_client.apply_batch([carla.command.DestroyActor(x) for x in npc_list])
