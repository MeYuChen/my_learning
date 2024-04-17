import carla
client = carla.Client("127.0.0.1",2000)
world =  client.get_world()
actors = world.get_actors()
while True :
    snapshot = world.wait_for_tick()
    actors =  world.get_actors()
    for actor in actors:
        size = actor.bounding_box.extent
        print(" box size",size.x,size.y,size.z,actor.type_id)

print(world)