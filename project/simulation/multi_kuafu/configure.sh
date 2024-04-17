MAP_NAME=dxy
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.12-py2.7-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla
if [ $1 ] 
then
  MAP_NAME=$1
fi
echo "Open the map $MAP_NAME"
sudo rm /var/zone/route/*
sudo cp ./waypoint_map/$MAP_NAME/* /var/zone/route/
python src/client_configure_map.py $MAP_NAME
echo "Configuring carla environment finished !"
