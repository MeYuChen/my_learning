CURRENT_DIR=$(cd `dirname $0`; pwd)
WORKSPACE=$(cd $CURRENT_DIR/../../..; pwd)
PACKAGE_DIR=$(cd $CURRENT_DIR/..; pwd)
PACKAGE_NAME=${PACKAGE_DIR##*/}
ROS_VERSION=$(rosversion  -d)

FAKE_ROOT_DIR=$WORKSPACE/fakeroot
INSTALL_DIR=$WORKSPACE/install
DEB_DIR=$WORKSPACE/debs

rm -r $FAKE_ROOT_DIR
rm -r $INSTALL_DIR

bash -c "cd $WORKSPACE && source /opt/ros/$ROS_VERSION/setup.bash &&\
catkin_make -DCMAKE_BUILD_TYPE=RELEASE -j2 && catkin_make -DCMAKE_BUILD_TYPE=RELEASE install --pkg $PACKAGE_NAME"

mkdir -p $FAKE_ROOT_DIR/DEBIAN/
mkdir -p $FAKE_ROOT_DIR/opt/ros/$ROS_VERSION
mkdir -p $DEB_DIR

if [ -d $INSTALL_DIR/include ]; then
   cp -r $INSTALL_DIR/include $FAKE_ROOT_DIR/opt/ros/$ROS_VERSION/
fi

if [ -d $INSTALL_DIR/lib ]; then
   cp -r $INSTALL_DIR/lib $FAKE_ROOT_DIR/opt/ros/$ROS_VERSION/
fi

cp -r $INSTALL_DIR/share $FAKE_ROOT_DIR/opt/ros/$ROS_VERSION/

cp $PACKAGE_DIR/control $FAKE_ROOT_DIR/DEBIAN/control

CONTROL_FILE=$FAKE_ROOT_DIR/DEBIAN/control

while [[ ! $(sed -n '$p' $CONTROL_FILE) =~ "Depends" ]] 
do
  if [[ $(sed -n '$p' $CONTROL_FILE) =~ "Architecture" ]]; then
    break
  fi
  sed -i '$d' $CONTROL_FILE
done

sed -i '$a\\' $CONTROL_FILE
VERSION=$(sed -n '2p' $CONTROL_FILE | grep -oP '[0-9]+\.[0-9]+\.[0-9]+')
echo $VERSION
BUILD_PACKAGE_NAME=$(sed -n '1p' $CONTROL_FILE | grep -oP 'Package: \K.*')

rm -f $DEB_DIR/$BUILD_PACKAGE_NAME*.deb
dpkg-deb -b $FAKE_ROOT_DIR $DEB_DIR/$BUILD_PACKAGE_NAME-${VERSION}-$ROS_VERSION.deb

