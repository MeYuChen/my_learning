#!/bin/bash
CURRENT_DIR=$(cd `dirname $0`; pwd)
WORKSPACE=$(cd $CURRENT_DIR/../../..; pwd)

for file in $WORKSPACE/src/*
do
if [ -d "$file" ]; then
echo $file
bash $file/deb_maker/build_example.sh
fi;
done
