#!/bin/sh

od2trips -n taz.xml -z odpairs.xml  -o trips.xml --vtype passenger

duarouter -n quickstart.net.xml -a vtype.add.xml -r trips.xml -o routes.xml --ignore-errors

sumo -n quickstart.net.xml -b 0 -e 3600 -a=edgedata.add.xml,det.add.xml -r routes.xml --ignore-route-errors

# to visualize: sumo-gui -n quickstart.net.xml -b 0 -e 3600 -r routes.xml --ignore-route-errors

python InductionLoopToTable.py