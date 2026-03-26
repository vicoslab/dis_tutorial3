# Navodila za zagon vsega

## Zaženi zenoh router
V terminalu 1:

```bash
pkill -9 -f ros && ros2 daemon stop
ros2 run rmw_zenoh_cpp rmw_zenohd
```

## Zaženi simulacijo
V terminalu 2:


Za že zgrajeno mapo:
```bash
ros2 launch dis_tutorial3 sim_turtlebot_nav.launch.py
```

## Zaženi detect_people.py` skripto
V terminalu 3:

```bash
source /opt/ultralytics/bin/activate
ros2 run dis_tutorial3 detect_people.py
```

## Zaženi `patrol_people_collector.py` skripto
V terminalu 4:

```bash
ros2 run dis_tutorial3 patrol_people_collector.py
```

## Izvedi service call za zagon obhoda
V terminalu 5:

```bash
ros2 service call /start_patrol std_srvs/srv/Trigger "{}"
```
