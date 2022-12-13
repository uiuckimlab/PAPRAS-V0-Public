# Headless Docker env setup for NUC

1. Build Docker environment

```
docker build -t papras .
```

2. Open a terminal inside the environment with this repository mounted

```
docker run -it -v `pwd`:/opt/catkin_ws/src/PAPRAS papras

or

./gui-docker -it -v `pwd`:/opt/catkin_ws/src/PAPRAS papras
./gui-docker -c <container>
```

3. Go to `/opt/arpp`, the mounted directory, build the package, and configure ROS

```
cd /opt/catkin_ws
catkin_make
source devel/setup.bash
```