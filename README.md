### Playground Usage

- Run playgound with `docker run` (macOS)

```
xhost + 127.0.0.1
$ docker run -it --rm \
  -e DISPLAY=host.docker.internal:0 \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_INDIRECT=1
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ./catkin_ws/src:/root/catkin_ws/src \
  ros-noetic-dev
```
- Linux 
```
xhost +local:root  

docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_INDIRECT=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ./catkin_ws/src:/root/catkin_ws/src \
  ros-noetic-dev

```
