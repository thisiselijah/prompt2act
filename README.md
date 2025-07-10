### Playground Usage

```
docker run -it --rm \
  -e DISPLAY=host.docker.internal:0 \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ./catkin_ws:/root/catkin_ws \
  ros-noetic-dev
```