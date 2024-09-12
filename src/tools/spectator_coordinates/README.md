# Spectator Coordinate Echoer

Echos `x`, `y`, and `z` of spectator coordinates in Carla. This is used to retrieve coordinates to spawn actors or hardcode route strings.

## Usage

Carla must be running. Inside `carla_bridge` docker container:

```bash
./src/tools/spectator_coordinates/echo.py
# Or python ./src/tools/spectator_coordinates/echo.py
```