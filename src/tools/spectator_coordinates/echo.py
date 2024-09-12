#!/usr/bin/env python3

import os
import time

import carla

_HOST_ = "127.0.0.1"
_SLEEP_TIME_MS = 200
_PORT = int(os.environ["CARLA_PORT"])
_CLIENT_CONN_TIMEOUT_SECS = 10.0


def main():
    client = carla.Client(_HOST_, _PORT)
    client.set_timeout(_CLIENT_CONN_TIMEOUT_SECS)
    world = client.get_world()

    try:
        while True:
            t = world.get_spectator().get_transform()
            coordinate_str = "(x,y,z) = ({},{},{})".format(
                t.location.x, t.location.y, t.location.z
            )
            print(coordinate_str)
            time.sleep(_SLEEP_TIME_MS / 1000)
    except KeyboardInterrupt:
        # Shutdown process gracefully on Ctrl+C.
        exit(0)


if __name__ == "__main__":
    main()
