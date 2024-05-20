# import holoocean
import cv2
import numpy as np
from pynput import keyboard
import matplotlib.pyplot as plt
from holoocean.environments import HoloOceanEnvironment

config ={   "name": "HoveringImagingSonar",
            "world": "SimpleUnderwater",
            "main_agent": "auv0",
            "ticks_per_sec": 200,
            "frames_per_sec": True,
            "octree_min": 0.02,
            "octree_max": 5.0,
            "agents":[
                {
                    "agent_name": "auv0",
                    "agent_type": "HoveringAUV",
                    "sensors": [
                        {
                            "sensor_type": "PoseSensor",
                            "socket": "IMUSocket"
                        },
                        {
                            "sensor_type": "VelocitySensor",
                            "socket": "IMUSocket"
                        },
                        {
                            "sensor_type": "IMUSensor",
                            "sensor_name": "IMUSensor",
                            "socket": "IMUSocket",
                            "Hz": 200,
                            "configuration": {
                                "AccelSigma": 0.00277,
                                "AngVelSigma": 0.00123,
                                "AccelBiasSigma": 0.00141,
                                "AngVelBiasSigma": 0.00388,
                                "ReturnBias": True
                            }
                        },
                        {
                            "sensor_type": "GPSSensor",
                            "socket": "IMUSocket",
                            "Hz": 5,
                            "configuration":{
                                "Sigma": 0.5,
                                "Depth": 1,
                                "DepthSigma": 0.25
                            }
                        },
                        {
                            "sensor_type": "DVLSensor",
                            "socket": "DVLSocket",
                            "Hz": 20,
                            "configuration": {
                                "Elevation": 22.5,
                                "VelSigma": 0.02626,
                                "ReturnRange": True,
                                "MaxRange": 50,
                                "RangeSigma": 0.1
                            }
                        },
                        {
                            "sensor_type": "DepthSensor",
                            "socket": "DepthSocket",
                            "Hz": 100,
                            "configuration": {
                                "Sigma": 0.255
                            }
                        },
                        {
                            "sensor_type": "RGBCamera",
                            "sensor_name": "LeftCamera",
                            "socket": "CameraLeftSocket",
                            "Hz": 5,
                            "configuration": {
                                "CaptureWidth": 512,
                                "CaptureHeight": 512
                            }
                        }
                    ],
                    "control_scheme": 0,
                    "location": [-2,0,8],
                    "rotation": [0.0, 0.0, 0.0]
                }
            ],
        "window_width": 1200,
        "window_height": 720    
        }

pressed_keys = list()
force = 25


def on_press(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.append(key.char)
        pressed_keys = list(set(pressed_keys))


def on_release(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.remove(key.char)


listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()


def parse_keys(keys, val):
    command = np.zeros(8)
    if 'i' in keys:
        command[0:4] += val
    if 'k' in keys:
        command[0:4] -= val
    if 'j' in keys:
        command[[4, 7]] += val*0.1
        command[[5, 6]] -= val*0.1
    if 'l' in keys:
        command[[4, 7]] -= val*0.1
        command[[5, 6]] += val*0.1

    if 'w' in keys:
        command[4:8] += val
    if 's' in keys:
        command[4:8] -= val
    if 'a' in keys:
        command[[4, 6]] += val
        command[[5, 7]] -= val
    if 'd' in keys:
        command[[4, 6]] -= val
        command[[5, 7]] += val

    return command*1


sonar_config = config['agents'][0]['sensors'][-1]["configuration"]
plt.ion()
plt.grid(False)

#### RUN SIMULATION
# command = np.array([0,0,0,0,0,0,0,0])
with HoloOceanEnvironment(scenario=config, start_world=False) as env:
    while True:

        command = parse_keys(pressed_keys, force)
        env.act("auv0", command)
        state = env.tick()

        if "LeftCamera" in state:
            pixels = state["LeftCamera"]
            cv2.namedWindow("Camera Output")
            cv2.imshow("Camera Output", pixels[:, :, 0:3])
            if cv2.waitKey(1) & 0xFF == ord('e'):
             break
             
