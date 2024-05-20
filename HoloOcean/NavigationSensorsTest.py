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
                            "sensor_type": "RotationSensor",
                            "socket": "IMUSocket"
                        },
                        {
                            "sensor_type": "VelocitySensor",
                            "socket": "IMUSocket"
                        },
                        {
                            "sensor_type": "LocationSensor",
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
                            "sensor_name": "DVLSensor",
                            "socket": "IMUSocket",
                            "Hz": 200,
                            "configuration": {
                                "Elevation": 22.7,
                                "DebugLines": False,
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
                    ],
                    "control_scheme": 0,
                    "location": [10,0,8],
                    "rotation": [0.0, 0.0, 0.0]
                }
            ],
        "window_width": 800,
        "window_height": 1000    
        }

pressed_keys = list()
force = 8


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

#### RUN SIMULATION
with HoloOceanEnvironment(scenario=config, start_world=False) as env:
    while True:

        command = parse_keys(pressed_keys, force)
        env.act("auv0", command)
        state = env.tick()
        RotationSensor = state["RotationSensor"]
        Location = state["LocationSensor"]
        IMU = state["IMUSensor"]
        DVL = state["DVLSensor"]

        try:
            counter += 1
        except NameError:
            counter = 1

        print("This is the Rotation Sensor \n",RotationSensor)
        print("This is the Location \n",Location)
        print("This is the IMU \n", IMU)
        print("This is the DVL \n", DVL)

        with open("SensorMeasurements.txt", "a") as file:
            file.write(f"Measurement {counter}:\n")
            file.write("Rotation Sensor:\n")
            file.write(str(RotationSensor) + "\n\n")
            file.write("IMU:\n")
            file.write(str(IMU) + "\n\n")
            file.write("DVL:\n")
            file.write(str(DVL) + "\n\n")
            file.write("Location:\n")
            file.write(str(Location) + "\n\n")
        


