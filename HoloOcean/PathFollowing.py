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
                    "location": [10,0,2],
                    "rotation": [0.0, 0.0, 180.0]
                }
            ],
        "window_width": 800,
        "window_height": 1000    
        }

pressed_keys = list()
force = 8


#### RUN SIMULATION
with HoloOceanEnvironment(scenario=config, start_world=False) as env:
    command = np.array([0, 0, 0, 0, 6.3, 6.3, 6.3, 6.3])
    for _ in range(1000):
        state = env.step(command)
        # To access specific sensor data:

    command = np.array([0, 0, 0, 0, 0.2, 0, 0, 0.2])
    for _ in range(1000):
        # We alternatively use the act function
        env.act("auv0", command)
        state = env.tick()

    command = np.array([0, 0, 0, 0, 1.7, 1.7, 1.7, 1.7])
    for _ in range(1000):
        env.act("auv0", command)
        state = env.tick()

    command = np.array([0, 0, 0, 0, 0.158, 0, 0, 0.158])
    for _ in range(1000):
        # We alternatively use the act function
        env.act("auv0", command)
        state = env.tick()

    command = np.array([0, 0, 0, 0, 5, 5, 5, 5])
    for _ in range(1000):
        # We alternatively use the act function
        env.act("auv0", command)
        state = env.tick()
