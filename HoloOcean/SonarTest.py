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
                            "sensor_type": "ImagingSonar",
                            "socket": "SonarSocket",
                            "Hz": 1,
                            "configuration": {
                                "RangeBins": 256,
                                "AzimuthBins": 64,
                                "RangeMin": 1,
                                "RangeMax": 30,
                                "InitOctreeRange": 50,
                                "Elevation": 20,
                                "Azimuth": 120,
                                "AzimuthStreaks": -1,
                                "ScaleNoise": True,
                                "AddSigma": 0.08,
                                "MultSigma": 0.08,
                                "RangeSigma": 0.1,
                                "MultiPath": True,
                                "ViewRegion": True,
                                "ViewOctree": -1
                            }
                        }
                    ],
                    "control_scheme": 0,
                    "location": [-2,0,8],
                    "rotation": [0.0, 0.0, 0.0]
                }
            ],
        "window_width": 800,
        "window_height": 1000    
        }



sonar_config = config['agents'][0]['sensors'][-1]["configuration"]
azi = sonar_config['Azimuth']
minR = sonar_config['RangeMin']
maxR = sonar_config['RangeMax']
binsR = sonar_config['RangeBins']
binsA = sonar_config['AzimuthBins']

plt.ion()
fig, ax = plt.subplots(subplot_kw=dict(projection='polar'), figsize=(8,5))
ax.set_theta_zero_location("N")
ax.set_thetamin(-azi/2)
ax.set_thetamax(azi/2)

theta = np.linspace(-azi/2, azi/2, binsA)*np.pi/180
r = np.linspace(minR, maxR, binsR)  
T, R = np.meshgrid(theta, r)
z = np.zeros_like(T)

plt.grid(False)
plot = ax.pcolormesh(T, R, z, cmap='gray', shading='auto', vmin=0, vmax=1)
plt.tight_layout()
fig.canvas.draw()
fig.canvas.flush_events()

#### RUN SIMULATION
command = np.array([0,0,0,0,0,0,0,0])
with HoloOceanEnvironment(scenario=config, start_world=False) as env:
    for i in range(800):
        env.act("auv0", command)
        state = env.tick()

        if 'ImagingSonar' in state:
            s = state['ImagingSonar']
            plot.set_array(s.ravel())

            fig.canvas.draw()
            fig.canvas.flush_events()

print("Finished Simulation!")
plt.ioff()
plt.show()
