/*    
    This file is a part of Stonefish.

    Stonefish is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Stonefish is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  UnderwaterTestManager.cpp
//  Stonefish
//
//  Created by Patryk Cieslak on 04/03/2014.
//  Copyright(c) 2014-2023 Patryk Cieslak. All rights reserved.
//

#include "UnderwaterTestManager.h"

#include <iostream>
#include <fstream>

#include <Stonefish/sensors/scalar/IMU.h>
#include "UnderwaterTestApp.h"
#include <core/FeatherstoneRobot.h>
#include <entities/statics/Plane.h>
#include <entities/statics/Obstacle.h>
#include <entities/solids/Polyhedron.h>
#include <entities/solids/Box.h>
#include <entities/solids/Sphere.h>
#include <entities/solids/Torus.h>
#include <entities/solids/Cylinder.h>
#include <entities/solids/Compound.h>
#include <entities/solids/Wing.h>
#include <graphics/OpenGLPointLight.h>
#include <graphics/OpenGLSpotLight.h>
#include <graphics/OpenGLTrackball.h>
#include <utils/SystemUtil.hpp>
#include <entities/statics/Obstacle.h>
#include <entities/statics/Terrain.h>
#include <actuators/Thruster.h>
#include <actuators/Servo.h>
#include <actuators/VariableBuoyancy.h>
#include <sensors/scalar/Pressure.h>
#include <sensors/scalar/Odometry.h>
#include <sensors/scalar/DVL.h>
#include <sensors/scalar/Compass.h>
#include <sensors/scalar/IMU.h>
#include <sensors/scalar/GPS.h>
#include <sensors/Contact.h>
#include <sensors/vision/ColorCamera.h>
#include <sensors/vision/DepthCamera.h>
#include <sensors/vision/Multibeam2.h>
#include <sensors/vision/FLS.h>
#include <sensors/vision/SSS.h>
#include <sensors/vision/MSIS.h>
#include <comms/AcousticModem.h>
#include <sensors/Sample.h>
#include <actuators/Light.h>
#include <sensors/scalar/RotaryEncoder.h>
#include <sensors/scalar/Accelerometer.h>
#include <entities/FeatherstoneEntity.h>
#include <entities/forcefields/Trigger.h>
#include <entities/forcefields/Pipe.h>
#include <entities/forcefields/Jet.h>
#include <entities/forcefields/Uniform.h>
#include <entities/AnimatedEntity.h>
#include <sensors/scalar/Profiler.h>
#include <sensors/scalar/Multibeam.h>
#include <utils/UnitSystem.h>
#include <core/ScenarioParser.h>
#include <core/NED.h>

FILE *freopen(const char string[13], const char string1[2], const char string2[48], sf::Scalar d, sf::Scalar d1,
              sf::Scalar d2);

UnderwaterTestManager::UnderwaterTestManager(sf::Scalar stepsPerSecond)
: SimulationManager(stepsPerSecond, sf::SolverType::SOLVER_SI, sf::CollisionFilteringType::COLLISION_EXCLUSIVE)
{
}

void UnderwaterTestManager::BuildScenario()
{
#ifdef PARSED_SCENARIO
    sf::ScenarioParser parser(this);
    bool success = parser.Parse(sf::GetDataPath() + "underwater_test.scn");
    if(!success)
        cCritical("Scenario parser: Parsing failed!");
#else
    ///////MATERIALS////////
    CreateMaterial("Dummy", sf::UnitSystem::Density(sf::CGS, sf::MKS, 0.9), 0.3);
    CreateMaterial("Fiberglass", sf::UnitSystem::Density(sf::CGS, sf::MKS, 1.5), 0.9);
    CreateMaterial("Rock", sf::UnitSystem::Density(sf::CGS, sf::MKS, 3.0), 0.6);
    SetMaterialsInteraction("Dummy", "Dummy", 0.5, 0.2);
    SetMaterialsInteraction("Fiberglass", "Fiberglass", 0.5, 0.2);
    SetMaterialsInteraction("Rock", "Rock", 0.9, 0.7);
    SetMaterialsInteraction("Fiberglass", "Dummy", 0.5, 0.2);
    SetMaterialsInteraction("Rock", "Dummy", 0.6, 0.4);
    SetMaterialsInteraction("Rock", "Fiberglass", 0.6, 0.4);

    ///////LOOKS///////////
    CreateLook("yellow", sf::Color::RGB(1.f, 0.9f, 0.f), 0.3f, 0.f);
    CreateLook("grey", sf::Color::RGB(0.3f, 0.3f, 0.3f), 0.4f, 0.5f);
    CreateLook("seabed", sf::Color::RGB(0.7f, 0.7f, 0.5f), 0.9f, 0.f, 0.f, "/home/external-holo/stonefish/Tests/Data/sand_normal.png");
    CreateLook("propeller", sf::Color::RGB(1.f, 1.f, 1.f), 0.3f, 0.f, 0.f, "/home/external-holo/stonefish/Tests/Data/propeller_tex.png");
    CreateLook("black", sf::Color::RGB(0.1f, 0.1f, 0.1f), 0.4f, 0.5f);
    CreateLook("manipulator", sf::Color::RGB(0.2f, 0.15f, 0.1f), 0.6f, 0.8f);
    CreateLook("link4", sf::Color::RGB(1.f, 1.f, 1.f), 0.6f, 0.8f, 0.f, "/home/external-holo/stonefish/Tests/Data/link4_tex.png");

    ////////OBJECTS
    //Create environment
    EnableOcean(0.0);
    getOcean()->setWaterType(0.2);
    getOcean()->AddVelocityField(new sf::Jet(sf::Vector3(0,0,0), sf::VY(), 0.3, 5.0));
    getOcean()->AddVelocityField(new sf::Uniform(sf::Vector3(0,0.0,0.0)));
    getOcean()->EnableCurrents();
    getAtmosphere()->SetupSunPosition(0.0, 60.0);
    getNED()->Init(41.77737, 3.03376, 0.0);

    //sf::Obstacle* tank = new sf::Obstacle("CIRS Tank", sf::GetDataPath() + "cirs_tank.obj", 1.0, sf::I4(), "Rock", "seabed");
    //AddStaticEntity(tank, sf::I4());
    //sf::Plane* seabed = new sf::Plane("Seabed", 1000.0, "Rock", "seabed", 5.f);
    //AddStaticEntity(seabed, sf::Transform(sf::IQ(), sf::Vector3(0,0,5.0)));
    sf::Terrain* seabed = new sf::Terrain("Seabed", "/home/external-holo/stonefish/Tests/Data/terrain.png", 1.0, 1.0, 5.0, "Rock", "seabed", 5.f);
    AddStaticEntity(seabed, sf::Transform(sf::IQ(), sf::Vector3(0,0,15.0)));

    sf::Obstacle* cyl = new sf::Obstacle("Cyl", 0.5, 5.0, sf::I4(), "Fiberglass", "yellow");
    AddStaticEntity(cyl, sf::Transform(sf::Quaternion(0,M_PI_2,0), sf::Vector3(6.0,2.0,5.0)));
    sf::Obstacle* ball = new sf::Obstacle("Ball", 0.5,sf::I4(), "Fiberglass", "yellow");
    AddStaticEntity(ball, sf::Transform(sf::Quaternion(0.0, 0.0, 0.0), sf::Vector3(2.0, 0.0, 5.0)));

	sf::Light* spot = new sf::Light("Spot", 0.02, 50.0, sf::Color::BlackBody(5000.0), 100.0);
	spot->AttachToWorld(sf::Transform(sf::Quaternion(0,0,M_PI/3.0), sf::Vector3(0.0,0.0,1.0)));
	AddActuator(spot);

    sf::Light* omni = new sf::Light("Omni", 0.02, sf::Color::BlackBody(5000.0), 10000.0);
	omni->AttachToWorld(sf::Transform(sf::Quaternion(0,0,M_PI/3.0), sf::Vector3(2.0,2.0,0.5)));
	AddActuator(omni);

    //Create underwater vehicle body
    //Externals
    sf::BodyPhysicsSettings phy;
    phy.mode = sf::BodyPhysicsMode::SUBMERGED;
    phy.collisions = true;

    phy.buoyancy = false;
    sf::Polyhedron* hullB = new sf::Polyhedron("HullBottom", phy, "/home/external-holo/stonefish/Tests/Data/hull_hydro.obj", sf::Scalar(1), sf::I4(), "Fiberglass", "yellow", sf::Scalar(0.003));
    sf::Polyhedron* hullP = new sf::Polyhedron("HullPort", phy, "/home/external-holo/stonefish/Tests/Data/hull_hydro.obj", sf::Scalar(1), sf::I4(), "Fiberglass", "yellow", sf::Scalar(0.003));
    sf::Polyhedron* hullS = new sf::Polyhedron("HullStarboard", phy, "/home/external-holo/stonefish/Tests/Data/hull_hydro.obj", sf::Scalar(1), sf::I4(), "Fiberglass", "yellow", sf::Scalar(0.003));
    sf::Polyhedron* vBarStern = new sf::Polyhedron("VBarStern", phy, "/home/external-holo/stonefish/Tests/Data/vbar_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "grey", sf::Scalar(0.003));
    sf::Polyhedron* vBarBow = new sf::Polyhedron("VBarBow", phy, "/home/external-holo/stonefish/Tests/Data/vbar_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "grey", sf::Scalar(0.003));

    phy.buoyancy = true;
    sf::Polyhedron* ductSway = new sf::Polyhedron("DuctSway", phy, "/home/external-holo/stonefish/Tests/Data/duct_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "black");
    sf::Polyhedron* ductSurgeP = new sf::Polyhedron("DuctSurgePort", phy, "/home/external-holo/stonefish/Tests/Data/duct_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "black");
    sf::Polyhedron* ductSurgeS = new sf::Polyhedron("DuctSurgeStarboard", phy, "/home/external-holo/stonefish/Tests/Data/duct_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "black");
    sf::Polyhedron* ductHeaveS = new sf::Polyhedron("DuctHeaveStern", phy, "/home/external-holo/stonefish/Tests/Data/duct_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "black");
    sf::Polyhedron* ductHeaveB = new sf::Polyhedron("DuctHeaveBow", phy, "/home/external-holo/stonefish/Tests/Data/duct_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "black");
    //Internals
    sf::Cylinder* batteryCyl = new sf::Cylinder("BatteryCylinder", phy, 0.13, 0.6, sf::I4(), "Dummy", "manipulator");
    batteryCyl->ScalePhysicalPropertiesToArbitraryMass(sf::Scalar(70));
    sf::Cylinder* portCyl = new sf::Cylinder("PortCylinder", phy, 0.13, 1.0, sf::I4(), "Dummy", "manipulator");
    portCyl->ScalePhysicalPropertiesToArbitraryMass(sf::Scalar(20));
    sf::Cylinder* starboardCyl = new sf::Cylinder("StarboardCylinder", phy, 0.13, 1.0, sf::I4(), "Dummy", "manipulator");
    starboardCyl->ScalePhysicalPropertiesToArbitraryMass(sf::Scalar(20));

    //Build whole body
    sf::Compound* vehicle = new sf::Compound("Vehicle", phy, hullB, sf::I4());
    vehicle->AddExternalPart(hullP, sf::Transform(sf::IQ(), sf::Vector3(0,-0.35,-0.7)));
    vehicle->AddExternalPart(hullS, sf::Transform(sf::IQ(), sf::Vector3(0,0.35,-0.7)));
    vehicle->AddExternalPart(vBarStern, sf::Transform(sf::Quaternion::getIdentity(), sf::Vector3(-0.25,0.0,-0.15)));
    vehicle->AddExternalPart(vBarBow, sf::Transform(sf::Quaternion::getIdentity(), sf::Vector3(0.30,0.0,-0.15)));
    vehicle->AddExternalPart(ductSway, sf::Transform(sf::Quaternion(M_PI_2,M_PI,0), sf::Vector3(-0.0137, 0.0307, -0.38)));
    vehicle->AddExternalPart(ductSurgeP, sf::Transform(sf::Quaternion(0,0,M_PI), sf::Vector3(-0.2807,-0.2587,-0.38)));
    vehicle->AddExternalPart(ductSurgeS, sf::Transform(sf::Quaternion(0,0,0), sf::Vector3(-0.2807,0.2587,-0.38)));
    vehicle->AddExternalPart(ductHeaveS, sf::Transform(sf::Quaternion(M_PI_2,-M_PI_2,0), sf::Vector3(-0.5337,0.0,-0.6747)));
    vehicle->AddExternalPart(ductHeaveB, sf::Transform(sf::Quaternion(-M_PI_2,-M_PI_2,0), sf::Vector3(0.5837,0.0,-0.6747)));
    vehicle->AddInternalPart(batteryCyl, sf::Transform(sf::Quaternion(0,M_PI_2,0), sf::Vector3(-0.1,0,0)));
    vehicle->AddInternalPart(portCyl, sf::Transform(sf::Quaternion(0,M_PI_2,0), sf::Vector3(0.0,-0.35,-0.7)));
    vehicle->AddInternalPart(starboardCyl, sf::Transform(sf::Quaternion(0,M_PI_2,0), sf::Vector3(0.0,0.35,-0.7)));

    vehicle->setDisplayInternalParts(false);

    //Manipulator bodies
    sf::Polyhedron* baseLink = new sf::Polyhedron("ArmBaseLink", phy, "/home/external-holo/stonefish/Tests/Data/base_link_uji_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "manipulator");
    sf::Polyhedron* link1 = new sf::Polyhedron("ArmLink1", phy, "/home/external-holo/stonefish/Tests/Data/link1_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "manipulator");
    sf::Polyhedron* link2 = new sf::Polyhedron("ArmLink2", phy, "/home/external-holo/stonefish/Tests/Data/link2_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "manipulator");
    sf::Polyhedron* link3 = new sf::Polyhedron("ArmLink3", phy, "/home/external-holo/stonefish/Tests/Data/link3_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "manipulator");
    sf::Polyhedron* link4 = new sf::Polyhedron("ArmLink4", phy, "/home/external-holo/stonefish/Tests/Data/link4_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "link4");
    sf::Polyhedron* ee = new sf::Polyhedron("EE", phy, "/home/external-holo/stonefish/Tests/Data/eeprobe_hydro.obj", sf::Scalar(1), sf::I4(), "Neutral", "manipulator");
    sf::Polyhedron* finger1 = new sf::Polyhedron("Finger1", phy, "/home/external-holo/stonefish/Tests/Data/fingerA_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "manipulator");
    sf::Polyhedron* finger2 = new sf::Polyhedron("Finger2", phy, "/home/external-holo/stonefish/Tests/Data/fingerA_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "manipulator");


    std::vector<sf::SolidEntity*> arm;
    arm.push_back(baseLink);
    arm.push_back(link1);
    arm.push_back(link2);
    arm.push_back(link3);
    arm.push_back(link4);
    arm.push_back(ee);
    arm.push_back(finger1);
    arm.push_back(finger2);

    //Create manipulator servomotors
    sf::Servo* srv1 = new sf::Servo("Servo1", 1.0, 1.0, 100.0);
    sf::Servo* srv2 = new sf::Servo("Servo2", 1.0, 1.0, 100.0);
    sf::Servo* srv3 = new sf::Servo("Servo3", 1.0, 1.0, 100.0);
    sf::Servo* srv4 = new sf::Servo("Servo4", 1.0, 1.0, 100.0);
    sf::Servo* srv5 = new sf::Servo("FServo1", 1.0, 1.0, 10.0);
    sf::Servo* srv6 = new sf::Servo("FServo2", 1.0, 1.0, 10.0);

    //Create thrusters
    sf::Polyhedron* prop1 = new sf::Polyhedron("Propeller1", phy, "/home/external-holo/stonefish/Tests/Data/propeller.obj", sf::Scalar(1), sf::I4(), "Dummy", "propeller");
    sf::Polyhedron* prop2 = new sf::Polyhedron("Propeller2", phy, "/home/external-holo/stonefish/Tests/Data/propeller.obj", sf::Scalar(1), sf::I4(), "Dummy", "propeller");
    sf::Polyhedron* prop3 = new sf::Polyhedron("Propeller3", phy, "/home/external-holo/stonefish/Tests/Data/propeller.obj", sf::Scalar(1), sf::I4(), "Dummy", "propeller");
    sf::Polyhedron* prop4 = new sf::Polyhedron("Propeller4", phy, "/home/external-holo/stonefish/Tests/Data/propeller.obj", sf::Scalar(1), sf::I4(), "Dummy", "propeller");
    sf::Polyhedron* prop5 = new sf::Polyhedron("Propeller5", phy, "/home/external-holo/stonefish/Tests/Data/propeller.obj", sf::Scalar(1), sf::I4(), "Dummy", "propeller");
    sf::Thruster* thSway = new sf::Thruster("ThrusterSway", prop1, 0.18, std::make_pair(0.48, 0.48), 0.05, 1000.0, true);
    sf::Thruster* thSurgeP = new sf::Thruster("ThrusterSurgePort", prop2, 0.18, std::make_pair(0.88, 0.48), 0.05, 1000.0, true);
    sf::Thruster* thSurgeS = new sf::Thruster("ThrusterSurgeStarboard", prop3, 0.18, std::make_pair(0.48, 0.48), 0.05, 1000.0, true);
    sf::Thruster* thHeaveS = new sf::Thruster("ThrusterHeaveStern", prop4, 0.18, std::make_pair(0.48, 0.48), 0.05, 1000.0, true);
    sf::Thruster* thHeaveB = new sf::Thruster("ThrusterHeaveBow", prop5, 0.18, std::make_pair(0.48, 0.48), 0.05, 1000.0, true);

    //Create VBS
    //std::vector<std::string> vmeshes;
    //vmeshes.push_back(sf::GetDataPath() + "vbs_max.obj");
    //vmeshes.push_back(sf::GetDataPath() + "vbs_min.obj");
    //sf::VariableBuoyancy* vbs = new sf::VariableBuoyancy("VBS", vmeshes, 0.002);

    //Create ligths
    //sf::Light* spot1 = new sf::Light("Spot1", sf::Color::BlackBody(4000.0), 100000.0); //OMNI
    //sf::Light* spot1 = new sf::Light("Spot1", 30.0, sf::Color::BlackBody(4000.0), 100000.0);

    //Create sensors
    sf::Odometry* odom = new sf::Odometry("Odom");
    sf::Pressure* press = new sf::Pressure("Pressure");
    press->setNoise(1.0);

    sf::DVL* dvl = new sf::DVL("DVL", 30.0, false);
    dvl->setNoise(0.0, 0.02, 0.05, 0.0, 0.02);
    dvl->setRange(sf::Vector3(0.05, 0.05, 0.1), 0, 20);

    sf::IMU* imu = new sf::IMU("IMU");
    imu->setNoise(sf::V0(), sf::Vector3(0.05, 0.05, 0.1), 0.0, sf::Vector3(0.01, 0.01, 0.02));
    sf::Compass* fog = new sf::Compass("FOG");
    fog->setNoise(0.01);
    sf::GPS* gps = new sf::GPS("GPS");
    gps->setNoise(0.5);
    //sf::Multibeam2* mb = new sf::Multibeam2("Multibeam", 1000, 300, 50.0, 40.0, 0.1, 10.0, 10.0);
    //mb->setDisplayOnScreen(true);
    //sf::DepthCamera* dc = new sf::DepthCamera("DepthCam", 1000, 350, 50.0, 0.1, 10.0, 10.0);
    //dc->setDisplayOnScreen(true);
    sf::FLS* fls = new sf::FLS("FLS", 256, 500, 150.0, 30.0, 1.0, 20.0, sf::ColorMap::GREEN_BLUE);
    fls->setNoise(0.05, 0.05);
    fls->setDisplayOnScreen(true, 150, 120, 0.5f);
    //sf::SSS* sss = new sf::SSS("SSS", 800, 400, 70.0, 1.5, 50.0, 1.0, 100.0, sf::ColorMap::GREEN_BLUE);
    //sss->setDisplayOnScreen(true, 710, 5, 0.6f);
    //sf::MSIS* msis = new sf::MSIS("MSIS", 1.5, 500, 2.0, 30.0, -50, 50, 1.0, 100.0, sf::ColorMap::GREEN_BLUE);
    //msis->setDisplayOnScreen(true, 880, 455, 0.6f);
    //sf::ColorCamera* cam = new sf::ColorCamera("Cam", 300, 200, 60.0, 10.0);
    //cam->setDisplayOnScreen(true, 600, 400, 200);
    //sf::ColorCamera* cam2 = new sf::ColorCamera("Cam", 300, 200, 60.0);

    //Create AUV
    sf::Robot* auv = new sf::FeatherstoneRobot("GIRONA500", false);

    //Mechanical structure
    auv->DefineLinks(vehicle, arm);
    auv->DefineFixedJoint("VehicleToArm", "Vehicle", "ArmBaseLink", sf::Transform(sf::IQ(), sf::Vector3(0.74,0.0,0.0)));
    auv->DefineRevoluteJoint("Joint1", "ArmBaseLink", "ArmLink1", sf::I4(), sf::Vector3(0.0, 0.0, 1.0), std::make_pair(-1.0, 1.0));
    auv->DefineRevoluteJoint("Joint2", "ArmLink1", "ArmLink2", sf::Transform(sf::IQ(), sf::Vector3(0.1065, 0.0, 0.0)), sf::Vector3(0.0, 1.0, 0.0), std::make_pair(-1.0, 1.0));
    auv->DefineRevoluteJoint("Joint3", "ArmLink2", "ArmLink3", sf::Transform(sf::IQ(), sf::Vector3(0.23332, 0.0, 0.0)), sf::Vector3(0.0, 1.0, 0.0), std::make_pair(-1.0, 1.0));
    auv->DefineRevoluteJoint("Joint4", "ArmLink3", "ArmLink4", sf::Transform(sf::IQ(), sf::Vector3(0.103, 0.0, 0.201)), sf::Vector3(0.0, 0.0, 1.0),  std::make_pair(-1.0, 1.0));
    auv->DefineFixedJoint("Fix", "ArmLink4", "EE", sf::Transform(sf::IQ(), sf::Vector3(0.0, 0.0, 0.05)));
    auv->DefineRevoluteJoint("Joint5", "EE", "Finger1", sf::Transform(sf::IQ(), sf::Vector3(0.03,0,0.1)), sf::VY(), std::make_pair(0.0, 1.0));
    auv->DefineRevoluteJoint("Joint6", "EE", "Finger2", sf::Transform(sf::Quaternion(M_PI, 0.0, 0.0), sf::Vector3(-0.03,0,0.1)), sf::VY(), std::make_pair(0.0, 1.0));
    auv->BuildKinematicStructure();

    //Joint motors
    auv->AddJointActuator(srv1, "Joint1");
    auv->AddJointActuator(srv2, "Joint2");
    auv->AddJointActuator(srv3, "Joint3");
    auv->AddJointActuator(srv4, "Joint4");
    auv->AddJointActuator(srv5, "Joint5");
    auv->AddJointActuator(srv6, "Joint6");

    //Thrusters
    auv->AddLinkActuator(thSway, "Vehicle", sf::Transform(sf::Quaternion(M_PI_2,M_PI,0), sf::Vector3(-0.0137, 0.0307, -0.38)));
    auv->AddLinkActuator(thSurgeP, "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(-0.2807,-0.2587,-0.38)));
    auv->AddLinkActuator(thSurgeS, "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(-0.2807,0.2587,-0.38)));
    auv->AddLinkActuator(thHeaveS, "Vehicle", sf::Transform(sf::Quaternion(0,-M_PI_2,0), sf::Vector3(-0.5337,0.0,-0.6747)));
    auv->AddLinkActuator(thHeaveB, "Vehicle", sf::Transform(sf::Quaternion(0,-M_PI_2,0), sf::Vector3(0.5837,0.0,-0.6747)));
    //auv->AddLinkActuator(spot1, "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(0,0,1.0)));
    //auv->AddLinkActuator(vbs, "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(-0.5,0.0,0.0)));

    //Sensors
    auv->AddLinkSensor(odom, "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(0,0,0)));
    auv->AddLinkSensor(press, "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(0.6,0,-0.7)));
    auv->AddLinkSensor(dvl, "Vehicle", sf::Transform(sf::Quaternion(-M_PI_4,0,M_PI), sf::Vector3(-0.5,0,0.1)));
    auv->AddLinkSensor(imu, "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(0,0,-0.7)));
    auv->AddLinkSensor(fog, "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(0.3,0,-0.7)));
    auv->AddLinkSensor(gps, "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(-0.5,0,-0.9)));
    auv->AddVisionSensor(fls, "Vehicle", sf::Transform(sf::Quaternion(1.57, 0.0, 0.8), sf::Vector3(0.0,0.0,1.0)));
    //auv->AddVisionSensor(sss, "Vehicle", sf::Transform(sf::Quaternion(1.57, 0.0, 0.0), sf::Vector3(0.0,0.0,0.0)));
    //auv->AddVisionSensor(msis, "Vehicle", sf::Transform(sf::Quaternion(0.0, 0.0, 1.57), sf::Vector3(0.0,0.0,1.0)));
    //auv->AddVisionSensor(cam, "Vehicle", sf::Transform(sf::Quaternion(1.57, 0.0, 1.57), sf::Vector3(0.0,0.0,1.0)));
    //auv->AddVisionSensor(cam2, "Vehicle", sf::Transform(sf::Quaternion(1.57, 0.0, 1.57), sf::Vector3(0.0,0.0,2.0)));
    AddRobot(auv, sf::Transform(sf::Quaternion(0,0,0), sf::Vector3(0.0,0.0,2.0)));

    //thSurgeP->setSetpoint(0.55);
    //thSurgeS->setSetpoint(0.58);

#endif
}

void UnderwaterTestManager::SimulationStepCompleted(sf::Scalar timeStep)
{
    cInfo("Simulation time: %1.3lf", getSimulationTime());
    sf::Scalar Time = getSimulationTime();

    sf::DVL* ddvl = (sf::DVL*)getRobot("GIRONA500")->getSensor("DVL");
    sf::FLS* fls = (sf::FLS*)getRobot("GIRONA500")->getSensor("FLS");
    printf("BeamAngle: %1.3lf\n", ddvl->getBeamAngle());

    if(Time > 1 && Time < 1.05) {
        std::ofstream outFile("/home/external-holo/stonefish/Simulator/Sim2/dataSim2.txt");
        std::ostream &output = outFile.is_open() ? outFile : std::cout;
        GLubyte *displayData = fls->getDisplayDataPointer();
        if ((displayData != nullptr) && (outFile.is_open())) {

            unsigned int width = 480;
            unsigned int height = 264;

            for (unsigned int y = 0; y < height; ++y) {
                outFile << std::endl;
                for (unsigned int x = 0; x < width; ++x) {

                    unsigned int index = (y * width + x) * 3;
                    GLubyte red = displayData[index];
                    GLubyte green = displayData[index + 1];
                    GLubyte blue = displayData[index + 2];
                    output << "(" << x << ", " << y << "): R=" << (int) red << ", G=" << (int) green << ", B="
                           << (int) blue << "  ";
                    std::cout << "(" << x << ", " << y << "): R=" << (int) red << ", G=" << (int) green << ", B="
                              << (int) blue << std::endl;
                }
            }
            std::cout << "Pixel data has been written to pixel_data.txt." << std::endl;
        } else {
            std::cerr << "Display data pointer is null." << std::endl;
        }

        if (outFile.is_open()) {
            outFile.close();
        }
    }
    /*void* imageData = fls->getImageDataPointer();
    if (imageData != nullptr) {
        // Iterate over the imageData and print each element
        std::cout << "Something" << std::endl;
    } else {
        std::cerr << "Error: Image data is nullptr!" << std::endl;
    }

    std::cout << "What is this?: " << fls->getDisplayDataPointer() << std::endl;

     */

    /*sf::Thruster* th = (sf::Thruster*)getRobot("GIRONA500")->getActuator("ThrusterSurgePort");
    printf("Setpoint: %1.3lf Thrust: %1.3lf Torque: %1.3lf\n", th->getSetpoint(), th->getThrust(), th->getTorque());
     */

    //printf("Velocity X:\n", ddvl->getVelocityX());

    /* std::ofstream outfile("/home/external-holo/stonefish/Simulator/Sim2/dataSim2.txt", std::ios::app);
    std::ostream& output = outfile.is_open() ? outfile : std::cout;

    output <<"Simulation time: " << getSimulationTime() << "\n Setpoint: " << th->getSetpoint() << ", Thrust: " << th->getThrust() << ", Torque: " << th->getTorque() << std::endl;

    if (outfile.is_open()) {
        outfile.close();
    }

     */

        /*sf::Comm* modem = getComm("Modem");
        if(modem->isNewDataAvailable())
        {
            sf::Vector3 pos;
            std::string frame;
            ((sf::AcousticModem*)modem)->getPosition(pos, frame);
            printf("%1.3lf %1.3lf %1.3lf\n", pos.getX(), pos.getY(), pos.getZ());
            modem->MarkDataOld();
        }
        */
}