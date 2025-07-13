# Robot AI Arm
## Overview
This repository houses the software and hardware designs for an AI-powered Robot Arm, a sophisticated project designed to bring intelligent automation to your workstation. The system integrates real-time object recognition, precise physical manipulation, and an intuitive user interface to interact with its environment. This project showcases a blend of C#, Python, and AXML scripts built upon the Avalonia UI framework, demonstrating robust cross-platform capabilities, especially on embedded systems like the Raspberry Pi.

## Features
The AI Robot Arm software features a flexible, rearrangeable user interface with five main sections, providing comprehensive control and monitoring capabilities:

### 1. Camera View
Experience real-time visual feedback from the robot's perspective. This section was a significant development challenge, particularly achieving a live camera feed on the Raspberry Pi with Avalonia, where readily available solutions were scarce. The innovative solution involves continuously updating a JPG image on a specific document and dynamically resizing the display to adapt seamlessly to your UI layout.

### 2. Processed View
This view leverages a Python vision script for advanced object and people recognition. The robot arm is programmed to perform actions based on what it detects, such as:

Waving when a person is identified.

Attempting to grab a bottle upon detection.

### 3. Encoder Values
Monitor the precise angular positions of the robot arm's joints. Communication with the AS5600 magnetic encoders presented a unique challenge due to incompatible standard packages. A custom solution was engineered to interpret I2C signals. With four magnetic encoders and limited I2C pins on the RAMPS hat, a multiplexer chip was essential. This chip acts as a traffic controller, directing signals between multiple I2C ports while maintaining a common ground for reliability. The system automatically pulls raw values from each encoder, alongside distance data from two TOF (Time-of-Flight) sensors (one via I2C, the other via UART).

### 4. Work in Progress Panel (3D Model Visualization)
This exciting section is dedicated to real-time 3D visualization of the robot arm. A 3D model, developed in a separate repository, mirrors the physical movements of the arm, providing a crucial visual representation. While still undergoing fine-tuning for precise articulation, future updates will include live construction of objects being grabbed and handled within this 3D environment.

### 5. Activity Window
Get real-time feedback on the robot's current actions. This feature is fully functional when an older software version runs directly on the Raspberry Pi, offering immediate insights into the robot's operations. Remote access and control for this feature are planned for future development.

## Hardware
The physical construction of the robot arm is designed for extensibility, though current iterations are subject to ongoing improvements for robustness and wire management. The core design integrates several existing open-source models with custom modifications.

### Key Components & Assembly Notes:
Main Robot Arm Structure: The primary structure, while not an original design, has been significantly altered to accommodate specific components and wiring. Building the main arm requires careful attention and "elbow grease."

Raspberry Pi Case: This is also a modified existing design, adapted to house the Raspberry Pi and its associated components, including a custom heat sink.

Lazy Susan Base: This component allows for the arm's rotational movement.

It accommodates a dedicated motor at the bottom.

Bearings are placed within specific ridges for smooth rotation.

The three-layer planetary gear model (not the four-layer) is used for the base and customized to attach the magnet for the encoder. Crucially, the magnet's North and South poles must face side-by-side for correct magnetic encoder readings.

Super glue is recommended for securing the magnet.

Each planetary gear ring needs to be 3D printed specifically (e.g., ring 2 for layer 2, ring 3 for layer 3, etc.) for proper fit.

Magnetic Encoders (AS5600): These are strategically placed at each joint, with the chip facing forward. Custom slots allow for pins or direct soldering of wires.

TOF Sensors: Two Time-of-Flight sensors are integrated, one at the gripper (using smaller screws for attachment) and another on the camera module.

Gripper: The gripper design incorporates a specific material for effective grasping.

Hard material can be used for the main gripper structure.

TPU (Thermoplastic Polyurethane) is essential for the gripper "fingers" to enable effective grasping.

Servos: Three servo motors are used for the camera's movement and one for the gripper mechanism. No additional attachments that come with the servos are needed.

Arduino Mega with RAMPS Hat: This combination is central to controlling the motors and other peripherals. The case design allows for a secure click-in fit.

A fan for the RAMPS hat is highly recommended due to potential heat issues with motor controllers. It's advised to use blue motor controllers as they have proven more durable than green ones.

Power Supply: The system is powered by USBC. A small, custom-designed chip is screwed into the case to manage power, and super glue can be used if parts break off.

Wiring: Cable management is a known area for improvement in the current design. Rubber bands can be used to group wires. Ensure the hole for wires on each arm segment faces backward to streamline cable routing.

Raspberry Pi Camera Module (Pi Camera Noir 3 recommended): The camera is positioned at the front of the arm. Be mindful of the TOF sensor potentially pushing against the camera. A solution involves screwing the camera mounting screws in deeper on all sides to create a space between the TOF sensor and the camera.

![Robot arm ai wireing guide_250703_182612.pdf](https://github.com/Omart19/AI-Robot-Arm/blob/c01de4e7e4336e17fcfd561a30be6d7fbdb2581e/Robot%20arm%20ai%20wireing%20guide_250703_182612.pdf)

## Getting Started
To get your AI Robot Arm up and running:

Assemble the Hardware: Follow the hardware assembly instructions, paying close attention to the planetary gear configurations, magnet orientations, and wiring.

Power On: Always power on the Arduino first BEFORE connecting it to the Raspberry Pi via USB. This prevents the Arduino from attempting to draw power from the Raspberry Pi's USB port, which can lead to issues.

Software Setup: Clone this repository to your Raspberry Pi. Detailed instructions for setting up the Avalonia UI framework, C# and Python dependencies, and configuring the camera and encoder scripts will be provided in future documentation.

Future Development
This project is a continuous work in progress, with exciting plans for future enhancements:

Improved Arm Design: A complete redesign of the physical arm is planned to enhance stability, improve cable management, and address current limitations.

Live Object Construction in 3D: Integrate the ability to dynamically render objects the robot interacts with within the 3D visualization.

Comprehensive Database: Implement a database for the "Tool for Tab" section to log all objects and people the robot interacts with, tracking last seen dates and storage locations.

Remote Access for Activity Window: Enable remote monitoring and control of the robot's activities.

AR-based UI for Workstations and Home: Explore the development of a groundbreaking Augmented Reality (AR) user interface, leveraging AR glasses and custom gauntlets for hand motion tracking. This UI aims to provide an immersive workspace for problem-solving and organization, integrating standard computer functions within an AR environment.

We welcome contributions and feedback to help refine and expand the capabilities of this AI Robot Arm. Stay tuned for more updates!

## Video Links:
### Software:
https://youtu.be/Ny3csvNJm78

### Hardware:
https://youtu.be/vSIQqDmVzqs

## Items:
Steppers: https://a.co/d/iiJf8Wz

Stepper drivers: https://a.co/d/dlJRYvP

55mm screws: https://a.co/d/g2Emexe

55mm screw nut: https://a.co/d/gLMvXZo

m3 screw set: https://a.co/d/0gE9JI0

arduino mega: https://a.co/d/bND0tAM

ramps hat: https://a.co/d/dDMKtNR

magnetic encoders: https://a.co/d/0Xh1Rho

TOF sensor: https://a.co/d/dJkkkVE

i2c multiplexer: https://a.co/d/6HLA56E

16 peice servo motor set: https://a.co/d/f1CV0Sq

lube for bearings: https://a.co/d/1YG2ZDC

bearings: https://a.co/d/5AxLEhn

type c fast charge decoy: https://a.co/d/0HkhXqW

raspberrypi: https://a.co/d/drlnI4A

raspberrypi camera module: https://a.co/d/98rBo57
