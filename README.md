# SensorTile as Unity Game Controller

## Goal of the Project

The motivation of this project is to apply to build a hand-free gaming controller. Players just need to wear a wearable device, e.g. smart watch, to play games using hand gestures or body movement. Another application can be human activity tracking and visualization in 3D game engine such as Unity.

This project aims to integrate SensorTile(s) as a game controller in Unity game engine. SensorTile’s micro-accelerometer, micro-gyroscope, magnetometer can be combined to provide operation of a basic game controller. The communication between SensorTile is done via serial communication either via USB connection or Bluetooth connection.

There are two types of input for this controller:

1. Position / orientation: these information can be captured directly from SensorTiles's  micro-accelerometer, micro-gyroscope, magnetometer.

2. Buttons: Button trigger can be captured as gestures. For a basic game controller, 2 types of gestures are detected to mimic trigger of a primary button and a secondary button. Proposed gesture could be "shaking" gestures, for example, "up-and-down" shaking and "left-to-right" shaking  (assume that players holding SensorTile in their hand).

To acquire (2), data collected from SensorTile while performs two distinct shaking gestures is used to build a supervised ML model that is able to these  gestures.

## Software and Developing Tools

* System Workbench for STM32
* Unity 2019.4 (https://unity3d.com/get-unity/update)
* Ardity (https://ardity.dwilches.com) plugin for receiving COM communication in Unity. 
* Python's Scikit-learn

## Hardware Used

* SensorTile x1
* Nucleo-64 Board x1
