# SensorTile as Unity Game Controller

# Final Project Submission

* Final Report: [FinalReport.pdf](FinalReport.pdf)

* Slides: [FinalPresentation.pdf](FinalPresentation.pdf)

* Live Demo: https://youtu.be/VUMhXRM9ZEg

* Code: This project includes 3 sub-projects:
  * [STile_Rotation](STile_Rotation)
  * [beaglebone/sensortile](beaglebone/sensortile)
  * [SensorTileController](SensorTileController)

* Data:
  * [training.csv](data/training.csv)
  * [testing.csv](data/testing.csv)

## Goal of the Project

The motivation of this project is to apply to build a hand-free gaming controller. Players just need to wear a wearable device, e.g. smart watch, to play games using hand gestures or body movement. Another application can be human activity tracking and visualization in 3D game engine such as Unity.

In this project, SensorTile is used as the game controller that allow players to perform some motion patterns that are detected by an embedded machine learning (ML) model running on the SensorTile’s processor. It allows decent machine learning technique running on a lightweight device and works independently to a target application. SensorTile detects user’s gesture and send the result to the integrated game engine via Bluetooth Low Energy and Wifi connection.

[Demo Video](https://youtu.be/VUMhXRM9ZEg)

## Project Code

This Github repository of this project contains 3 sub-projects including:

### `STile_Rotation`

Implementation of SensorTile firmware. This project extends on the project provided in Tutorial 13 to update the program flow and add BLE communication with a connected device, e.i. BeagleBone:

  * Receive command to for training step.
  * Send detected motion in running step

### `beaglebone/sensortile`

This NodeJS project implements a server program running on a BeagleBone. This program provides the connectivity between a client program and SensorTile by:

* Send and receive data to SensorTile via BLE connection. It is done by invoking gatttool installed on the operating system of BeagleBone.

* Host a Websocket server to allow connection from a Websocket client, e.g. the demo Unity program, forward request and data received from SensorTile.

### `SensorTileController`

A Unity project implements of the demo game scene in a Unity project

## Software and Developing Tools

* System Workbench for STM32
* Embedded ML framework (https://github.com/merrick7/EmbeddedML)
* Unity 2019.4 (https://unity3d.com/get-unity/update)
* WebSocket library for Unity (https://github.com/endel/NativeWebSocket)
* Node.js developement enviroment (Running on BeagleBone)

## Hardware Used

* SensorTile x1
* Nucleo-64 Board x1
* BeagleBone Wireless x1

