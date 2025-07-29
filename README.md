# FOD Detection and Measurement System

An IoT-based embedded system for detecting, classifying, and estimating the size of Foreign Object Debris (FOD) on airport runways using AI and computer vision, with the capability of integration into autonomous vehicles.

<p align="center">
  <img src="https://github.com/Hiepnguyenhoang/FOD/blob/calc-size/Result/Rover.jpg?raw=true" alt="Rover Image" width="550"/>
</p>

## 📌 Overview

This project proposes an end-to-end system that integrates:
- **YOLOv11** for object detection
- **LiDAR** for measuring distance between rover and FOD
- **Camera calibration & Canny edge detection** for real-world size estimation
- All deployed on **NVIDIA Jetson Nano** using GPU for edge computing.

The system is designed for smart airports and can be integrated with robotic arms or rovers for autonomous FOD collection.

## 📦 Dataset

![FOD dataset](https://github.com/Hiepnguyenhoang/FOD/blob/calc-size/Result/FOD_example.png?raw=true)
- 10-class FOD dataset
- 21,194 images (collected and augmented)
- Sources: Roboflow, Kaggle, Google Schoolar, Github, Self-collected

## 📷 System Architecture

<p align="center">
  <img src="https://github.com/Hiepnguyenhoang/FOD/blob/calc-size/Result/System.png?raw=true" alt="Rover Image" width="550"/>
</p>

## 📱 App

To visualize how the model detects and measures the size of FOD, I provide a simple **Streamlit-based application** that runs locally. This app allows users to observe detection results, FOD size estimation.

The app source code is located in the [`App/`](https://github.com/Hiepnguyenhoang/FOD/tree/calc-size/App) directory.

```powershell
$env:Path = "C:\Users\My PC\AppData\Local\Programs\Python\Python312\" + ";" + $env:Path
streamlit run app.py
```
![FOD dataset](https://github.com/Hiepnguyenhoang/FOD/blob/calc-size/Result/Streamlit.png?raw=true)
