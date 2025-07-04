# URDF Viewer with MediaPipe Teleoperation and IK

[![Deploy with Vercel](https://vercel.com/button)](https://vercel.com/new/clone?repository-url=https://github.com/atharvasakpal/urdf_visualizer)
[![Live Demo](https://img.shields.io/badge/Live%20Demo-robotmanipulator.vercel.app-blue)](https://robotmanipulator.vercel.app)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

> **An interactive browser-based platform for visualizing URDF robots with real-time teleoperation using MediaPipe and inverse kinematics support.**

## 🎯 Overview

This project provides a comprehensive web-based solution for robotics visualization and control, combining URDF parsing, 3D visualization, real-time pose detection, and inverse kinematics in a single platform. Whether you're a robotics researcher, student, or enthusiast, this tool enables you to load any URDF-based robot model and interact with it through multiple control methods.

### 📸 Screenshots

| Main Interface | Teleoperation | Inverse Kinematics |
|----------------|---------------|-------------------|
| ![Main Interface](./images/main-interface.png) | ![Teleoperation](./images/teleoperation-demo.gif) | ![IK Solver](./images/ik-demo.gif) |

> **📹 [Watch Full Demo Video](https://your-demo-video-link.com)**

## ✨ Key Features

### 🤖 Universal URDF Support
- **Drag-and-drop** loading for `.urdf`, `.dae`, and `.stl` files
- **Automatic parsing** of robot kinematic structures
- **Multi-format mesh support** (Collada DAE, STL)
- **Real-time visualization** updates in Three.js

### 🎮 Multiple Control Methods
- **Manual Control**: Interactive sliders for all joint angles
- **Teleoperation**: Real-time webcam-based control using MediaPipe
- **Inverse Kinematics**: Target-based positioning with automatic joint computation
- **Video Playback**: Control robots using pre-recorded motion videos

### 🔧 Advanced Visualization
- **3D Scene Navigation**: Mouse-based camera controls (rotate, zoom, pan)
- **Visual Debugging**: Toggle joint axes, coordinate frames, and wireframes
- **Grid and Lighting**: Customizable scene environment
- **Auto-rotation**: Automatic model rotation for presentation

### 📱 Real-time Pose Detection
- **MediaPipe Integration**: Pose and hand landmark detection
- **Webcam Support**: Live camera feed processing
- **Video File Support**: Upload and process motion videos
- **Optimized Performance**: Efficient landmark tracking and mapping

## 🚀 Live Demo

Experience the visualizer at: **[robotmanipulator.vercel.app](https://robotmanipulator.vercel.app)**

## 📋 Supported Robot Models

### ✅ Full Teleoperation Support

#### Poppy Humanoid
![Poppy Humanoid](./images/poppy-humanoid-demo.gif)
- **Complete body pose mapping**
- **Real-time joint control**
- **Natural gesture recognition**

#### Allegro Hand
![Allegro Hand](./images/allegro-hand-demo.gif)
- **Finger and hand gesture control**
- **Precision grip simulation**
- **Hand landmark tracking**

### 🔧 Manual Control + IK Support
![Various Robot Models](./images/robot-models-showcase.png)
- **Universal**: All URDF-compliant robot models
- **Robotic Arms**: Manipulator arms with joint control
- **Mobile Robots**: Wheeled and tracked vehicles
- **Custom Models**: Any valid URDF specification

## 🛠️ Technology Stack

| Component | Technology | Purpose |
|-----------|------------|---------|
| **3D Rendering** | Three.js | WebGL-based 3D visualization |
| **Pose Detection** | MediaPipe | Real-time landmark detection |
| **Build Tool** | Vite | Fast development and bundling |
| **Deployment** | Vercel | Serverless hosting platform |
| **Language** | Vanilla JavaScript | Core application logic |
| **Kinematics** | Custom IK Solver | Inverse kinematics computation |

## 🔧 Installation & Setup

### Prerequisites
- **Node.js** (v16 or higher)
- **npm** or **yarn**
- **Modern web browser** (Chrome, Firefox, Safari, Edge)

### Local Development

```bash
# Clone the repository
git clone https://github.com/atharvasakpal/urdf_visualizer.git
cd urdf_visualizer

# Install dependencies
npm install

# Start development server
npm run dev

# Build for production
npm run build

# Preview production build
npm run preview
```

### Quick Start
1. **Access the application** at `http://localhost:5173`
2. **Load a URDF file** by dragging and dropping it into the browser
3. **Choose control method**:
   - Use sliders for manual control
   - Enable webcam for teleoperation (supported models only)
   - Set IK targets for automatic positioning

## 📁 Project Structure

```
urdf_visualizer/
├── src/
│   ├── js/
│   │   ├── urdf-parser.js      # URDF file parsing logic
│   │   ├── three-scene.js      # Three.js scene management
│   │   ├── mediapipe-handler.js # Pose detection integration
│   │   ├── ik-solver.js        # Inverse kinematics algorithms
│   │   └── robot-controllers/  # Robot-specific control mappings
│   │       ├── poppy-humanoid.js
│   │       └── allegro-hand.js
│   ├── css/
│   │   └── styles.css          # Application styling
│   └── assets/
│       └── models/             # Sample URDF models
├── public/
│   └── index.html              # Main HTML file
├── package.json                # Dependencies and scripts
└── vite.config.js             # Vite configuration
```

## 🎮 Usage Guide

### Loading Robot Models

![Drag and Drop Demo](./images/drag-drop-demo.gif)

1. **Drag and Drop Method**:
   - Drag `.urdf`, `.dae`, or `.stl` files directly into the browser
   - The parser will automatically process and display the robot

2. **File Selection**:
   - Click the "Load URDF" button
   - Select your robot model files
   - Wait for parsing and visualization

### Control Methods

#### Manual Control
![Manual Control Interface](./images/manual-control.png)
- **Joint Sliders**: Adjust individual joint angles
- **Real-time Updates**: See changes instantly in 3D view
- **Range Limits**: Automatic joint limit enforcement

#### Teleoperation (Supported Models)
![Teleoperation Setup](./images/teleoperation-setup.png)
1. **Enable Camera**: Click "Start Teleoperation"
2. **Grant Permissions**: Allow browser camera access
3. **Pose Detection**: Position yourself in camera view
4. **Real-time Control**: Your movements control the robot

#### Inverse Kinematics
![IK Target Setting](./images/ik-target-demo.gif)
1. **Set Target**: Click in 3D space or use coordinate inputs
2. **Compute IK**: Algorithm calculates required joint angles
3. **Smooth Animation**: Watch robot move to target position

### Visualization Options

| Control | Function |
|---------|----------|
| **Mouse Drag** | Rotate camera around model |
| **Mouse Wheel** | Zoom in/out |
| **Right Click + Drag** | Pan camera |
| **Joint Axes Toggle** | Show/hide joint coordinate frames |
| **Wireframe Mode** | Display model structure |
| **Grid Toggle** | Show/hide reference grid |
| **Auto-rotate** | Automatic model rotation |


## 🙏 Acknowledgments

- **Three.js** community for 3D rendering capabilities
- **MediaPipe** team for pose detection technology
- **ROS** community for URDF standards
- **Vercel** for deployment platform
- **Open robotics** community for sample models

## 📞 Support

- **GitHub Issues**: [Report bugs or request features](https://github.com/atharvasakpal/urdf_visualizer/issues)
- **Discussions**: [Community discussions](https://github.com/atharvasakpal/urdf_visualizer/discussions)
- **Email**: [Contact maintainer](mailto:your-email@example.com)

## 🔮 Roadmap

- [ ] **ROS Integration**: Direct ROS topic publishing
- [ ] **Physics Simulation**: Collision detection and dynamics
- [ ] **Multi-robot Support**: Control multiple robots simultaneously
- [ ] **AR/VR Support**: Extended reality interfaces
- [ ] **Cloud Deployment**: Robot control from anywhere
- [ ] **Mobile App**: Native mobile applications
- [ ] **Advanced IK**: Neural network-based solvers

---

**Built with ❤️ for the robotics community**
