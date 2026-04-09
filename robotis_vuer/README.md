# ROBOTIS Vuer Overview

**ROBOTIS Vuer** for AI Worker lets you view a 3D scene on a **Meta Quest 3** headset and interact with the robot using hand tracking and related input. A browser-based VR client runs together with the **ROS 2** stack on the robot side.

## Stack summary

| Component | Description |
|-----------|-------------|
| **Headset** | Meta Quest 3 |
| **VR client** | [Vuer](https://github.com/vuer-ai/vuer)-based web app (WebXR). On the headset, open the page in the browser (or built-in browser) to start the VR session. |
| **Vuer version** | **v0.1.5** (version used and validated with AI Worker). Other versions may behave differently. [Official docs](https://docs.vuer.ai) |
| **Robot / PC** | ROS 2 nodes and applications connect to the Vuer server over **WebSocket**, exchanging pose, visualization, and control data. |

In short: **Quest 3 → (HTTPS/WSS) → Vuer** is the user-facing path, and **Vuer ↔ ROS 2** carries robot control and state.

## What is Vuer?

**Vuer** is a **Python** toolkit for **3D visualization and interaction** in the browser, aimed at robotics and VR. The server defines the scene (meshes, frames, markers, etc.) and events; the client renders with **WebXR** on the headset and sends hand and controller input back to the server.

- **Role**: Acts as the in-browser VR viewer and a **bidirectional bridge** to the robot PC. A typical pattern is ROS 2 nodes running alongside the Vuer server, linked via **WebSocket (`wss://`)**.
- **Why HTTPS/WSS**: WebXR and device APIs expect a **secure context**, so setups often use **HTTPS** and a **secure WebSocket** even on a local network.
- **Robotics**: Suited to robot models (e.g. URDF), live poses and sensor data, and teleoperation-style UIs. AI Worker aligns Quest 3 visuals and input with ROS 2 logic on this path.
- **Version in AI Worker**: Packages and Docker images target **Vuer v0.1.5**. Newer releases may change APIs or behavior; when debugging, compare against v0.1.5.

Official docs: [docs.vuer.ai](https://docs.vuer.ai) (may reflect newer releases than v0.1.5) · Source: [github.com/vuer-ai/vuer](https://github.com/vuer-ai/vuer)
