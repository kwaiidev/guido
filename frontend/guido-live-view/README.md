# Guido Live View

React + Tailwind frontend for the Guido ROS telemetry bridge.

## Run

Start the ROS bridge:

```bash
source /opt/ros/humble/setup.bash
source /Users/carlos/Development/Guido/install/local_setup.bash
ros2 launch guido_bringup guido_frontend_bridge.launch.py
```

Then start the frontend:

```bash
cd /Users/carlos/Development/Guido/frontend/guido-live-view
npm install
npm run dev
```

The app expects the bridge on `http://localhost:8765` by default. Override it with:

```bash
VITE_TELEMETRY_BASE=http://<jetson-ip>:8765 npm run dev
```
