#path_planning
#motion_planning
#Camera_calibration
#Simulation
#PID
#YOLO
#ROS2 
# Requirements
- [ ] Omniverse **Isaac Sim 4.2.0** 
- [ ] Ubuntu **22.04
- [ ] GPU **RTX3080+
- [ ] ROS2 **Humble
- [ ] Ultralytics **YoloV8
- [ ] Pytorch **2.7.0**
- [ ] CUDA **12.6**
- [ ] Anylabeling(latest)

## Overview
----
- Simulation 
- Ackerman Steering and Forward Kinematics for Car-Like Vehicles
- Camera Calibration 
- YOLO
	- Line segmentation 
- Path planning
	- Curve fitting 
- Motion planning
	- PID controller
	- NAV2
# Simulation 
#asphalt_finisher 3D model is in Isaac sim. We have added some custom materials. So we should contain all these folder. 
If you need to do #Camera_calibration use checker board.

![[Pasted image 20250509100611.png]]
#asphalt_finisher 3D in omniverse.
![[Pasted image 20250509102824.png]]

# Ackerman Steering and Forward Kinematics for Car-Like Vehicles 

![[Pasted image 20250508175244.png]]

## Ackerman Steering Geometry

Ackerman steering is used in #asphalt_finisher vehicles kinematic to reduce tire slippage during turns by rotating the inner wheel at a sharper angle than the outer wheel. And it is helps to following the path.

### Definitions

- **Track width**: $w$ – lateral separation between wheels
    
- **Wheelbase**: $l$ – longitudinal separation between front and rear wheels
    
- **$\phi_i$** – inner wheel steering angle
    
- **$\phi_o$** – outer wheel steering angle
    
- **$r$** – distance from vehicle center to Instantaneous Center of Curvature (ICC)
    

### Steering Triangle Relationships

From geometry:

$$\tan \phi = \frac{l}{r}, \quad 
\tan \phi_i = \frac{l}{r - \frac{w}{2}}, \quad 
\tan \phi_o = \frac{l}{r + \frac{w}{2}}$$
```python
def compute_inner_outer_steering(self, phi):
	# Handle small angles to avoid division by zero
	if abs(phi) < 1e-6:
		return 0.0, 0.0

	num = 2 * self.l * math.sin(phi)
	den_i = 2 * self.l * math.cos(phi) - self.w * math.sin(phi)
	den_o = 2 * self.l * math.cos(phi) + self.w * math.sin(phi)

	phi_i = math.atan(num / den_i)
	phi_o = math.atan(num / den_o)
	return phi_i, phi_o
```

### Ackerman Steering Equation

Subtracting reciprocals:

$$\frac{1}{\tan \phi_o} - \frac{1}{\tan \phi_i} = \cot \phi_o - \cot \phi_i = \frac{l}{r + \frac{w}{2}} - \frac{l}{r - \frac{w}{2}} = \frac{lw}{r^2 - \left(\frac{w}{2}\right)^2} \approx \frac{w}{l}$$

### Reformulation with Base Angle **$\phi$**

Using a common steering base angle **$\phi$**:

$$
\cot \phi_i = \cot \phi - \frac{w}{2l}, \quad
\cot \phi_o = \cot \phi + \frac{w}{2l}
$$


### Handling **$\phi=0$** 

Since $\cot(0)$ is undefined, we can reformulate using:

$$\cot \alpha = \frac{\cos \alpha}{\sin \alpha}$$

So:
$$\phi_i = \tan^{-1} \left( \frac{2l \sin \phi}{2l \cos \phi - w \sin \phi} \right), \quad \phi_o = \tan^{-1} \left( \frac{2l \sin \phi}{2l \cos \phi + w \sin \phi} \right)$$

---

## Forward Kinematics for Car-Like Vehicles

The forward kinematics describes the vehicle's future state given its current configuration and speed.

### State Representation

$$(x, y, \theta, \phi)$$

- $x, y$ – position in global frame
    
- $\theta$ – heading angle
    
- $\phi$ – steering angle
    
- $s$ – vehicle speed
    

### Kinematic Equations

$$\dot{x} = s \cos \theta$$$$ \dot{y} = s \sin \theta $$$$\dot{\theta} = \frac{s}{l} \tan \phi \approx \frac{s}{l} \phi$$
```python
# Forward kinematics using base angle
	dx = speed * math.cos(self.pose.theta) * dt
	dy = speed * math.sin(self.pose.theta) * dt
	dtheta = (speed / self.l) * math.tan(phi) * dt

	# Update pose
	self.pose.x += dx
	self.pose.y += dy
	self.pose.theta += dtheta
```
> Note: The small-angle approximation $\tan \phi \approx \phi$ holds with acceptable error (≈ 3° at 30° steering lock).

---

## Full Implementation 

```python
# The real state of Asphalt finisher position and angle values. 
def ackermann_callback(self, msg: AckermannDriveStamped):
	current_time = self.get_clock().now()
	dt = (current_time - self.last_time).nanoseconds * 1e-9
	self.last_time = current_time
	
	speed = msg.drive.speed
	phi = msg.drive.steering_angle  # base (middle) steering angle
	
	# Calculate inner and outer steering angles
	phi_i, phi_o = self.compute_inner_outer_steering(phi)

	# Forward kinematics using base angle
	dx = speed * math.cos(self.pose.theta) * dt
	dy = speed * math.sin(self.pose.theta) * dt
	dtheta = (speed / self.l) * math.tan(phi) * dt

	# Update pose
	self.pose.x += dx
	self.pose.y += dy
	self.pose.theta += dtheta

	# Publish pose
	self.publisher.publish(self.pose)

	# Logging
	self.get_logger().info(
		f"x: {self.pose.x:.2f}, y: {self.pose.y:.2f}, θ: {math.degrees(self.pose.theta):.2f}° | "
		f"ϕ: {math.degrees(phi):.2f}°, ϕᵢ: {math.degrees(phi_i):.2f}°, ϕₒ: {math.degrees(phi_o):.2f}°"
	)

def compute_inner_outer_steering(self, phi):
	# Handle small angles to avoid division by zero
	if abs(phi) < 1e-6:
		return 0.0, 0.0

	num = 2 * self.l * math.sin(phi)
	den_i = 2 * self.l * math.cos(phi) - self.w * math.sin(phi)
	den_o = 2 * self.l * math.cos(phi) + self.w * math.sin(phi)

	phi_i = math.atan(num / den_i)
	phi_o = math.atan(num / den_o)
	return phi_i, phi_o
```

# Camera calibration and pose estimation from **Image**
We need pose of 
### Goal 
- All camera has distortion from the lens we need to remove them. So We need to take picture of chess board. Take around 20-30 images are near and close.
![[Pasted image 20250509105556.png]]
Now take a picures in Isaac sim . Save all images into 1 folder. 
![[Pasted image 20250509104837.png]]

#### Calibration measurement 
You need define camera location in real world coordinate.
- [ ] Take images, save in same folder.
- [ ] Rename a images same format. **image-1.jpg, image-2.jpg**
- [ ] Measurement of 4 points. It can be 4 corner of chess board. **($z=0$)**
- [ ] Make `.csv` file.
	`x_i , y_i` = image pixel of x, y.
	`xw, yw, zw` = world coordinate measurement 0.01 = 1m.
```c 
Names,xi,yi,xw,yw,zw
image1,1025,551,0,0,0
image1,1093,518,0.5,0,0
image1,1025,551,0,0,0
image1,1093,518,0.5,0,0
```

# YOLO semantic segmentation
## Data set Structure 

> [!WARNING]
> Extracted images must save in `images` folder!

```
data_source/ 
├── images/ 
│   ├── folder_1/ 
│   │   ├── {folder_1}_frame_00000.png
│   │   ├── {folder_1}_frame_00000.json
│   │   └── ... 
│   ├── folder_2/ 
│   │   ├── {folder_2}_frame_00000.png 
│   │   └── {folder_2}_frame_00000.json 
│   │   └── ... 
│   └── synthetic/ 
│   |   ├── {synthetic}_frame_00000.png 
│   |   └── {synthetic}_frame_00000.json 
│   │   └── ... 
└── videos/ 
	├── {recieved_date}_teams/
	│   └── {folder_1}.mp4 
	└── {recieved_date}_aws/ 
		└── {folder_2}.mp4
```
### Labeling 
You need install **[Anylabeling](https://github.com/vietanhdev/anylabeling.git)**. 
- [ ] Choose `Polygon` or press `P`.
- [ ] Label the objects.
>[!IMPORTANT]
>Label the each line. Then labeling together but name is same class name. Both are `line`.
>![[Pasted image 20250509163829.png]]
![[Pasted image 20250509164123.png]]

| **class name list** |
| :-----------------: |
|        line         |
|   concrate_guide    |
|    wooden_guide     |

### Training 
- [ ] Prepare a dataset like this structure.
```
data  
├── train  
│   ├── image1.png 
│   └── image1.txt 
└── val
	├── image_test1.png 
	└── image_test2.txt
```
- [ ] Create a `train.py` be tuned
```python 
from ultralytics import YOLO
# Load a model
model = YOLO("yolov8n-seg.pt")  # load a pretrained model (if you aren't installed it is automatically install)

# Train using the single most idle GPU
results = model.train(data="data.yaml", epochs=50, imgsz=640, device=-1)
```
The`data.yaml` is routing the training and validation images folder. 
```yaml
train: './data/train'
val: './data/val'
nc: 3
names: ['line', 'concrate_guide''wooden_guide']
```

# Path planning
We are using the segmentation model to predict the lines. Then We get the nearest point of the segmentation result. This method is helping us to back wheel path. Because back wheel must parallel to the path. The path can curve or line anyway. But we going to calculate the parallel point of back wheel what point of derivative tangent. It give us back wheel error $\theta$
value that back wheel direction. 