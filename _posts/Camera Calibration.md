# Camera Calibration Documentation

## Goal
Predict pixel coordinates of *line*, *wooden_guide*, and *concrete_guide* into real-world coordinates using camera calibration.

## Requirements
- 🔗 **GitHub Repo**: [Camera_calibration](git@github.com:miraitechnologies/sv_af_codes.git) to access the code.
- OpenCV-Python
- Numpy
- Chessboard pattern (7x9 squares)
- Web camera
- Meter-scale measurement tool

---

## Intrinsic Camera Calibration

### Process Flow
1. **Define 3D Coordinates**  
   Create world coordinates for chessboard corners (7x9 grid, 2.2cm square size).
![[Pasted image 20250508153153.png]]

3. **Capture Multiple Views**  
   Collect chessboard images from different angles (saved in `./synthetic data/calibration measurement/`).

4. **Corner Detection**  
   Use OpenCV's `findChessboardCorners` to detect (u,v) pixel coordinates:
   ```python
   ret, corners = cv2.findChessboardCorners(grayColor, CHECKERBOARD, flags)
   ```

5. **Refine Corners**  
   Sub-pixel refinement with `cornerSubPix`:
   ```python
   corners2 = cv2.cornerSubPix(grayColor, corners, (11,11), (-1,-1), criteria)
   ```

6. **Calibration**  
   Compute camera parameters via `calibrateCamera`:
   ```python
   ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(threedpoints, twodpoints, gray.shape[::-1], None, None)
   ```

### Key Outputs
**Camera Matrix**  
```python
[[ 36.26    0.    125.69]
 [  0.     36.77  142.50]
 [  0.      0.      1.  ]]
```

**Distortion Coefficients**  
```python
[-0.00125  0.000099 -0.00289  0.000453 -0.0000033]
```

**Undistortion Example**  
```python
undst = cv2.remap(image, map1, map2, cv2.INTER_LINEAR)
```

---

## Extrinsic Calibration

### Setup
- Chessboard placed in world coordinates (origin at left-bottom corner) like this.
![[Pasted image 20250508153722.png]]
- CSV file maps 3D points (x,y,z) to 2D pixels (u,v):
  ```c
  Names,xi,yi,xw,yw,zw
  pt1,1025,551,0,0,0
  pt2,1093,518,0.5,0,0
  ...
  ```
![[Pasted image 20250508153736.png]]

### Calibration Process
1. **Load Calibration Data**  
   ```python
   camera_matrix = np.load("camera_matrix.npy")
   dist_coeff = np.load("distortion_coeff.npy")
   ```

2. **Solve PnP**  
   Compute camera pose relative to world coordinates:
   ```python
   ret, rvec, tvec = cv2.solvePnP(worldPoints, imagePoints, camera_matrix, dist_coeff)
   ```

### Key Outputs
**Rotation Vector**  
```python
[-0.0577, 0.0355, 1.5091]
```

**Translation Vector**  
```python
[4.6305, -3.7428, 1.6424]
```

**Projection Matrix**  
```
[36.26 0 125.69 0;
 0 36.77 142.50 0;
 0 0 1 0] × [R|t]
```

---

## World Coordinate Conversion

### Pixel → World Transformation
```python
def img2world(u, v):
    # 1. Convert to normalized coordinates
    uv_1 = np.array([[u, v, 1]]).T
    suv_1 = scaling_factor * uv_1
    xyz_c = inv(camera_matrix) @ suv_1 - tvec
    
    # 2. Apply inverse rotation
    XYZ = inv(R_matrix) @ xyz_c
    
    # 3. Solve for Z=0 plane
    Zc = 0
    eq = (Zc - A_point[2]) / (B_point[2] - A_point[2])
    Xc = eq*(B_point[0]-A_point[0]) + A_point[0]
    Yc = eq*(B_point[1]-A_point[1]) + A_point[1]
    return [Xc, Yc, Zc]
```

### Example Conversion

**Input Pixel**  
`u=1025, v=551`

**World Coordinates**  
```
X = 0.0 m  (ground truth matches CSV)
Y = 0.0 m
Z = 0.0 m
```

---

## Validation
1. **Reprojection Error**  
   Check chessboard corners reproject accurately onto images.

2. **Physical Measurement Check**  
   Verify real-world distances between predicted points match meter measurements.

3. **Z=0 Assumption**  
   All converted points lie on ground plane (Z=0) by design.

---

## Conclusion
This pipeline enables accurate conversion of detected pixels (lines/guides) to real-world coordinates through:
1. Precise intrinsic calibration removing lens distortions
2. Extrinsic calibration positioning camera in world space
3. Geometric projection using inverse camera matrices

Output matrices (`camera_matrix.npy`, `rvec.npy`, etc.) are saved for integration with detection algorithms.