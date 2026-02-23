# Far-Field Accurate Lane Detection System Architecture
## Production-Grade ADAS Lane Detection for CARLA

**Author:** Senior Autonomous Driving Engineer  
**Date:** 2025  
**Version:** 1.0  
**Status:** Production-Ready Design

---

## 📐 SYSTEM ARCHITECTURE OVERVIEW

```
┌─────────────────────────────────────────────────────────────────┐
│                    INPUT: RGB Camera Frame (640x480)            │
└────────────────────────────┬────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│  PHASE 1: RAW DETECTION                                          │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ Ultra-Fast-Lane-Detection-v2 Model                       │   │
│  │ Output: Lane coordinates (pixel space)                   │   │
│  └────────────────────────┬─────────────────────────────────┘   │
└────────────────────────────┼────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│  PHASE 2: PERSPECTIVE TRANSFORMATION (IPM)                       │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ Homography Estimation                                     │   │
│  │ H = [h11 h12 h13; h21 h22 h23; h31 h32 h33]              │   │
│  │ BEV Transformation: [x', y', 1]ᵀ = H · [x, y, 1]ᵀ      │   │
│  └────────────────────────┬─────────────────────────────────┘   │
└────────────────────────────┼────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│  PHASE 3: MULTI-ZONE DETECTION                                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐        │
│  │ NEAR ZONE    │  │ MID ZONE     │  │ FAR ZONE     │        │
│  │ (0-20m)      │  │ (20-50m)     │  │ (50-100m)    │        │
│  │ High conf    │  │ Medium conf  │  │ Low conf     │        │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘        │
└─────────┼─────────────────┼─────────────────┼──────────────────┘
          │                 │                 │
          └─────────────────┼─────────────────┘
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  PHASE 4: ROBUST POLYNOMIAL FITTING                             │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ RANSAC Outlier Rejection                                  │   │
│  │ Weighted Least Squares (WLS)                             │   │
│  │ Distance-aware weighting: w_i = exp(-d_i²/σ²)          │   │
│  │ Output: Polynomial coefficients [a, b, c]                │   │
│  └────────────────────────┬─────────────────────────────────┘   │
└────────────────────────────┼────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│  PHASE 5: TEMPORAL SMOOTHING (KALMAN FILTER)                    │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ State: x_k = [a_k, b_k, c_k]ᵀ (polynomial coeffs)        │   │
│  │ Prediction: x̂_{k|k-1} = F · x_{k-1}                     │   │
│  │ Update: x̂_k = x̂_{k|k-1} + K · (z_k - H·x̂_{k|k-1})      │   │
│  └────────────────────────┬─────────────────────────────────┘   │
└────────────────────────────┼────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│  PHASE 6: REAL-WORLD METRICS                                    │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ Curvature: κ = |2a| / (1 + b²)^(3/2) [1/m]              │   │
│  │ Lateral Offset: Δy = f(y_vehicle) - y_center            │   │
│  │ Lane Width: W = |f_left(y) - f_right(y)|                │   │
│  └────────────────────────┬─────────────────────────────────┘   │
└────────────────────────────┼────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│  PHASE 7: CONFIDENCE & VALIDATION                               │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ Coverage Score: C_cov = N_valid / N_total                │   │
│  │ Stability Score: C_stab = 1 - σ(Δcoeffs)                 │   │
│  │ Symmetry Score: C_sym = 1 - |κ_left - κ_right|           │   │
│  │ Final: C = w1·C_cov + w2·C_stab + w3·C_sym               │   │
│  └────────────────────────┬─────────────────────────────────┘   │
└────────────────────────────┼────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│  PHASE 8: PLANNING-READY OUTPUT                                 │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ Centerline: f_center(y) = (f_left(y) + f_right(y))/2    │   │
│  │ Sampled Spline: [x_i, y_i] for i ∈ [0, N]                │   │
│  │ MPC Format: Waypoints in BEV space                        │   │
│  └────────────────────────┬─────────────────────────────────┘   │
└────────────────────────────┼────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│                    OUTPUT: Lane Centerline + Metrics            │
└─────────────────────────────────────────────────────────────────┘
```

---

## 1️⃣ PERSPECTIVE TRANSFORMATION (IPM / BEV)

### 1.1 Homography Estimation

**Mathematical Foundation:**

The perspective transformation from camera view to Bird's Eye View (BEV) is a 2D homography:

```
[x']   [h11 h12 h13] [x]
[y'] = [h21 h22 h23] [y]
[1 ]   [h31 h32 h33] [1]
```

Where:
- `[x, y]` = pixel coordinates in camera view
- `[x', y']` = coordinates in BEV (meters)
- `H` = 3×3 homography matrix (8 DOF, h33=1)

### 1.2 Homography Computation

**Direct Linear Transform (DLT) Method:**

Given 4 correspondences: `(x_i, y_i) ↔ (x'_i, y'_i)`

For each correspondence:
```
[0  0  0  -x  -y  -1  x·y'  y·y'] [h11]   [y']
[x  y  1   0   0   0  -x·x' -y·x'] [h12] = [-x']
                                  [h13]
                                  [h21]
                                  [h22]
                                  [h23]
                                  [h31]
                                  [h32]
```

Stack all 4 correspondences → Solve `A·h = 0` using SVD.

### 1.3 IPM Matrix for CARLA

**Calibration Parameters:**

```python
# Camera intrinsics (from CARLA config)
focal_length = image_width / (2 * tan(FOV/2))  # pixels
cx, cy = image_width/2, image_height/2

# Camera extrinsics (assumed)
camera_height = 1.4  # meters (above ground)
pitch_angle = 0.0    # radians (looking forward)

# IPM region (ROI in camera view)
roi_top = 0.4 * image_height    # Horizon line
roi_bottom = image_height      # Bottom of image
roi_left = 0.1 * image_width
roi_right = 0.9 * image_width

# BEV region (meters)
bev_x_min, bev_x_max = -3.0, 3.0   # Lateral range
bev_y_min, bev_y_max = 0.0, 100.0  # Forward range
```

**Homography Calculation:**

```python
# Source points (camera view - ROI corners)
src_points = np.array([
    [roi_left, roi_top],      # Top-left
    [roi_right, roi_top],     # Top-right
    [roi_right, roi_bottom],  # Bottom-right
    [roi_left, roi_bottom]    # Bottom-left
], dtype=np.float32)

# Destination points (BEV - meters)
dst_points = np.array([
    [bev_x_min, bev_y_max],   # Far-left
    [bev_x_max, bev_y_max],   # Far-right
    [bev_x_max, bev_y_min],   # Near-right
    [bev_x_min, bev_y_min]    # Near-left
], dtype=np.float32)

# Compute homography
H = cv2.getPerspectiveTransform(src_points, dst_points)
H_inv = np.linalg.inv(H)  # For inverse transformation
```

### 1.4 Why BEV Improves Far-Lane Accuracy

**Problem in Camera View:**
- Far lanes appear compressed (perspective effect)
- Same physical distance = fewer pixels at horizon
- Polynomial fitting becomes unstable

**Solution in BEV:**
- Uniform scale: 1 meter = constant pixel distance
- Far lanes have same resolution as near lanes
- Polynomial fitting is more stable
- Real-world metrics (curvature, width) are direct

**Mathematical Justification:**

In camera view:
```
Pixel density: ρ(y) ∝ 1/y²  (inverse square law)
```

In BEV:
```
Pixel density: ρ(y') = constant  (uniform scale)
```

---

## 2️⃣ ROBUST LANE FITTING

### 2.1 Weighted Least Squares Polynomial Fitting

**Standard Form:**

For lane points `(x_i, y_i)` with weights `w_i`:

Fit polynomial: `x = a·y² + b·y + c`

**Weighted Least Squares:**

```
minimize: Σ w_i · (x_i - a·y_i² - b·y_i - c)²
```

**Matrix Form:**

```
X = [y₁²  y₁  1]     W = diag([w₁, w₂, ..., wₙ])
    [y₂²  y₂  1]     θ = [a, b, c]ᵀ
    [ ...     ]     Y = [x₁, x₂, ..., xₙ]ᵀ
    [yₙ²  yₙ  1]

Solution: θ = (Xᵀ·W·X)⁻¹ · Xᵀ·W·Y
```

### 2.2 Distance-Aware Weighting

**Strategy:**

Points closer to vehicle (lower y) have higher weight:

```
w_i = exp(-d_i² / σ²)
```

Where:
- `d_i` = distance from vehicle (in BEV, meters)
- `σ` = decay parameter (typically 20-30 meters)

**Alternative: Inverse Distance Weighting**

```
w_i = 1 / (1 + α·d_i²)
```

Where `α` controls decay rate.

### 2.3 RANSAC Outlier Rejection

**Algorithm:**

1. **Sample:** Randomly select `s` points (minimum for fitting, e.g., s=3 for quadratic)
2. **Fit:** Fit polynomial to sampled points
3. **Score:** Count inliers (points within threshold `t`)
4. **Repeat:** N iterations
5. **Select:** Best model (most inliers)

**Pseudocode:**

```python
def ransac_polynomial_fitting(points, max_iterations=100, threshold=0.5):
    best_model = None
    best_inliers = []
    best_score = 0
    
    for iteration in range(max_iterations):
        # Sample 3 random points
        sample = random.sample(points, 3)
        
        # Fit polynomial
        coeffs = fit_polynomial(sample)
        
        # Count inliers
        inliers = []
        for point in points:
            error = compute_residual(point, coeffs)
            if error < threshold:
                inliers.append(point)
        
        # Update best model
        if len(inliers) > best_score:
            best_score = len(inliers)
            best_inliers = inliers
            best_model = coeffs
    
    # Refit on all inliers
    if best_inliers:
        best_model = weighted_least_squares(best_inliers, weights)
    
    return best_model, best_inliers
```

### 2.4 Multi-Zone Fitting

**Strategy:**

Fit separate polynomials for near/mid/far zones, then fuse:

```python
# Zone definitions (in BEV, meters)
zones = {
    'near': (0, 20),    # High confidence
    'mid': (20, 50),    # Medium confidence
    'far': (50, 100)    # Low confidence
}

# Fit per zone
polynomials = {}
for zone_name, (y_min, y_max) in zones.items():
    zone_points = filter_points_by_y(points, y_min, y_max)
    if len(zone_points) >= 3:
        polynomials[zone_name] = fit_polynomial(zone_points)

# Fuse polynomials (weighted average at boundaries)
fused_polynomial = fuse_polynomials(polynomials, weights)
```

**Fusion Method (Spline):**

At zone boundaries, use weighted average:

```
f_fused(y) = w_near(y)·f_near(y) + w_mid(y)·f_mid(y) + w_far(y)·f_far(y)
```

Where weights sum to 1 and decay smoothly.

---

## 3️⃣ REAL-WORLD METRICS

### 3.1 Pixel to Meter Conversion

**In BEV Space:**

After IPM transformation, BEV coordinates are already in meters.

**In Camera Space (if needed):**

```
meters_per_pixel = (real_world_width_meters) / (pixel_width)
```

For CARLA (typical):
- Lane width ≈ 3.5 meters
- Image width ≈ 640 pixels
- At bottom of image: `meters_per_pixel ≈ 0.01 m/pixel`
- At horizon: `meters_per_pixel ≈ 0.1 m/pixel` (varies with distance)

### 3.2 Lane Curvature

**Mathematical Definition:**

For polynomial `x = a·y² + b·y + c`:

```
Curvature: κ = |x''(y)| / (1 + x'(y)²)^(3/2)
```

Where:
- `x'(y) = 2a·y + b` (first derivative)
- `x''(y) = 2a` (second derivative)

**Final Formula:**

```
κ = |2a| / (1 + (2a·y + b)²)^(3/2)  [1/meters]
```

**Radius of Curvature:**

```
R = 1/κ  [meters]
```

### 3.3 Vehicle Lateral Offset

**Definition:**

Distance from vehicle center to lane centerline:

```
Δy = x_vehicle - f_center(y_vehicle)
```

Where:
- `x_vehicle` = vehicle lateral position (typically 0 in BEV)
- `f_center(y)` = lane centerline polynomial
- `y_vehicle` = forward position (typically 0)

**Simplified (at y=0):**

```
Δy = -c_center  [meters]
```

Where `c_center` is the constant term of centerline polynomial.

### 3.4 Lane Width Estimation

**Definition:**

```
W(y) = |f_left(y) - f_right(y)|  [meters]
```

Where `f_left` and `f_right` are left/right lane boundary polynomials.

**Average Lane Width:**

```
W_avg = (1/N) · Σ W(y_i)  for y_i ∈ [0, 50] meters
```

### 3.5 Horizon Confidence Decay

**Model:**

Confidence decreases with distance:

```
C_distance(y) = exp(-y² / (2·σ²))
```

Where:
- `y` = forward distance (meters)
- `σ` = decay parameter (typically 30-50 meters)

**Alternative (Linear):**

```
C_distance(y) = max(0, 1 - y / y_max)
```

Where `y_max` is maximum detection range (e.g., 100 meters).

---

## 4️⃣ TEMPORAL CONSISTENCY (KALMAN FILTER)

### 4.1 State Vector

**Definition:**

```
x_k = [a_k, b_k, c_k]ᵀ
```

Where `[a, b, c]` are polynomial coefficients at frame `k`.

### 4.2 State Transition Model

**Assumption:**

Lane shape changes slowly (constant velocity model for coefficients):

```
x_k = F · x_{k-1} + w_{k-1}
```

Where:
- `F` = state transition matrix (identity for constant model)
- `w_{k-1}` = process noise

**Transition Matrix:**

```
F = [1  0  0]
    [0  1  0]
    [0  0  1]
```

**Process Noise:**

```
Q = [σ_a²  0     0   ]
    [0     σ_b²  0   ]
    [0     0     σ_c²]
```

Where `σ_a, σ_b, σ_c` are process noise standard deviations.

### 4.3 Measurement Model

**Observation:**

```
z_k = H · x_k + v_k
```

Where:
- `H` = observation matrix (identity, we observe coefficients directly)
- `v_k` = measurement noise

**Observation Matrix:**

```
H = [1  0  0]
    [0  1  0]
    [0  0  1]
```

**Measurement Noise:**

```
R = [σ_meas_a²  0          0         ]
    [0          σ_meas_b²  0         ]
    [0          0          σ_meas_c² ]
```

### 4.4 Kalman Filter Equations

**Prediction Step:**

```
x̂_{k|k-1} = F · x̂_{k-1|k-1}
P_{k|k-1} = F · P_{k-1|k-1} · Fᵀ + Q
```

**Update Step:**

```
K_k = P_{k|k-1} · Hᵀ · (H · P_{k|k-1} · Hᵀ + R)⁻¹
x̂_{k|k} = x̂_{k|k-1} + K_k · (z_k - H · x̂_{k|k-1})
P_{k|k} = (I - K_k · H) · P_{k|k-1}
```

### 4.5 Noise Modeling

**Process Noise (Q):**

Based on expected lane change rate:

```
σ_a = 0.0001  # Curvature change (1/m² per frame)
σ_b = 0.01    # Heading change (rad per frame)
σ_c = 0.1     # Lateral shift (m per frame)
```

**Measurement Noise (R):**

Based on detection uncertainty:

```
σ_meas_a = 0.001   # Fitting uncertainty
σ_meas_b = 0.05
σ_meas_c = 0.2
```

---

## 5️⃣ FAR LANE CONFIDENCE MODEL

### 5.1 Coverage Ratio

**Definition:**

```
C_cov = N_valid / N_total
```

Where:
- `N_valid` = number of valid detected points
- `N_total` = expected number of points

**Normalized:**

```
C_cov = min(1.0, N_valid / N_min_required)
```

### 5.2 Stability Score

**Definition:**

```
C_stab = 1 - min(1.0, σ(Δcoeffs) / threshold)
```

Where:
- `σ(Δcoeffs)` = standard deviation of coefficient changes over time
- `threshold` = maximum acceptable variation

**Per Coefficient:**

```
C_stab_a = 1 - min(1.0, σ(Δa) / 0.001)
C_stab_b = 1 - min(1.0, σ(Δb) / 0.01)
C_stab_c = 1 - min(1.0, σ(Δc) / 0.1)
```

**Combined:**

```
C_stab = (C_stab_a + C_stab_b + C_stab_c) / 3
```

### 5.3 Symmetry Score

**Definition:**

For left and right lanes:

```
C_sym = 1 - min(1.0, |κ_left - κ_right| / κ_max)
```

Where:
- `κ_left, κ_right` = curvatures of left/right lanes
- `κ_max` = maximum expected curvature difference

**Alternative (Lane Width Consistency):**

```
C_sym = 1 - min(1.0, σ(W(y)) / W_expected)
```

Where `W(y)` is lane width as function of distance.

### 5.4 Temporal Smoothness

**Definition:**

```
C_temp = 1 - min(1.0, ||Δx_k|| / threshold)
```

Where:
- `Δx_k = x_k - x_{k-1}` = coefficient change
- `||·||` = Euclidean norm
- `threshold` = maximum acceptable change

### 5.5 Distance-Weighted Confidence

**Definition:**

```
C_dist = Σ w_i · confidence_i / Σ w_i
```

Where:
- `w_i = exp(-d_i² / σ²)` = distance weight
- `confidence_i` = per-point confidence

### 5.6 Final Confidence Score

**Weighted Combination:**

```
C_final = w1·C_cov + w2·C_stab + w3·C_sym + w4·C_temp + w5·C_dist
```

Where weights sum to 1:

```
w1 = 0.25  # Coverage
w2 = 0.25  # Stability
w3 = 0.20  # Symmetry
w4 = 0.15  # Temporal
w5 = 0.15  # Distance
```

---

## 6️⃣ FAIL-SAFE LOGIC

### 6.1 Far Zone Confidence Low

**Condition:** `C_far < 0.3`

**Action:**
- Use mid-zone polynomial extrapolation
- Reduce far-zone weight to 0.1
- Increase uncertainty bounds

### 6.2 Lane Curvature Explodes

**Condition:** `|κ| > κ_max` (e.g., κ_max = 0.1 1/m, R_min = 10m)

**Action:**
- Reject current detection
- Use previous frame's polynomial
- Increase process noise (Q) for next frame

### 6.3 Sudden Lane Shift Detected

**Condition:** `|Δc| > threshold` (e.g., 0.5 meters in one frame)

**Action:**
- Flag as potential lane change
- Use gradual transition model
- Increase measurement noise (R)

### 6.4 Only One Boundary Detected

**Condition:** Only left OR right lane detected

**Action:**
- Estimate missing boundary from detected one + lane width
- Reduce confidence by 50%
- Use historical lane width if available

### 6.5 Horizon Noise Dominates

**Condition:** Far-zone points have high variance

**Action:**
- Disable far-zone fitting
- Use mid-zone extrapolation only
- Set far-zone confidence to 0

---

## 7️⃣ PLANNING-READY OUTPUT

### 7.1 Continuous Polynomial

**Format:**

```
f_center(y) = a·y² + b·y + c  [meters]
```

Valid for `y ∈ [0, y_max]` (e.g., 0-100 meters).

### 7.2 Sampled Spline

**Format:**

```
waypoints = [(x_i, y_i) for i in range(N)]
```

Where:
- `y_i = i · Δy` (uniform spacing, e.g., Δy = 1 meter)
- `x_i = f_center(y_i)`

**Typical:** N = 50-100 waypoints.

### 7.3 BEV Centerline

**Format:**

```
centerline_bev = {
    'polynomial': [a, b, c],
    'waypoints': [(x_i, y_i), ...],
    'curvature': κ(y),
    'confidence': C_final
}
```

### 7.4 MPC-Compatible Format

**Format:**

```
mpc_reference = {
    'x_ref': [x_0, x_1, ..., x_N],      # Lateral positions
    'y_ref': [y_0, y_1, ..., y_N],      # Forward positions
    'psi_ref': [ψ_0, ψ_1, ..., ψ_N],    # Heading angles
    'kappa_ref': [κ_0, κ_1, ..., κ_N]  # Curvatures
}
```

Where:
- `ψ_i = arctan(f'(y_i))` = heading angle
- `κ_i = κ(y_i)` = curvature at waypoint i

---

## 8️⃣ PERFORMANCE OPTIMIZATION

### 8.1 Async Frame Buffer

**Design:**

```python
class AsyncFrameBuffer:
    def __init__(self, max_size=3):
        self.buffer = queue.Queue(maxsize=max_size)
        self.latest_frame = None
    
    def add_frame(self, frame):
        try:
            self.buffer.put_nowait(frame)
        except queue.Full:
            # Drop oldest frame
            try:
                self.buffer.get_nowait()
            except queue.Empty:
                pass
            self.buffer.put_nowait(frame)
    
    def get_latest(self):
        while not self.buffer.empty():
            self.latest_frame = self.buffer.get_nowait()
        return self.latest_frame
```

### 8.2 Frame Skipping Logic

**Strategy:**

Process every N-th frame if FPS drops:

```python
frame_skip_counter = 0
target_fps = 20
current_fps = measure_fps()

if current_fps < target_fps * 0.8:
    skip_frames = int(target_fps / current_fps)
    frame_skip_counter = skip_frames
else:
    frame_skip_counter = 0

if frame_skip_counter > 0:
    frame_skip_counter -= 1
    continue  # Skip this frame
```

### 8.3 CPU Optimization

**Strategies:**

1. **Vectorization:** Use NumPy operations
2. **Reduced Resolution:** Process at 320x240, upscale results
3. **Sparse Processing:** Process every 2nd row in far zone
4. **Early Termination:** Skip far zone if confidence too low
5. **Caching:** Cache IPM matrix, reuse across frames

---

## 9️⃣ VALIDATION STRATEGY

### 9.1 Ground Truth Comparison

**In CARLA:**

- Use CARLA's map API to get ground truth lane centerline
- Compare detected centerline vs. ground truth
- Metrics: Lateral error, heading error, curvature error

### 9.2 Quantitative Metrics

```
Lateral Error: E_lat = mean(|x_detected(y) - x_gt(y)|)
Heading Error: E_psi = mean(|ψ_detected(y) - ψ_gt(y)|)
Curvature Error: E_kappa = mean(|κ_detected(y) - κ_gt(y)|)
```

### 9.3 Qualitative Assessment

- Visual inspection in CARLA
- Stability over time
- Handling of edge cases (curves, intersections, etc.)

---

## 🔟 IMPLEMENTATION MODULES

See separate implementation files:
- `perception/far_field_lane_detector.py` - Main detector
- `perception/ipm_transformer.py` - IPM transformation
- `perception/lane_fitter.py` - Polynomial fitting
- `perception/kalman_lane_tracker.py` - Temporal smoothing
- `perception/lane_confidence.py` - Confidence scoring
- `perception/lane_validator.py` - Fail-safe logic

---

**END OF ARCHITECTURE DOCUMENT**

