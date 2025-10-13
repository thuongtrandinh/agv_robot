# Tóm tắt & Hướng dẫn triển khai — Bám quỹ đạo sử dụng Fuzzy Control (dựa trên paper điển hình)

> Mục tiêu tài liệu: tóm tắt ý tưởng, dẫn dắt thiết kế bộ điều khiển fuzzy để bám quỹ đạo cho robot di chuyển 2 bánh (DDMR) hoặc AGV, và cung cấp hướng dẫn triển khai (MATLAB + ROS2/DWA) đủ chi tiết để code trực tiếp.

---

## 1. Tóm tắt ý tưởng chính (Abstract)
Paper mô tả một hệ thống bám quỹ đạo kết hợp giữa planner (global path / reference trajectory) và bộ điều khiển dựa trên logic mờ (fuzzy) để điều chỉnh vận tốc tuyến tính và vận tốc góc nhằm giảm sai lệch vị trí và hướng so với quỹ đạo tham chiếu. Hệ thống thường thêm một lớp local planner (ví dụ DWA) để tránh vật cản; fuzzy điều chỉnh tham số mục tiêu của DWA hoặc trực tiếp tạo lệnh (v, ω). Paper chứng minh rằng fuzzy controller cải thiện ổn định bám quỹ đạo trong môi trường có nhiễu cảm biến, trượt, hoặc khi robot đổi mô-men quán tính.

---

## 2. Kiến trúc hệ thống (system architecture)
- **Input chính**
  - Pose robot hiện tại: \(x, y, \theta\)
  - Quỹ đạo tham chiếu (sequence of waypoints/spline): \( \{x_r(t), y_r(t), \theta_r(t)\} \)
  - Thông tin vận tốc hiện tại: \(v, \omega\) (tuỳ chọn)
  - Bản đồ/occupancy grid hoặc khoảng cách đến vật cản (nếu tích hợp DWA)

- **Các block chính**
  1. Trajectory follower / path tracker (fuzzy controller)
  2. Local planner (DWA) hoặc kinem controller (tùy chọn)
  3. Low-level velocity controller (PID motor)
  4. Safety / watchdog (subgoal selection, escape behavior)

- **Output**
  - Lệnh tốc độ: linear velocity \(v_{cmd}\) (m/s), angular velocity \(\omega_{cmd}\) (rad/s)
  - Mode flags (NORMAL, ESCAPE, SUBGOAL) nếu cần

- **Hai kiểu tích hợp fuzzy**
  - *Direct control*: fuzzy trực tiếp sinh (v, ω)
  - *Supervisory*: fuzzy điều chỉnh setpoint/weight/goal cho DWA (ví dụ: thay đổi lookahead, v_max, weighting between heading và clearance)

---

## 3. Mô hình chuyển động & lỗi cần điều chỉnh
- Robot differential-drive, kinematics:
  \[
  \dot{x} = v \cos\theta,\quad \dot{y} = v \sin\theta,\quad \dot{\theta} = \omega
  \]
- Định nghĩa lỗi (common):
  - **Cross-track error** (lateral): \(e_y\) — khoảng cách vuông góc từ robot đến đường tham chiếu
  - **Along-track error** (longitudinal): \(e_x\) — khoảng cách dọc (có thể dùng để điều chỉnh tốc độ)
  - **Heading error**: \(e_\theta = \theta_r - \theta\) (normalized to \([-π, π]\))
- Thường dùng biến trạng thái đầu vào cho fuzzy: \((e_y, e_\theta, \dot{e_y}, v_{current})\)

---

## 4. Thiết kế bộ điều khiển Fuzzy

### 4.1 Chọn inputs và outputs
- **Inputs (ví dụ)**
  1. `e_lat` — lateral error (m)
  2. `e_head` — heading error (rad)
  3. `v_err` — optionally speed error or along-track error

- **Outputs**
  1. `Δω` — điều chỉnh angular velocity (rad/s) hoặc trực tiếp `ω_cmd`
  2. `v_cmd` — tốc độ tuyến tính (m/s) hoặc điều chỉnh `Δv`

### 4.2 Hàm thành viên (Membership Functions, MFs)
- **e_lat (m)**: NB, NM, Z, PM, PB  
  (Negative Big, Negative Medium, Zero, Positive Medium, Positive Big)  
  các MF tam giác/trapezoid — scale: ví dụ ±1.0 m
- **e_head (rad)**: NB, NS, Z, PS, PB — scale ±π/2 hoặc ±π
- **v_cmd**: SLOW, MEDIUM, FAST → numeric mapping (vdomain 0..v_max)
- **Δω**: NB, NM, Z, PM, PB → map to rad/s (ví dụ ±1.5 rad/s)

> Gợi ý chuẩn hóa: scale inputs về [-1,1] trước khi đánh MF để reuse same MFs.

### 4.3 Bộ luật (Rule base)
Một rule典型 ví dụ (English-like):
- IF `e_lat` IS NB AND `e_head` IS PB THEN `Δω` IS PB AND `v_cmd` IS SLOW
- IF `e_lat` IS Z  AND `e_head` IS Z  THEN `Δω` IS Z  AND `v_cmd` IS FAST
- IF `e_lat` IS PB AND `e_head` IS NB THEN `Δω` IS NB AND `v_cmd` IS SLOW
- IF `abs(e_lat)` IS BIG THEN `v_cmd` IS SLOW
- IF `abs(e_head)` IS BIG THEN `v_cmd` IS SLOW AND `Δω` IS LARGE_CORRECTION

Số lượng rule: thường 5x5 = 25 rules cho 2-inputs (e_lat × e_head). Nếu thêm input thứ 3 thì kết hợp tăng rules, hoặc dùng sparse rule set.

### 4.4 Inference & defuzzification
- Inference: Mamdani phổ biến (min-max)
- Aggregation: max
- Defuzzification: centroid (COG) → gives continuous output `v_cmd`, `Δω`
- Tuning: MF widths & rule weights có thể tuned thủ công hoặc bằng tối ưu (PSO / GA).

---

## 5. Thuật toán bám quỹ đạo (pseudocode)

### 5.1 Preprocessing: reference tracking
- Sử dụng nearest point on path hoặc pure pursuit lookahead để tìm `closest_point` và `target_point` trên quỹ đạo (lookahead distance Ld).
- Tính `e_lat` (ký hiệu lateral distance signed), `e_head` (heading difference to tangent at closest/target point).

### 5.2 Fuzzy loop (sensor → control)
