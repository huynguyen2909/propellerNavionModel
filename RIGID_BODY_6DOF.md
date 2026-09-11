# Navion V1 - Rigid Body Newton-Euler 6-DOF

## 1. Kết quả triển khai

Module mới tích phân chuyển động phi tuyến của máy bay cứng quanh khối tâm trong
mô hình flat-Earth, non-rotating NED:

- trạng thái động lực: `u, v, w, p, q, r`;
- vị trí CG: North, East, Down;
- attitude nội bộ: unit quaternion `q_nb` đổi vector body sang NED;
- output đọc/plot: Euler 3-2-1 `roll, pitch, yaw`;
- tải vào: lực body và moment body quanh CG của từng component;
- bộ tích phân: classical RK4, đánh giá lại tải phụ thuộc trạng thái tại cả bốn
  stage;
- gravity được khối rigid body cộng riêng, không nằm trong tổng tải component.

Mã nằm tại:

```text
include/navion/dynamics/RigidBody6Dof.hpp
include/navion/dynamics/PropellerRigidBodyCoupling.hpp
src/RigidBody6Dof.cpp
src/PropellerRigidBodyCoupling.cpp
app/propeller_6dof_demo.cpp
tests/test_rigid_body.cpp
```

## 2. Quy ước hệ trục và trạng thái

| Đại lượng | Quy ước |
|---|---|
| Body | `+x_b` trước, `+y_b` phải, `+z_b` xuống |
| Earth | flat-Earth NED: North, East, Down |
| `q_nb` | scalar-first Hamilton quaternion, đổi body components sang NED |
| `C_nb` | DCM body-to-NED tạo từ `q_nb` |
| `velocityCgWrtEarth_body_m_s` | vận tốc CG so với Earth, biểu diễn trong body |
| `angularRateBodyWrtNed_body_rad_s` | `[p,q,r]`, tốc độ body so với NED, biểu diễn trong body |
| Lực | `[X,Y,Z]` body, N |
| Moment | `[L,M,N]` quanh CG, body, N m |
| Đơn vị | SI và radian |

Mọi component phải quy moment về CG trước khi đi vào `BodyLoads`. Nếu một
component tính moment tại reference point `P`, dùng

\[
\mathbf M_{CG}^{b}=\mathbf M_P^{b}+\mathbf r_{P/CG}^{b}\times\mathbf F^{b}.
\]

Phân biệt hai vận tốc là bắt buộc khi có gió:

\[
\mathbf V_{CG/a}^{b}
=\mathbf V_{CG/E}^{b}-\mathbf C_{b\leftarrow n}\mathbf V_{wind/E}^{n}.
\]

State rigid body lưu `V_CG/E`; adapter tạo `V_CG/a` rồi mới gọi propeller.

## 3. Phương trình Newton

Với tổng lực **không kể trọng lực** của các component là
`componentLoads.force_body_N`, code tính

\[
\dot{\mathbf V}_{CG/E}^{b}
=\frac{\mathbf F_{components}^{b}}{m}
+\mathbf C_{b\leftarrow n}
\begin{bmatrix}0\\0\\g\end{bmatrix}
-\boldsymbol\omega^{b}\times\mathbf V_{CG/E}^{b}.
\]

Đây là dạng vector của Zipfel (2025), Eq. (5.41), và tương đương ba phương
trình lực trong Roskam Eq. (2.55) / Nelson Table 3.1. Với attitude level,
gravity body là `[0,0,+g]`, đúng quy ước `+z_b` xuống.

Vị trí được tích phân bằng

\[
\dot{\mathbf r}_{CG}^{n}
=\mathbf C_{n\leftarrow b}\mathbf V_{CG/E}^{b},
\]

tương ứng Zipfel Eq. (5.42).

## 4. Phương trình Euler

Tensor moment quán tính là tensor thật quanh CG, biểu diễn trong body:

\[
\mathbf I_b=
\begin{bmatrix}
I_{xx}&-I_{xy}&-I_{xz}\\
-I_{xy}&I_{yy}&-I_{yz}\\
-I_{xz}&-I_{yz}&I_{zz}
\end{bmatrix}.
\]

Code không hard-code dạng đối xứng `Ixy=Iyz=0`; nó giải trực tiếp hệ 3x3:

\[
\dot{\boldsymbol\omega}^{b}
=\mathbf I_b^{-1}
\left[
\mathbf M_{components,CG}^{b}
-\boldsymbol\omega^{b}\times
(\mathbf I_b\boldsymbol\omega^{b})
\right].
\]

Đây là Zipfel Eq. (6.41). Khi tensor chỉ còn `Ixz`, khai triển cho ra đúng ba
phương trình moment Roskam Eq. (2.56) và Nelson Table 3.1.

`RigidBodyParameters::validate()` bắt buộc tensor hữu hạn, đối xứng và xác định
dương. Hàm `inertiaTensorFromMassMoments()` nhận các **product of inertia**
theo convention flight dynamics và tự đặt dấu âm vào off-diagonal tensor.

## 5. Động học attitude bằng quaternion

Euler angles thuận tiện để đọc nhưng có singularity tại pitch ±90 độ. Vì vậy
state tích phân unit quaternion `q_nb=[q0,q1,q2,q3]`. Với body rates
`[p,q,r]`, code dùng

\[
\begin{bmatrix}
\dot q_0\\\dot q_1\\\dot q_2\\\dot q_3
\end{bmatrix}
=\frac12
\begin{bmatrix}
0&-p&-q&-r\\
p&0&r&-q\\
q&-r&0&p\\
r&q&-p&0
\end{bmatrix}
\begin{bmatrix}q_0\\q_1\\q_2\\q_3\end{bmatrix},
\]

đúng Zipfel Eq. (4.77). Quaternion được chuẩn hóa tại các stage và sau bước
RK4. Euler 3-2-1 chỉ là output dẫn xuất.

## 6. Dữ liệu khối lượng và quán tính Navion

Nelson (1998), Appendix B, Figure B.1 cho bộ dữ liệu **general aviation
airplane: NAVION**:

| Tham số gốc | Giá trị gốc | Giá trị dùng trong code |
|---|---:|---:|
| `W` | 2750 lb | `mass = 1247.3790175 kg` |
| `CG` | 29.5% MAC | metadata; origin động lực đặt ngay tại CG |
| `Ix` | 1048 slug ft² | 1420.897209851307 kg m² |
| `Iy` | 3000 slug ft² | 4067.453844994201 kg m² |
| `Iz` | 3530 slug ft² | 4786.037357609844 kg m² |
| `Ixz` | 0 | 0 kg m² |

Hệ số đổi dùng trong factory:

\[
1\ \mathrm{lbm}=0.45359237\ \mathrm{kg},\qquad
1\ \mathrm{slug\,ft^2}=1.3558179483314004\ \mathrm{kg\,m^2}.
\]

Đây là **một configuration tham chiếu** tại 2750 lb, không phải tensor bất biến
cho mọi Navion hoặc mọi loading. TCDS A-782 bao phủ nhiều model/weight khác
nhau nhưng không cung cấp tensor quán tính dùng cho simulation. Khi xây weight
and balance model, phải thay factory bằng `mass`, `CG` và tensor của đúng loading
case; đồng thời cập nhật `hubPositionFromCg_body_m` của propeller theo CG mới.

Factory hiện tại:

```cpp
RigidBodyParameters p = makeNelsonNavionRigidBodyParameters();
```

## 7. Ghép với propeller hiện tại

Không cộng từng trường moment của propeller. Adapter chỉ lấy:

```cpp
loads.force_body_N = output.force_body_N;
loads.momentAtCg_body_Nm = output.totalMomentAtCg_body_Nm;
```

`totalMomentAtCg_body_Nm` đã bằng

\[
\mathbf M_{hub,aero}^{b}
+\mathbf r_{hub/CG}^{b}\times\mathbf F_{prop}^{b}
+\mathbf M_{gyro}^{b}.
\]

Trong implementation BEMT hiện tại, shaft reaction torque đã nằm trong
`aerodynamicMomentAtHub_body_Nm`. Cộng `reactionTorque_body_Nm`,
`momentArm_body_Nm` hoặc `gyroscopicMoment_body_Nm` thêm lần nữa sẽ
double-count.

`PropellerRigidBodyCoupling::stepRk4()`:

1. dùng state từng RK4 stage để tạo air-relative propeller input;
2. gọi `PropellerModel::evaluate()` stateless với cùng accepted warm start;
3. đặt output propeller vào slot `AircraftComponentLoads::propeller`;
4. để fuselage, wing, horizontal tail, vertical tail và gear bằng zero nếu không
   có callback;
5. tích phân rigid body;
6. solve propeller tại end-state và chỉ khi hội tụ mới commit `lambdaInduced` làm
   warm start bước sau.

Cách này không biến `lambdaInduced` thành ODE state và không làm hạ RK4 xuống
first-order do giữ lực propeller cố định trong cả bước.

## 8. API tối thiểu

```cpp
RigidBody6Dof body(makeNelsonNavionRigidBodyParameters());

AircraftComponentLoads components{};
components.propeller = bodyLoadsFromPropeller(propellerOutput);
// Các component chưa có tự động bằng zero.

RigidBodyState next = body.stepRk4(
    state,
    dt_s,
    [&components](const RigidBodyState&) { return components.total(); }
);
```

Ghép tự động với propeller BEMT:

```cpp
PropellerRigidBodyCoupling simulation{
    RigidBody6Dof(makeNelsonNavionRigidBodyParameters()),
    PropellerComponent(PropellerModel(makeEstimatedNavionNaca5868_9Parameters()))
};

CoupledStepResult out = simulation.stepRk4(
    state,
    dt_s,
    FlatEarthEnvironment{}
);
state = out.state;
```

Khi wing/fuselage/tail/gear hoàn thành, truyền callback trả về các slot tương
ứng. Coupler sẽ tự overwrite slot propeller bằng tải propeller tại đúng stage.
Callback được gọi nhiều lần cho các state thử của RK4; vì vậy nó nên là phép
ánh xạ thuần hoặc chỉ commit hidden state sau khi bước thời gian được chấp nhận.

## 9. Kiểm thử

`tests/test_rigid_body.cpp` gồm 9 bài:

1. đổi đơn vị bộ mass/inertia Nelson;
2. Euler-quaternion-DCM round trip;
3. free fall analytic;
4. hạng vận chuyển body-axis của Newton;
5. residual phương trình Euler với đủ product of inertia;
6. bảo toàn vận tốc NED khi force-free và bảo toàn unit quaternion;
7. tổng tải các component;
8. adapter không double-count moment propeller;
9. một bước ghép thật BEMT-propeller-rigid-body.

Chạy:

```bash
make test
```

hoặc:

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

## 10. Demo và giới hạn vật lý hiện tại

```bash
make build/propeller_6dof_demo
./build/propeller_6dof_demo 0.5 0.01
```

Hai argument là `duration_s` và `dt_s`. CSV được ghi vào
`results/propeller_6dof_history.csv`.

Demo bắt đầu đứng yên ở 100 m trên origin NED, có gravity và chỉ có tải
propeller. Do chưa có wing/fuselage/tail/landing gear:

- máy bay rơi tự do đồng thời tăng tốc về trước;
- reaction torque làm thân roll;
- kết quả không phải takeoff-roll prediction;
- không được đặt máy bay trên runway rồi kỳ vọng constraint mặt đất khi slot
  landing gear vẫn bằng zero.

Để mô phỏng chạy đà đúng, component landing gear phải cấp normal force,
spring-damper, rolling resistance/braking và moment quanh CG; các surface khí
động phải cấp tải theo air-relative state.

## 11. Tài liệu đối chiếu

- J. Roskam, *Airplane Flight Dynamics and Automatic Flight Controls*, 1979,
  Vol. I, Ch. 2, Eqs. (2.53)-(2.57), pp. 31-33.
- R. C. Nelson, *Flight Stability and Automatic Control*, 2nd ed., 1998,
  Ch. 3, pp. 97-105, Table 3.1; Appendix B, Fig. B.1, p. 401.
- P. H. Zipfel, *Modeling and Simulation of Aerospace Vehicle Dynamics*,
  4th ed., 2025, Eqs. (4.77)-(4.78), pp. 134-135; Eqs. (5.41)-(5.42),
  p. 172; Eq. (6.41), p. 202.
- FAA Dynamic Regulatory System, TCDS A-782 family record for Navion models:
  <https://drs.faa.gov/>.
- Sierra Hotel Aero/Navion, current type-certificate-holder context:
  <https://www.navion.com/>.

Internet/TCDS material was used to check configuration scope. The numerical
mass and inertia in code come from Nelson Appendix B because the certification
record does not publish the complete simulation tensor.
