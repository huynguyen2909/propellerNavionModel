# Navion Propeller V1 - bản C++ tái triển khai

Đây là implementation C++17 độc lập của propeller component cho mô hình
nonlinear 6-DOF Navion.

Mục tiêu của V1 là làm rõ calculation flow và interface trước khi có đủ dữ liệu
hình học/polar thật của propeller Navion/T-6C Texan II. 
Mọi con số của NACA 5868-9/Clark-Y đang dùng đều là placeholder có chủ ý, không phải dữ liệu chính thức.

## Những gì đã triển khai

- BEMT phi tuyến trên lưới `radius x azimuth`;
- `Cl(alpha)` và `Cd(alpha)` dạng bảng;
- phép chiếu lift/drag chính xác, không dùng xấp xỉ inflow angle nhỏ;
- một uniform induced-inflow algebraic hidden state;
- safeguarded Newton/bisection momentum solver;
- static thrust tại `V=0`;
- force vector ba thành phần và full aerodynamic hub moment;
- reaction torque, mean P-factor và aerodynamic rate damping từ cùng phép tích
  phân `r cross dF`;
- moment sinh ra do propeller hub lệch với CG;
- rigid-propeller gyroscopic moment;
- constant RPM và constant pitch;
- CSV history, CSV từng blade element và SVG plot được tạo bởi executable C++.

Toàn bộ runtime API và output dữ liệu dùng SI/radian.

## Cấu hình ước lượng hiện tại

| Tham số | Giá trị V1 | Trạng thái |
|---|---:|---|
| Số cánh | 2 | ước lượng |
| Đường kính | 2.1336 m | cấu hình Navion 84 in tạm chọn |
| Tốc độ quay | 240.8554368 rad/s | tương đương 2300 rpm, ước lượng |
| Root cutout | 0.20 R | ước lượng |
| Pitch tại `r/R=30/42` | 0.27 rad | ước lượng; không gọi nhầm là theta75 |
| Rotating inertia | 5.5 kg m2 | ước lượng |
| Hub so với CG, body | `[1.65, 0, -0.08]` m | ước lượng |
| Installation | shaft trùng body `+x` | ước lượng |
| Rotation sign | `+1` | ước lượng |
| BEMT grid | `64 x 96` | lựa chọn số học V1 |
| Ngưỡng cảnh báo compressibility | Mach 0.70 | chỉ là flag, không hiệu chỉnh tải |
| Airfoil | Clark-Y alpha-only table | bảng giả lập |

Tất cả nằm trong `makeEstimatedNavionNaca5868_9Parameters()` ở cuối
`src/PropellerModel.cpp`. Thay dữ liệu tại đây không cần sửa kernel.

## Runtime interface

Input chính:

```cpp
struct RuntimeInput {
    Vec3 velocityCgRelativeAir_body_m_s;
    Vec3 angularRateBodyWrtInertial_body_rad_s;
    double airDensity_kg_m3;
    double speedOfSound_m_s;
    bool enabled;
};
```

Entry point stateless:

```cpp
PropellerOutput PropellerModel::evaluate(
    const RuntimeInput& input,
    double previousLambdaInduced
) const;
```

`PropellerComponent::step()` là wrapper lưu nghiệm `lambda_i` trước chỉ để
warm-start bước sau. Đây vẫn là hidden state đại số, không phải dynamic-inflow
ODE state.

Output chính:

- `force_body_N`;
- `totalMomentAtCg_body_Nm`;
- full disk/hub loads;
- decomposition reaction/bending/arm/gyro;
- `lambda_i`, residual, iteration count và solver status;
- `CT_rotor`, `CT_prop`, `CQ_prop`;
- alpha range, max section Mach, polar-clamp và reverse-flow flags.
- `compressibilityWarning` khi section Mach vượt 0.70.

## Calculation flow

```text
runtime state
  -> hub velocity and propeller-frame kinematics
  -> trial lambda_i
  -> BEMT over every (psi,r) cell
  -> disk thrust CT_BE
  -> momentum residual CT_BE - CT_MT
  -> safeguarded Newton/bisection until converged
  -> final full disk force and hub moment
  -> body transform + arm moment + gyro moment
  -> force_body and moment_CG_body
```

Xem dẫn xuất và bản đồ nguồn tại
[`docs/MODEL_EQUATIONS.md`](docs/MODEL_EQUATIONS.md).

## Build

Không cần thư viện ngoài.

Với `g++`/MinGW:

```bash
make
make test
make demo
```

Với CMake và Visual Studio Build Tools:

```powershell
cmake -S . -B build-cmake
cmake --build build-cmake --config Release
ctest --test-dir build-cmake -C Release --output-on-failure
./build-cmake/Release/takeoff_roll.exe
```

## Kịch bản chạy đà

Executable quy định tuyến tính:

\[
V(t)=29.0576\frac{t}{20}\;\text{m/s},\qquad 0\le t\le20\;\text{s}.
\]

Đây là prescribed-kinematics test, chưa phải nghiệm ground roll từ lực tổng của
máy bay. Các giả thiết:

- không gió, sea-level density;
- ground pitch cố định `0.035 rad`;
- `p=q=r=0`;
- shaft trùng body-x;
- propeller đã ở fixed takeoff RPM tại `t=0`;
- mô phỏng dừng đúng lúc đạt `V_rotation`; chưa điều khiển elevator và chưa bắt
  đầu rotation.

Do ground pitch khác zero, vận tốc runway có một thành phần `W_b>0`. Với
`rotationSign=+1`, mô hình tạo mean P-factor yaw moment âm như xu hướng của
Stevens Eq. 8.2-25. Gyroscopic moment bằng zero trong bài này vì `q=0`; unit
test riêng kiểm tra trường hợp `q` khác zero.

Kết quả được ghi vào:

```text
results/takeoff_roll_history.csv
results/takeoff_roll_history.svg
results/blade_elements_static.csv
results/blade_elements_vrotation.csv
```

## Kết quả placeholder hiện tại

| Điểm | Thrust [N] | Torque required [N m] | Power required [W] |
|---|---:|---:|---:|
| Brake release | 4554.89 | 531.73 | 128069.98 |
| V_rotation | 3141.58 | 555.23 | 133730.41 |

Các giá trị này chỉ chứng minh đường tính chạy và có độ lớn hữu hạn. Không dùng
chúng để kết luận performance của Navion trước khi thay geometry, polar và
operating parameters bằng dữ liệu thật.

## Cấu trúc source

```text
include/navion/propeller/Math3.hpp
include/navion/propeller/PropellerModel.hpp
src/PropellerModel.cpp
app/takeoff_roll.cpp
tests/test_propeller.cpp
docs/MODEL_EQUATIONS.md
docs/VERIFICATION.md
```

`evaluateDiskAtInflow(..., true)` cho phép xuất từng blade element để kiểm tra
`Ua`, `Ut`, `phi`, `alpha`, `Cl`, `Cd`, `dF` và `dM` tại một nghiệm `lambda_i`.
