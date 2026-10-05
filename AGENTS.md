# autoRCcar — Agent Guide

1/8 스케일 자율주행 RC카(VRX RH818 개조) 프로젝트.
실차 ROS 2 스택(`ros2/`)과 Isaac Sim 시뮬레이션 환경(`isaacsim/`)으로 구성된다.
이 파일은 모든 에이전트(Claude Code, Codex 등)의 공통 가이드이다.

## 작업 기록

- 그날 작업을 시작할 때 `.history/` 폴더의 최근 `agent_log_<YYYYMMDD>.md`를 확인하고, 작업이 끝나면 그날 파일(`작업한 내용`, `남은 할 일`, `알게 된 주의사항`)을 추가·갱신한다.

## 환경

- Ubuntu 24.04, ROS 2 **Jazzy** (`/opt/ros/jazzy`). 원래 Humble에서 작성된 코드를 포팅함
- Isaac Sim **6.1.0**: `~/isaacsim` (Python 실행은 `~/isaacsim/python.sh`)
- GPU: RTX 4070
- Isaac Sim 공식 에이전트 스킬: `~/isaacsim/skills/<name>/SKILL.md`
  (작업 주제가 맞을 때만 필요한 스킬을 읽는다. 예: `isaac-sim-ros2-bridge`,
  `usd-articulation`, `physics-simulation`, `isaac-sim-sensor`, `isaac-sim-headless-deployment`)

## 폴더 구조

```
ros2/src/        실차 ROS 2 패키지 (colcon 워크스페이스 = ros2/)
isaacsim/        Isaac Sim 시뮬레이션
  assets/        차량/환경 USD (autoRCcar_RH818.usda)
  scripts/       에셋 생성·시뮬레이션 실행 스크립트
  tests/         시뮬레이션 테스트
  logs/          실행 로그 (커밋하지 않음)
GCS/             Flutter 지상국 앱 (rosbridge로 통신)
utils/           bag 파싱, fake livox publisher 등 유틸 스크립트
hardware/        펌웨어, 도면
archive/         더 이상 쓰지 않는 패키지 (수정하지 않음)
Livox-SDK2/      Livox SDK (init_setup.sh에서 빌드·설치)
```

## 빌드

```bash
./init_setup.sh      # 최초 1회: apt 의존성, Livox-SDK2 설치 (sudo 필요)
./build_ros2.sh      # livox_ros_driver2 먼저 빌드 후 나머지 패키지 빌드
source ros2/install/setup.bash
```

- `livox_ros_driver2`는 반드시 `build_ros2.sh`(또는 `livox_ros_driver2/build.sh jazzy`)로 빌드한다.
  `package.xml`이 빌드 스크립트에서 생성된다.
- 단일 패키지 재빌드: `cd ros2 && colcon build --symlink-install --packages-select <pkg>`

## autoRCcar 실행방법
```
./start_rosbridge.sh                       # GCS와 Ros 사이 메시지 교환을 위한 rosbridge 실행
./start_process_manager.sh                 # GCS와 Ros 간 제어를 위한 패키지 실행
./GCS/GCS_Linux_Release/autorccar_gcs      # 리눅스 환경에서의 GCS 실행
GCS/GCS_Windows_Release/autorccar_gcs.exe  # 윈도우 환경에서의 GCS 실행         
ros2 run gscam gscam_node                                                # 센서 - 카메라 실행
ros2 run autorccar_ubloxf9r ubloxf9r                                     # 센서 - IMU, GNSS 실행
ros2 launch livox_ros_driver2 msg_MID360_launch.py                       # 센서 - Livox MID360 LiDAR 실행
ros2 launch lio_sam run.launch.py                                        # 항법 - LIO-SAM SLAM 실행
ros2 launch autorccar_ins_gnss ins_gnss_nav.launch.py                    # 항법 - INS/GNSS 복합항법 실행
ros2 launch autorccar_planning_control planning_control.launch.py        # 경로계획 실행
ros2 launch autorccar_hardware_control hardware_control.launch.py        # HW 제어 실행
ros2 launch autorccar_costmap costmap.launch.py                          # LiDAR 기반 Cost Map 실행
```
- LIO-SAM 항법의 경우, `msg_MID360_launch` 런치파일만 실행하면 된다.
- INS/GNSS 복합항법의 경우, `ubloxf9r` 노드만 실행하면 된다. 

## Isaac Sim 시뮬레이션 원칙

- 목표: 실차 ROS 2 스택을 **수정 없이(또는 최소 수정으로)** sim에 연결한다.
  sim은 하드웨어 계층(라이다, IMU, `hardware_control`)을 대체한다.
- 연동 시 고려사항:
  - lio_sam은 `PointCloud2`가 아닌 Livox `CustomMsg`(점별 `offset_time`)를 구독한다
  - 차량 구동은 `ControlCommand` → 조향/구동 joint 변환 노드로 `hardware_control`을 대체
  - 현재 패키지에 `use_sim_time` 설정이 없음 → sim 연동 시 `/clock` + `use_sim_time` 필요
- 좌표계: Z-up, X-forward, Y-left, 단위 m/kg. 차량 `base_link` 원점 = 휠베이스 중앙, 차축 높이
- 차량 실측 파라미터: wheelbase 0.325 m, 앞 트레드 0.270 m, 뒤 트레드 0.260 m,
  바퀴 반지름 0.055 m, 최대 조향 ±45°, 총 질량 3.8 kg
- 에셋은 `isaacsim/scripts/build_car_usd.py`로 생성한다. `.usda`를 손으로 고치지 말고
  생성 스크립트를 수정한 뒤 다시 생성한다.
- 스크립트는 가능하면 **headless standalone**(`~/isaacsim/python.sh <script>`,
  `SimulationApp({"headless": True})`)으로 작성해 에이전트가 직접 실행·검증할 수 있게 한다.
  GUI Script Editor 전용 스크립트(`test_drive.py` 등)는 그 사실을 파일 상단 docstring에 명시한다.
- Isaac Sim과 ROS 2 노드를 함께 띄울 때는 같은 `ROS_DOMAIN_ID`, RMW를 사용한다.

## 코드 규칙

- 기존 코드의 스타일(주석 밀도, 네이밍)을 따른다. 주석은 영어로 작성한다.
- `archive/`, `Livox-SDK2/`(빌드 오류 수정 제외), `ros2/src/livox_ros_driver2`는 꼭 필요할 때만 수정
- 실차 스택 패키지를 sim 때문에 수정할 경우, 실차 동작이 바뀌지 않도록 파라미터/launch 인자로 분기한다

## Git

- 커밋 메시지는 영어, **동사원형 소문자**로 시작 (예: `add ...`, `fix ...`, `port ...`)
- 브랜치: `main`이 기본, 작업 브랜치는 사용자가 직접 생성하며 `<이니셜>/<topic>` 형식 (예: `pgs/sim_jazzy`)
- 사용자가 요청하기 전에는 커밋/푸시하지 않는다
