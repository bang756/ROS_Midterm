# warehouse_drone_forklift_sim

 Gazebo 창고 환경에서 드론(X4_GPS_RGBD)과 터틀봇3 burger(지게차 역할)를 이용해 **물품 탐색 → 위치 전달 → 지게차 이동**까지 하나의 파이프라인으로 시뮬레이션하는 ROS2 Humble 프로젝트입니다.
Ubuntu 22.04 / WSL2 Ubuntu 22.04에서 이 README만 보고 따라 하면, **환경 설치 → ROS2 설치 → 워크스페이스 준비 → 프로젝트 빌드/실행**까지 진행할 수 있도록 구성했습니다.

ROS2 Humble은 Ubuntu 22.04에서 공식 지원되는 LTS 버전이고, Python 3.10은 해당 OS와 기본적으로 호환되며 `rclpy` 기반 Python 노드 개발에 적합합니다. 팀 전체가 동일한 버전을 사용해야 디버깅·협업 시 혼선을 줄일 수 있으므로, 이 조합을 전제로 합니다.

---

## 1. 프로젝트 개요

- **프로젝트 이름(임시)**: `warehouse_drone_forklift_sim`
- **한 줄 설명**:
Gazebo 창고에서 드론과 지게차(터틀봇3)를 연동해, ArUco 마커 기반으로 물품을 탐색하고 해당 선반 앞으로 지게차를 이동시키는 ROS2 Humble 시뮬레이션 프로젝트.
- **시스템 동작 개요**
    - Gazebo 월드(`warehouse_env.sdf`) 안에 선반(`empty_shelf` 3열)과 아르코 마커(ID 3/4/5)가 배치되어 있습니다.
    - 드론(X4_GPS_RGBD)이 선반 앞을 일정 고도로 순찰하며 카메라로 ArUco 마커를 검출합니다.
    - 아이템 DB(`item_db.yaml`)에서 “토마토/바나나/사과” → ArUco ID → 선반 위치(box_pose)를 매핑합니다.
    - 드론 순찰 노드는 `/find_item` 액션으로 특정 품목을 요청받으면, 해당 마커를 찾도록 드론을 이륙·순찰시키고 검출을 시도합니다.
    - ArUco 검출 노드(`ros2_aruco/aruco_node`)는 `/aruco_markers`, `/aruco_poses`를 퍼블리시합니다.
    - 브릿지 노드는 검출된 마커 ID(3/4/5)를 지게차 목적지 ID(1/2/3)로 매핑하고, `/goto_id` 액션을 호출합니다.
    - 터틀봇3 지게차 노드는 `/cmd_vel`을 퍼블리시하며 지정된 선반 앞까지 이동합니다.
- **주요 패키지 / 노드 / 인터페이스**
    - 패키지 `project`
        - 노드 `drone_patrol_node`
            - 액션 서버: `/find_item` (`msg_interface/action/FindItem`)
            - 동작:
                - `item_name`(토마토/바나나/사과)에 맞춰 `/set_detect_target` 호출
                - 드론 이륙 및 웨이포인트(z≈2.025 m) 순찰
                - 순찰 중 ArUco 마커 검출
        - 노드 `forklift_node`
            - 액션 서버: `/goto_id` (`msg_interface/action/GoToId`)
            - 동작:
                - id=1/2/3에 대해 **사전 정의된 TF 기반 스텝 경로**를 따라 `/cmd_vel` 퍼블리시
                - 각 ID는 선반 앞 특정 위치에 대응
        - 노드 `aruco_to_forklift_node`
            - 구독: `/aruco_markers`
            - 매핑:
                - ID 3 → `/goto_id`(id=1)
                - ID 4 → `/goto_id`(id=2)
                - ID 5 → `/goto_id`(id=3)
            - 한 번 유효한 명령을 전송한 후에는 추가 명령을 차단 (중복 이동 방지)
    - 패키지 `ros2_aruco`
        - 노드 `aruco_node`
            - 퍼블리시: `/aruco_markers`, `/aruco_poses`
            - 서비스: `/set_detect_target` (`msg_interface/srv/SetDetectTarget`)
            - 설정 파일: `config/aruco_parameters.yaml`
                - `marker_size = 0.18` (m)
                - `DICT_5X5_250`
                - 카메라 이미지 / 카메라 정보 토픽 지정
    - 리소스
        - Gazebo 월드:
        `project_description/models/warehouse_env.sdf`
        - ArUco 마커 모델:
        `aruco_visual_marker_{3,4,5}/` 및 `_pad` 버전 (숫자 오버레이 포함)
        - 아이템 DB:
        `src/project_description/config/item_db.yaml`

## 2. 사전 요구사항 (Prerequisites)

### 2.1 지원 OS

- **권장**
    - Ubuntu 22.04 LTS (Native 설치)
    - Windows 11 + WSL2 + Ubuntu 22.04

## 3. Python 3.10 설치 (Ubuntu 22.04 기준)

Ubuntu 22.04에는 기본적으로 Python 3.10이 포함되어 있지만, 환경에 따라 명시적으로 설치가 필요할 수 있습니다. 아래 순서를 그대로 따라 하십시오.

## 4. ROS2 Humble 설치

```
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL <https://raw.githubusercontent.com/ros/rosdistro/master/ros.key> -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] <http://packages.ros.org/ros2/ubuntu> $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install ros-humble-desktop

```

---

---

## 5. 실행 방법 (Usage) — 창고 드론·지게차 시뮬레이션

이 프로젝트는 여러 노드를 각각 다른 터미널에서 실행해야 합니다.

각 터미널마다 **ROS2 + 워크스페이스 환경**을 다시 적용하는 것을 잊지 마십시오.

### 5.1 공통 준비 (모든 터미널)

각 터미널에서 아래 두 줄은 항상 먼저 실행하십시오.

```bash
source /opt/ros/humble/setup.bash
source ~/project_ws/install/setup.bash

```

### 5.2 터미널 1 – 메인 시뮬레이션 (Gazebo + 브리지 + 드론/지게차 + 브릿지 노드)

```bash
# 터미널 1
source /opt/ros/humble/setup.bash
source ~/project_ws/install/setup.bash

ros2 launch project master.launch.py

```

### 5.3 터미널 2 – ArUco 검출 노드 실행

```bash
# 터미널 2
source /opt/ros/humble/setup.bash
source ~/project_ws/install/setup.bash

ros2 launch ros2_aruco aruco_recognition.launch.py

```

### 5.4 터미널 3 – 드론 순찰 + 품목 탐색 액션 호출

예: “토마토”를 탐색하는 경우:

```bash
# 터미널 3
source /opt/ros/humble/setup.bash
source ~/project_ws/install/setup.bash

ros2 action send_goal --feedback /find_item \
  msg_interface/action/FindItem "{item_name: '토마토'}"

```

- 다른 품목을 탐색하려면 `item_name`만 변경하면 됩니다.
    - 바나나:
        
        ```bash
        ros2 action send_goal --feedback /find_item \
          msg_interface/action/FindItem "{item_name: '바나나'}"
        
        ```
        
    - 사과:
        
        ```bash
        ros2 action send_goal --feedback /find_item \
          msg_interface/action/FindItem "{item_name: '사과'}"
        
        ```
        

정상 동작 시:

1. 드론이 이륙하여 선반 앞을 순찰합니다.
2. 해당 품목에 대응되는 ArUco ID(3/4/5)가 검출됩니다.
3. `aruco_to_forklift_node`가 ID를 1/2/3으로 매핑하여 `/goto_id` 액션을 호출합니다.
4. 터틀봇3 지게차가 `/cmd_vel`을 퍼블리시하며 해당 선반 앞으로 이동합니다.

---

## 6.  참고 자료

### 6.1 참고 자료

- ROS2 Humble 공식 문서
    - 설치 가이드:
        - https://docs.ros.org/en/humble/Installation.html
    - 튜토리얼(초보자용):
        - https://docs.ros.org/en/humble/Tutorials.html
- Gazebo 내 ArUco 마커 사용법 참조
    - https://github.com/SaxionMechatronics/ros2-gazebo-aruco.git

---
