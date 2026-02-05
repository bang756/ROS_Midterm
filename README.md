# warehouse_drone_forklift_sim

- Gazebo 창고 환경에서 드론(X4_GPS_RGBD)과 터틀봇3 burger(지게차 역할)를 이용해 **물품 탐색 → ID 전달 → 지게차 이동**까지 하나의 파이프라인으로 시뮬레이션하는 ROS2 Humble 프로젝트입니다.

---

## 1. 프로젝트 개요
- **한 줄 설명**:
Gazebo 창고에서 드론과 지게차(터틀봇3)를 연동해, ArUco 마커 기반으로 물품을 탐색하고 해당 선반 앞으로 지게차를 이동시키는 ROS2 Humble 시뮬레이션 프로젝트.
- **시스템 동작 개요**
    - Gazebo 월드(`warehouse_env.sdf`) 안에 선반(`empty_shelf` 3열)과 아르코 마커(ID 3/4/5)가 배치되어 있습니다.
    - 드론(X4_GPS_RGBD)이 선반 앞을 일정 고도로 순찰하며 카메라로 ArUco 마커를 검출합니다.
    - 아이템 DB(`item_db.yaml`)에서 “토마토/바나나/사과” → ArUco ID를 매핑합니다.
    - 드론 순찰 노드는 `/find_item` 액션으로 특정 품목을 요청받으면, 드론을 이륙시키고 해당 마커를 찾도록 순찰하며 검출을 시도합니다.
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

- **사용환경**
    - Ubuntu 22.04 LTS (Native 설치)
    - Windows 11 + WSL2 + Ubuntu 22.04
    - ROS2 Humble
    - Gazebo Harmonic

## Python 3.10 설치 (Ubuntu 22.04 기준)

Ubuntu 22.04에는 기본적으로 Python 3.10이 포함되어 있지만, 환경에 따라 명시적으로 설치가 필요할 수 있습니다.

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
## 6.  추후 활용

현재 프로젝트는 "찾을 물건 이름 입력 → 드론 순회 → ArUco 검출 → ID 매핑 → 지게차 waypoint(하드코딩 경로) 이동"까지의 파이프라인을 검증하는 구조입니다.

아래 확장은 추후 구현하려 했던 방법들입니다.

### 6.1 SLAM + Nav2로 확장 (ArUco Pose를 Nav2 Goal로 사용)

지금은 지게차가 /goto_id 액션에서 ID별 사전 정의 경로로 움직입니다.
추후에는 TurtleBot이 SLAM으로 맵을 만들고 Nav2를 붙이면, ArUco 검출 노드가 퍼블리시하는 /aruco_poses를 목적지(goal)로 변환해서 “진짜 위치 기반 자율주행”으로 확장할 수 있습니다.

- **핵심 아이디어**
    - 드론 카메라에서 나온 /aruco_poses는 기본적으로 카메라 좌표계 기준입니다.
    - Nav2 goal은 보통 map 프레임의 geometry_msgs/PoseStamped가 필요합니다.

- **구현 방향(개념)**
    - SLAM 실행해서 map 프레임 생성
    - Nav2 실행 (nav2_bringup)
    - aruco_to_forklift_node를 확장/교체해서: /aruco_poses 구독
    - tf2로 camera_frame -> map 변환
    - 변환된 pose를 Nav2 goal로 전송

이렇게 하면 “ID→고정 경로”가 아니라, 실제 검출된 마커 위치 기반으로 지게차가 자율주행하게 됩니다.

### 6.2 현재 waypoint 이동 로직 그대로 활용해서 “복귀 + 재탐색” 루프 만들기

현재 지게차는 /goto_id로 선반 앞까지 이동할 수 있습니다.

여기서 코드를 크게 바꾸지 않고도, 다음을 추가하면 “새 물품을 다시 찾으러 가는 반복 시나리오”가 가능합니다.

- **목표**
    - 지게차가 목표 선반 앞 도착 → 처음 시작 위치(Home)로 복귀
    - 다음에 “바나나” 같은 새로운 품목 요청이 오면 → 다시 이동

기존 /goto_id의 waypoint(스텝 경로)를 그대로 두고, 이동 후 대기(10초) 이후 기존 경로를 역순으로 실행해서 복귀하도록 만들 수 있습니다.

- **최종 동작 흐름**
    - /find_item으로 “토마토” 요청
    - 드론이 토마토 마커 검출 → 지게차 /goto_id(id=1) 이동
    - 5~10초 대기 후 기존 경로를 역순으로 실행해서 복귀
    - 이후 /find_item으로 “바나나” 요청 시
    - 같은 방식으로 드론 탐색/검출 → 지게차가 다시 이동

 이렇게 하면 요청이 들어올 때마다 반복 수행 가능한 물류 데모로 확장할 수 있습니다.


### 6.3 확장 시 체크포인트

1. Nav2 goal로 쓰려면 /aruco_poses가 map 프레임으로 변환되어야 합니다. (TF 변환이 핵심)

2. 복귀 루프를 만들려면 기존 경로를 역순으로 실행해서 복귀시키는 코드를 추가하는 게 가장 단순합니다.

3. 지게차가 이동 중  /find_item으로 요청이 들어와도 home이 아니면 이동하지 않게, 중복 방지 코드를 추가해 줘야 합니다.

---

## 7.  참고 자료

### 7.1 참고 자료

- Turtlebot3 매뉴얼
    - https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
- Gazebo 내 ArUco 마커 사용법 참조
    - https://github.com/SaxionMechatronics/ros2-gazebo-aruco.git

---

