# Night Patrol Robot

Gazebo 시뮬레이션 환경에서 TurtleBot3 Waffle Pi를 사용해 야간 순찰 로봇을 구성하는 ROS Noetic 패키지입니다. 현재 프로젝트는 사무실 형태의 월드에서 SLAM 기반 맵 생성, frontier 기반 자동 탐색, 저장된 맵 기반 waypoint 순찰, 카메라 영상 기반 화재 감지를 함께 실행하는 것을 목표로 합니다.

## 주요 기능

- Gazebo 월드와 TurtleBot3 로봇 자동 실행
- `gmapping`을 이용한 SLAM 맵 생성
- frontier 기반 자동 탐색 노드 제공
- 저장된 맵, AMCL, `move_base`를 이용한 waypoint 순찰
- 카메라 RGB 이미지에서 빨강/주황 계열을 감지하는 화재 감지 노드
- RViz 설정과 카메라 디버그 이미지 뷰어 실행 옵션 제공

## 패키지 구조

```text
night_patrol_robot/
├── config/                  # AMCL, costmap, move_base 설정
├── launch/                  # Gazebo, 순찰, 맵 저장 launch 파일
├── maps/                    # 저장된 patrol_map 출력 위치
├── rviz/                    # RViz 설정
├── scripts/                 # 탐색, 순찰, 화재 감지 ROS 노드
├── urdf/                    # 로봇 관련 xacro 파일
└── worlds/                  # Gazebo 사무실/테스트 월드
```

## 표준 실행 방법

이 프로젝트는 `launch/patrol_one_button.launch`를 기본 진입점으로 사용합니다. 실행 흐름은 `맵 생성 -> 맵 저장 -> 순찰 실행` 순서로 고정해서 관리합니다.

### 1. 빌드 및 환경 설정

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 맵 생성 모드 실행

Gazebo, TurtleBot3, SLAM, `move_base`, frontier 탐색, 화재 감지를 함께 실행합니다.

```bash
roslaunch night_patrol_robot patrol_one_button.launch mapping:=true
```

화면이 너무 무거우면 RViz와 카메라 뷰어를 끄고 실행합니다.

```bash
roslaunch night_patrol_robot patrol_one_button.launch mapping:=true use_rviz:=false use_camera_viewer:=false
```

frontier 선택 기준을 조정하면서 실행할 수도 있습니다.

```bash
roslaunch night_patrol_robot patrol_one_button.launch mapping:=true frontier_information_gain_weight:=1.5 frontier_distance_weight:=0.8 frontier_obstacle_penalty_weight:=0.4
```

### 3. 맵 저장

탐색으로 충분히 맵이 만들어졌다면 다른 터미널에서 저장합니다.

```bash
roslaunch night_patrol_robot save_patrol_map.launch
```

기본 저장 위치는 `maps/patrol_map.yaml`과 `maps/patrol_map.pgm`입니다.

### 4. 순찰 모드 실행

저장된 맵, AMCL, `move_base`, waypoint 순찰, 화재 감지를 함께 실행합니다.

```bash
roslaunch night_patrol_robot patrol_one_button.launch mapping:=false
```

맵 파일을 직접 지정하려면 다음처럼 실행합니다.

```bash
roslaunch night_patrol_robot patrol_one_button.launch mapping:=false map_file:=$(rospack find night_patrol_robot)/maps/patrol_map.yaml
```

### 자주 쓰는 옵션

```bash
# 최신 월드 대신 다른 월드로 실행
roslaunch night_patrol_robot patrol_one_button.launch mapping:=true world_name:=$(rospack find night_patrol_robot)/worlds/office_fire_detection_test.world

# 화재 감지 없이 순찰만 확인
roslaunch night_patrol_robot patrol_one_button.launch mapping:=false use_fire_detection:=false

# 순찰을 한 바퀴만 실행
roslaunch night_patrol_robot patrol_one_button.launch mapping:=false patrol_loop:=false
```

## 주요 launch 파일

- `launch/gazebo_robot.launch`: Gazebo empty world 실행, TurtleBot3 URDF 로드, 로봇 spawn
- `launch/patrol_one_button.launch`: 맵핑 모드와 순찰 모드를 하나의 launch에서 선택 실행
- `launch/save_patrol_map.launch`: `/map` 토픽을 `maps/patrol_map`으로 저장

## 주요 노드

- `scripts/frontier_explore_node.py`: `/map`에서 frontier 후보를 찾고 `move_base` goal로 전송
- `scripts/auto_explore_node.py`: LaserScan 기반의 간단한 벽 따라가기 탐색
- `scripts/patrol_waypoints_node.py`: AMCL pose 수신 후 설정된 waypoint를 순서대로 순찰
- `scripts/fire_detection_node.py`: `/camera/rgb/image_raw`를 받아 화재 후보 색상 영역을 감지하고 `/fire_detected`와 디버그 이미지를 publish

## 현재 진행 중인 부분

- `worlds/office_patrolv4.world` 기반의 최신 사무실 월드 구성이 진행 중입니다.
- `maps/patrol_map.yaml`과 이미지 파일은 아직 생성 산출물 단계이며, `maps/`에는 기본 placeholder만 있습니다.
- frontier 탐색은 기본 동작은 구현되어 있지만, goal 선택 기준이 가까운 frontier 우선이라 탐색 효율 개선 여지가 있습니다.
- 화재 감지는 현재 Gazebo 테스트 오브젝트에 맞춘 색상 threshold 방식이며, 실제 화재 일반화 모델은 아직 아닙니다.

## 앞으로 구현해야 할 부분

- 최신 월드 파일 이름과 launch 기본값 정리
- SLAM으로 생성한 `patrol_map.yaml`과 map image를 저장하고 순찰 모드에서 검증
- 실제 월드 구조에 맞는 waypoint 재설계 및 순찰 실패 시 복구 동작 추가
- 화재 감지 결과를 순찰 로직과 연동해 감지 시 정지, 알림, 위치 기록을 수행하도록 개선
- frontier goal 선택 기준에 정보량, 거리, 장애물 여유 공간을 함께 반영
- ROS launch smoke test 또는 간단한 노드 단위 테스트 추가
- `package.xml`의 license, maintainer metadata 정리
- README에 실행 화면, 맵 예시, 화재 감지 디버그 이미지 추가

## 개발 메모

- 기본 mapping 전략은 `mapping_strategy:=frontier`입니다.
- 기본 월드는 `worlds/office_patrolv4.world`입니다.
- 순찰 waypoint는 `launch/patrol_one_button.launch` 안의 `waypoints` 파라미터에서 수정합니다.
- 화재 감지 디버그 이미지는 기본적으로 `/fire_detection/debug_image`에서 확인합니다.

## Frontier 탐색 파라미터

`patrol_one_button.launch`에서 다음 arg로 goal 선택 기준을 조정할 수 있습니다.

- `frontier_goal_timeout_sec`: goal 하나에 머무는 최대 시간
- `frontier_min_goal_distance`: 너무 가까운 frontier 제외 거리
- `frontier_blacklist_radius`: 실패한 goal 주변 제외 반경
- `frontier_blacklist_ttl_sec`: 실패한 goal을 blacklist에 유지하는 시간
- `frontier_blacklist_max_size`: blacklist에 보관할 최대 goal 개수
- `frontier_clearance_cells`: goal 주변 장애물 여유 공간 확인 반경
- `frontier_occupied_threshold`: occupied cell로 판단할 occupancy 값
- `frontier_information_radius_cells`: 후보 주변 미탐색 셀 정보량 계산 반경
- `frontier_obstacle_penalty_radius_cells`: 후보 주변 장애물 패널티 계산 반경
- `frontier_information_gain_weight`: 미탐색 셀 정보량 가중치
- `frontier_distance_weight`: 거리 비용 가중치
- `frontier_obstacle_penalty_weight`: 장애물 근접 패널티 가중치
