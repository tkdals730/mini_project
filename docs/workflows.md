# Night Patrol Robot Workflows

이 문서는 야간 순찰 로봇 프로젝트의 실제 운영 흐름과 검증 절차를 정리한다.
프로젝트의 기본 목표는 처음 실행 시 지도를 만들고, 이후 저장된 지도를 기준으로 정해진 시간에 순찰을 수행한 뒤 원래 자리로 돌아오는 것이다.
화재 감지 시 경보 방식은 아직 설계 중이며, 현재는 감지 결과를 확인하는 단계로 둔다.

## 진행 현황

| 순서 | 단계 | 현재 상태 | 다음에 할 일 |
| --- | --- | --- | --- |
| 1 | 초기 맵핑 | 완료 | 필요 시 다른 월드/파라미터에서 재검증 |
| 2 | 맵 저장 | 완료 | 저장 맵 산출물을 기준 맵으로 유지 |
| 3 | 저장 맵 기반 순찰 | 완료 | 추가 실패 케이스 발견 시 복구 정책 보강 |
| 4 | 특정 시간 순찰 | 미구현 | scheduler 노드 또는 시작 트리거 방식 결정 |
| 5 | 원래 자리 복귀 | 완료 | 필요 시 home 좌표/tolerance 미세 조정 |
| 6 | 화재 감지 | 기본 감지 구현 | 감지 후 경보, 정지, 알림 정책 결정 |
| 7 | 원버튼 자동 실행 | 부분 구현 | `mapping:=auto` 전체 흐름을 처음부터 끝까지 검증 |

### 체크리스트 진행률

- [x] 1. 초기 맵핑 완료
  - [x] SLAM/gmapping 실행 확인
  - [x] frontier goal 생성 확인
  - [x] `move_base` goal 처리 확인
  - [x] RViz에서 맵 확장 안정성 확인
  - [x] 먼 미탐사 frontier 후보 선택 확인 (`frontier_max_goal_distance:=0.0`)
  - [x] 벽/장애물 내부 goal 미생성 확인
  - [x] 막힌 goal 반복 선택 방지 및 완료 전환 확인
- [x] 2. 맵 저장 완료
  - [x] 저장 명령/노드 구성 확인
  - [x] `maps/patrol_map.yaml` 최종 저장 확인
  - [x] `maps/patrol_map.pgm` 최종 저장 확인
  - [x] 저장 맵 품질 확인
  - [x] 저장 맵 `map_server` 로드 확인
- [x] 3. 저장 맵 기반 순찰 완료
  - [x] `map_server` 실행 확인
  - [x] AMCL 위치 추정 확인
  - [x] waypoint 순찰 확인
  - [x] 순찰 실패 시 복구/중단 동작 확인
- [ ] 4. 특정 시간 순찰 완료
  - [ ] 순찰 시간 설정 방식 결정
  - [ ] scheduler 또는 시작 트리거 구현
  - [ ] 순찰 시간 도달 시 자동 출발 확인
  - [ ] 다음 순찰 시간까지 대기 확인
- [x] 5. 원래 자리 복귀 완료
  - [x] home 위치 정의
  - [x] 순찰 완료 후 home 복귀 구현
  - [x] 맵 저장 후 home 복귀 구현
  - [x] home 복귀 최종 waypoint 성공 확인
- [ ] 6. 화재 감지 완료
  - [x] 카메라 토픽 확인
  - [x] `/fire_detected` publish 확인
  - [x] debug image 확인
  - [ ] 화재 감지 후 경보 방식 결정
  - [ ] 화재 감지 후 로봇 동작 정책 결정
- [ ] 7. 원버튼 자동 실행 완료
  - [x] `patrol_one_button.launch` 진입점 구성
  - [ ] 맵 없을 때 자동 맵핑 확인
  - [x] 맵 있을 때 자동 순찰 확인
  - [ ] 전체 시나리오 처음부터 끝까지 검증

### 현재 우선순위

1. 특정 시간 순찰 scheduler 또는 시작 트리거 방식을 결정한다.
2. 화재 감지 후 경보, 정지, 알림 등 대응 정책을 결정한다.
3. `mapping:=auto` 전체 흐름을 맵이 없는 상태부터 끝까지 재현 검증한다.
4. 발표/시연용으로 Gazebo/RViz 실행 화면과 핵심 로그를 정리한다.

## 전체 운영 흐름

```text
초기 맵핑
-> 맵 저장
-> 저장 맵 기반 순찰
-> 특정 시간 순찰 시작
-> waypoint 순찰
-> 원래 자리 복귀
-> 다음 순찰 시간까지 대기
-> 화재 감지 시 경보/대응
```

## Workflow 1. 초기 맵핑

### 목적

처음 환경을 실행했을 때 로봇이 Gazebo 사무실 월드에서 스스로 움직이며 SLAM 지도를 만든다.

### 실행

```bash
roslaunch night_patrol_robot patrol_one_button.launch mapping:=true
```

화면이 무거우면 RViz와 카메라 뷰어를 끈다.

```bash
roslaunch night_patrol_robot patrol_one_button.launch mapping:=true use_rviz:=false use_camera_viewer:=false
```

### 확인할 것

- `/map` 토픽이 publish되는지 확인
- `slam_gmapping` 노드가 실행되는지 확인
- `frontier_explore_node`가 frontier goal을 생성하는지 확인
- `move_base`가 goal을 받아 로봇을 이동시키는지 확인
- RViz에서 맵이 점점 확장되는지 확인
- goal이 벽 바로 옆이나 장애물 내부로 찍히지 않는지 확인
- 막힌 goal 실패 후 같은 위치를 계속 반복하지 않는지 확인

### 관련 토픽/노드

- `/map`
- `/move_base/current_goal`
- `/move_base/status`
- `/cmd_vel`
- `slam_gmapping`
- `frontier_explore_node`
- `move_base`

## Workflow 2. 맵 저장

### 목적

초기 맵핑이 끝난 뒤 순찰 모드에서 사용할 지도를 저장한다.

### 자동 저장

frontier 탐색이 완료되면 `auto_map_saver_node.py`가 `maps/patrol_map`으로 저장하고 home waypoint로 복귀한다. 현재 구현은 `/exploration_complete`를 수신하면 맵을 저장한 뒤 `home_approach_waypoint`, `home_waypoint` 순서로 `move_base` goal을 보낸다.

### 수동 저장

```bash
roslaunch night_patrol_robot save_patrol_map.launch
```

### 확인할 것

- `maps/patrol_map.yaml`이 생성 또는 갱신되는지 확인
- `maps/patrol_map.pgm`이 생성 또는 갱신되는지 확인
- 저장된 맵의 벽 구조가 Gazebo 월드와 크게 어긋나지 않는지 확인

## Workflow 3. 저장 맵 기반 순찰

### 목적

저장된 맵을 불러와 AMCL로 위치를 추정하고, waypoint를 따라 순찰한다.

### 실행

```bash
roslaunch night_patrol_robot patrol_one_button.launch mapping:=false
```

### 확인할 것

- `map_server`가 저장된 `patrol_map.yaml`을 불러오는지 확인
- `amcl`이 로봇 위치를 추정하는지 확인
- `move_base`가 waypoint goal을 처리하는지 확인
- 로봇이 waypoint를 순서대로 방문하는지 확인
- 순찰 중 실패한 goal이 있을 때 안전하게 다음 동작으로 넘어가는지 확인

### 관련 토픽/노드

- `/map`
- `/amcl_pose`
- `/move_base/current_goal`
- `/move_base/status`
- `map_server`
- `amcl`
- `move_base`
- `patrol_waypoints_node`

## Workflow 4. 특정 시간 순찰

### 목적

로봇이 항상 움직이는 것이 아니라, 정해진 순찰 시간이 되면 출발하고 순찰이 끝나면 원래 자리로 돌아온다.

### 현재 상태

아직 스케줄러 노드는 구현 전이다.
현재는 수동으로 `mapping:=false` 순찰 모드를 실행해 waypoint 순찰을 확인하는 단계다.

### 목표 동작

```text
대기 상태
-> 순찰 시간 도달
-> 저장 맵 기반 순찰 시작
-> 지정 waypoint 방문
-> 시작 위치 또는 충전 위치로 복귀
-> 다음 순찰 시간까지 대기
```

### 구현 후보

- ROS param으로 순찰 시작 시간 목록 설정
- 단순 Python scheduler 노드 추가
- `/start_patrol` 서비스 또는 토픽으로 수동/자동 시작 지원
- 순찰 완료 후 home waypoint로 복귀

### 확인할 것

- 순찰 시간이 아닐 때 로봇이 대기하는지 확인
- 순찰 시간이 되면 waypoint 순찰이 시작되는지 확인
- 순찰 완료 후 원래 자리 또는 home waypoint로 돌아오는지 확인
- 다음 순찰 시간까지 다시 대기하는지 확인

## Workflow 5. Home 복귀

### 목적

초기 맵핑/맵 저장 또는 저장 맵 기반 순찰이 끝난 뒤 로봇을 시작 위치 근처의 home waypoint로 돌려보낸다.

### 현재 구현

- `auto_map_saver_node.py`는 맵 저장 후 `home_approach_waypoint`, `home_waypoint` 순서로 복귀 goal을 보낸다.
- `patrol_waypoints_node.py`는 순찰 cycle 이후 `return_home:=true`이면 home waypoint로 복귀한다.
- RViz marker에는 home entry와 home 위치가 함께 표시된다.

### 확인할 것

- 맵 저장 후 `Saved map automatically` 로그가 출력되는지 확인
- 이어서 `Map saved. Returning robot to home position.` 로그가 출력되는지 확인
- home entry와 home waypoint goal이 순서대로 성공하는지 확인
- 실패 시 timeout, 대체 waypoint, 재시도 정책이 필요한지 기록

### 남은 위험

`office_patrol_nov4.world` 상단 끝 구역에서 reachable frontier가 남아 있는 것으로 판단되면 `/exploration_complete`가 늦어질 수 있다. 반복 실패 frontier suppress와 강제 완료 전환을 재실행으로 검증한다.

## Workflow 6. 화재 감지

### 목적

순찰 중 카메라 이미지에서 화재 후보를 감지한다.

### 실행

화재 감지 노드는 기본적으로 `patrol_one_button.launch`에서 함께 실행된다.

```bash
roslaunch night_patrol_robot patrol_one_button.launch mapping:=false use_fire_detection:=true
```

화재 감지 테스트 월드를 직접 지정할 수도 있다.

```bash
roslaunch night_patrol_robot patrol_one_button.launch mapping:=false world_name:=$(rospack find night_patrol_robot)/worlds/office_fire_detection_test.world
```

### 확인할 것

- `/camera/rgb/image_raw`가 publish되는지 확인
- `/fire_detected`가 publish되는지 확인
- `/fire_detection/debug_image`에서 감지 영역이 표시되는지 확인
- 화재가 없을 때 false 상태가 유지되는지 확인
- 화재 후보가 보이면 true 상태로 바뀌는지 확인

### 아직 결정할 것

- 화재 감지 시 소리 경보를 울릴지 결정
- 로봇을 즉시 정지시킬지 결정
- 감지 위치를 저장할지 결정
- 순찰 관리자에게 알림 토픽을 보낼지 결정
- RViz 또는 별도 UI에 경고를 표시할지 결정

## Workflow 7. 원버튼 자동 실행

### 목적

사용자가 복잡한 명령을 기억하지 않아도 `patrol_one_button.launch` 하나로 현재 상태에 맞는 모드를 실행한다.

### 실행

```bash
roslaunch night_patrol_robot patrol_one_button.launch
```

### 목표 동작

```text
저장된 맵 없음
-> mapping 모드 실행
-> SLAM + frontier 탐색
-> 맵 저장
-> home 복귀

저장된 맵 있음
-> patrol 모드 실행
-> map_server + AMCL + waypoint 순찰
-> home 복귀
```

### 확인할 것

- `mapping:=auto`에서 맵 파일 존재 여부를 기준으로 모드가 선택되는지 확인
- `mapping:=true`로 강제 맵핑이 되는지 확인
- `mapping:=false`로 강제 순찰이 되는지 확인
- RViz, 카메라 뷰어, 화재 감지를 arg로 끄고 켤 수 있는지 확인

## 권장 검증 순서

1. `office_patrol_nov4.world`에서 초기 맵핑 검증
2. 반복 실패 frontier가 완료 전환으로 빠지는지 확인
3. `maps/patrol_map.yaml` 저장 확인
4. 맵 저장 후 home 복귀 확인
5. `mapping:=false`로 저장 맵 순찰 검증
6. 순찰 완료 후 home 복귀 확인
7. 특정 시간 순찰 scheduler 설계
8. 화재 감지 후 경보/정지/알림 정책 결정
9. 원버튼 자동 실행 전체 시나리오 검증

## GitHub Issue 분리 기준

- 초기 맵핑 안정화
- 맵 저장 및 자동 전환
- 저장 맵 기반 waypoint 순찰
- home 복귀 동작
- 특정 시간 순찰 scheduler
- 화재 감지 후 대응 정책
- 원버튼 launch 통합 검증
