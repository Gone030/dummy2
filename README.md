# Car-like Robot Simulation __Dummy2__

<img src = "https://github.com/user-attachments/assets/30c32d0c-ce11-4ea5-bdbf-c6d86a9e6d4f" width="430" height="400"> <img src = "https://github.com/user-attachments/assets/80edad36-e67c-43a0-a945-7c514f113920" width="430" height="400">

## 프로젝트 소개
__Dummy2__ 는 Car-like 로봇을 Garzebo에 구현해 Nav2 패키지를 통한 navigation과 SLAM_TOOLBOX 패키지를 통한 SLAM을 시뮬레이션 할 수 있도록 해주는 패키지입니다.
추가적으로 아직 탐색되지 않은 미지의 구역에서 동작하기 위해 스스로 지도를 작성하는 SLAM 기반의 알고리즘이 본 패키지의 `exploration` 노드로 추가되어 있습니다.

시뮬레이션 실험을 통해 이전 빌드의 지도작성까지 소요시간에 비해 약 두 배 빠르게 개선되었습니다.


![2025_04_08_05_27_PM_1744100825](https://github.com/user-attachments/assets/09a389c6-2f8b-4081-8f22-b38696f34b5c)

## 개발 기간
* 2023.6 ~ 2025.04

## 패키지 구성
* `exploration` : __Dummy2__ 패키지에서 가장 핵심적인 기능이며 최초 실행시 생성되는 미지의 구역에서 최적의 목표를 설정하고 이동하며 지도를 확장 및 탐색하도록 합니다.

* `ackermann_gazebo_plugin` : Gazebo 시뮬레이션 환경에서 Car-like Robot의 기능과 움직임을 구현하기 위한 Gazebo 플러그인 입니다.  

* `dummy2_gazebo` : 시뮬레이션을 위한 launch 파일이 포함된 패키지 입니다. 이외에 로봇의 모델링을 담은 urdf 파일과 시뮬레이션할 World, 패키지 및 플러그인에서 필요로하는 매개변수가 담긴 yaml 파일과 Node가 포함되어 있습니다.

* [Nav2_ackermann_controller](https://github.com/Gone030/nav2_ackermann_controller) : Planner 에서 제공되는 경로를 따라서 Car-like 동작 방식을 가진 로봇을 제어하기 위해 제작된 Custom Controller 입니다.


## 주요 동작 설명
### exploraition
지도가 `Occupancy_grid_map` 으로 생성되고 map의 data가 SLAM을 통해 정해질 때 `이동이 가능한 탐색완료된 점`과 `미탐색 점`, `이동이 불가능한 탐색완료된 점`(장애물) 세 종류로 나뉜다는 점에서 착안해, Quadtree 알고리즘으로 `미탐색 점`의 점유율에 따라 셀을 분할하고, 로봇이 이동함에 따라 변동된 점의 종류 및 점유율에 따라 셀을 병합합니다.

각각의 분할된 셀을 차지하고 있는 점들의 점유율에 따라서 셀의 점수를 부여하고, 비교함으로써 현재 설정된 조건에 따른 가장 최적의 목표를 선정해 로봇을 이동시킵니다.

동작은 지도 작성이 완료될 때 까지 반복되며, 지도 내 모든 구역에 대한 탐색이 완료되면 종료됩니다.


https://github.com/user-attachments/assets/5f58a3b6-afb6-4ced-b3fb-a5d5a8e34afb

