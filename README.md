# Car-like Robot Simulation __Dummy2__

<img src = "https://github.com/user-attachments/assets/30c32d0c-ce11-4ea5-bdbf-c6d86a9e6d4f" width="430" height="400"> <img src = "https://github.com/user-attachments/assets/80edad36-e67c-43a0-a945-7c514f113920" width="430" height="400">

## 프로젝트 소개
__Dummy2__ 는 Car-like 로봇을 Garzebo에 구현해 Nav2 패키지를 통한 navigation과 SLAM_TOOLBOX 패키지를 통한 SLAM을 시뮬레이션 할 수 있도록 해주는 패키지입니다. 추가적으로 __자율 탐색__ 을 위해 제작중인 node가 포함되어 있습니다.  

현재까지 이 프로젝트의 목표는 노드의 실행만으로 Occupancy Grid Map 의 데이터를 활용해 조건에 부합하는 목표를 설정 후 이동하며 SLAM을 통한 지도 작성을 완수해내는 것 입니다.  

(수정예정)
[3배속 재생입니다.](https://github.com/user-attachments/assets/9495880b-95de-4f5e-b62c-b21c66504839)


후진할 때의 경로 이탈 발생과 중복되는 지역 탐색 등 개선해야 할 부분이 많지만, 오히려 개선 할 부분을 찾고 해결방안이 떠오르는 것을 다행이 생각하고 이를 해결하려고 노력할 것입니다.  

## 개발 기간
* 2023.6 ~

## 패키지 구성
* `ackermann_gazebo_plugin` : Gazebo 시뮬레이션 환경에서 Car-like Robot의 기능과 움직임을 구현하기 위한 Gazebo 플러그인 입니다.  
* `dummy2_gazebo` : 시뮬레이션을 위한 launch 파일이 포함된 패키지 입니다. 이외에 로봇의 모델링을 담은 urdf 파일과 시뮬레이션할 World, 패키지 및 플러그인에서 필요로하는 매개변수가 담긴 yaml 파일과 Node가 포함되어 있습니다.

* [Nav2_ackermann_controller](https://github.com/Gone030/nav2_ackermann_controller) : Planner 에서 제공되는 경로를 따라서 Car-like 동작 방식을 가진 로봇을 제어하기 위해 제작된 Custom Controller 입니다.

  
