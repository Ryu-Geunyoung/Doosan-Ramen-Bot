🍜 두산 E0509 자율 라면 조리 로봇 (Ramen Chef Bot)

이 프로젝트는 두산 로보틱스 E0509 협동 로봇을 이용하여 라면 사리 투하부터 조리 중 면 괴기(Aeration), 퍼포먼스(Dance), 그리고 최종 서빙까지 전 과정을 자동화한 시스템입니다.

특히 조리 중간에 면발을 공기 중에 노출시켜 쫄깃함을 더하는 '괴면' 기술과 조리 완료 후 안전한 서빙 모션에 최적화되어 있습니다.

🚀 주요 기능

Phase 1: 면 파지 및 투하 - 라면 사리를 정확히 집어 냄비에 넣습니다.

Phase 2 & 7: 셰프 댄스 - 조리 대기 시간 동안 지루하지 않게 좌우 흔들기, 끄덕이기 등 퍼포먼스를 수행합니다.

Phase 3 & 5: 면 휘젓기 - MoveC 원호 보간을 이용해 면이 뭉치지 않게 휘젓습니다.

Phase 4 & 6: 면 괴기 (Aeration) - 면을 공기 중에 들어 올려 식혀주는 정밀 관절 제어를 수행합니다.

Phase 8: 서빙 (Serving) - 조리가 끝난 냄비를 배달 구역으로 옮깁니다.

특이사항: 테이블에 내려놓을 때의 충격을 방지하기 위해 Z축 높이를 미세 조정하여 부드러운 안착을 구현했습니다.

🛠️ 개발 환경

Robot: Doosan Robotics E0509

OS: Ubuntu 22.04 LTS (추천)

Framework: ROS 2 Humble

Library: dsr_msgs2, DSR_ROBOT2, GripperController (Custom)

Language: Python 3

📂 파일 구조

test3.py: 메인 조리 알고리즘 및 로봇 모션 로직이 담긴 실행 파일

gripper_drl_controller.py: 그리퍼 제어를 위한 컨트롤러 클래스

⚙️ 설정 및 실행

1. Git 설정 (최초 1회)

커밋 기록을 남기기 위해 사용자 정보를 설정해야 합니다.

git config --global user.email "chaosdarkc@gmail.com"
git config --global user.name "Ryu-Geunyoung"


2. 로봇 드라이버 실행 (Real Robot)

ros2 launch dsr_bringup2 dsr_bringup2.launch.py model:=e0509 host:=110.120.1.6 mode:=real


3. 프로그램 실행

# 환경 설정 로드 후
source ~/test_ws/install/setup.bash
python3 test3.py


⚠️ 주의사항 및 튜닝

조리 시간: 현재 SHIN_RAMYUN_TIME은 270초(4분 30초)로 설정되어 있습니다. 로봇의 서빙 동선을 고려하여 실 시식 시간은 약 5분 내외가 됩니다.

서빙 높이: p_delivery_place 좌표는 테이블 바닥에서 약 1cm 정도 여유를 두고 냄비를 놓도록 튜닝되어 있습니다. (눌러버리는 현상 방지)

그리퍼 파지력: 냄비 서빙 시에는 GRIPPER_POT_CLOSE = 1000의 최대 파지력을 사용하여 안전을 확보합니다.

👨‍💻 Author

Name: Ryu-Geunyoung

GitHub: Ryu-Geunyoung

Contact: chaosdarkc@gmail.com