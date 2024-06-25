2024/5/29 urdf변형한 것 urdf 디렉터리에 존재 

nodelet 의존성 패키지
sudo apt-get install ros-melodic-usb-cam
sudo apt-get install pkg-config
sudo apt-get install libusb-1.0-0-dev
pip install rospkg


2024/6/24 12:00 nodelet 업로드 hello_world.launch를 라즈베리파이에서 실행
2024/6/24 12:40 launch/reciever.launch 업로드 -> 수신 받는 컴퓨터에서 실행
CMAKE를 하기 위해 CMakeLists.txt의 INCLUDE_DIRS include를 주석 처리 함 추후 카메라 문제 발생하면 봐야함

2024/6/24 16:45 src/yolo_processor.py, launch/yolo_processor.launch 생성 + PC의 CMakeLists.txt와 package.xml에 python, opencv, ultralytics 의존성 추가

2024/6/25 10:15 python-catkin-tools, python3-dev, python3-numpy 설치 -> 안되면 cv_bridge 직접 설치

주의 사항 : ros_master_uri, ros_ip 설정 필요(~/.bashrc), 사양 문제로 gpu가 아닌 cpu로 인식
입력 토픽 : /usb_cam/image_raw, 출력 토픽 : /yolo_output

2024/6/25 12:30 yolo_processor2.py를 통해 노드로 영상이 아닌 객체와 정확도만 송출, yolo_processor.launch 파일 수정(type만 변경하면 변환 가능)
입력 토픽 : /usb_cam/image_raw, 출력 토픽 : /yolo_results