
import cv2
import time

camera = None

def init(camera_id=0, width=640, height=480, buffer_size=1):
    global camera
    try:
        if camera is not None:
            camera.release()
            camera = None
            time.sleep(1)

        print(f"카메라 초기화 시도 (device: /dev/video{camera_id})")
        camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L)
        if not camera.isOpened():
            print("카메라를 열 수 없습니다!")
            return False

        # 카메라 설정
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, buffer_size)

        # 테스트 프레임
        success, _ = camera.read()
        if not success:
            print("테스트 프레임 읽기 실패")
            return False

        print("카메라 초기화 성공!")
        return True

    except Exception as e:
        print(f"카메라 초기화 오류: {e}")
        return False

def take_picture(most_recent=False):
    global camera
    if not camera or not camera.isOpened():
        return None

    # 버퍼 비우기
    if most_recent:
        buffer_len = int(camera.get(cv2.CAP_PROP_BUFFERSIZE))
        while buffer_len > 0:
            camera.grab()
            buffer_len -= 1

    success, image = camera.read()
    return image if success else None

def final():
    global camera
    if camera:
        camera.release()
    camera = None
