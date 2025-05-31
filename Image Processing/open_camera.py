import cv2
import numpy as np

# 0: 내장, 1: USB로 연결된 외장
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # Windows라면 CAP_DSHOW 옵션이 빠릅니다

#해상도 설정
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame Error")
        break
    
    #BGR to RGB
    #rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


    # 프레임 출력
    cv2.imshow("Camera Feed", frame)

    # 1ms 대기 후 'q' 누르면 루프 탈출
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 해제 및 윈도우 닫기
cap.release()
cv2.destroyAllWindows()
