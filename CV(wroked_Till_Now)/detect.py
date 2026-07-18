import cv2
import numpy as np


aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

def open_camera_with_fallback(max_index=4):
    # Windows often works better with DirectShow than MSMF for OpenCV capture.
    backends = [
        ("DSHOW", cv2.CAP_DSHOW),
        ("MSMF", cv2.CAP_MSMF),
        ("ANY", cv2.CAP_ANY),
    ]

    for index in range(max_index + 1):
        for backend_name, backend in backends:
            cap = cv2.VideoCapture(index, backend)
            if cap.isOpened():
                ok, frame = cap.read()
                if ok and frame is not None:
                    print(f"Opened camera index={index} backend={backend_name}")
                    return cap
                cap.release()
            else:
                cap.release()

    return None


cap = open_camera_with_fallback()

if cap is None:
    print("Error: could not open any camera feed.")
    print("Tip: close apps using the camera (Teams/Zoom/Browser) and check Windows Camera privacy settings.")
    exit(1)

while True:

    ret, frame = cap.read()

    if not ret:
        print("Error: failed to read frame")
        break

    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    corners_list, ids, _ = detector.detectMarkers(gray)

    if ids is not None and len(ids) > 0:
        ids = ids.flatten()

        for corners, tag_id in zip(corners_list, ids):
            corners = corners.reshape((4, 2)).astype(int)
            center = np.mean(corners, axis=0).astype(int)

            cv2.polylines(
                frame,
                [corners],
                True,
                (0,255,0),
                2
            )

            cv2.circle(
                frame,
                tuple(center),
                5,
                (0,0,255),
                -1
            )

            cv2.putText(

                frame,

                f'ID:{int(tag_id)}',

                (center[0]+10,
                 center[1]),

                cv2.FONT_HERSHEY_SIMPLEX,

                0.7,

                (255,0,0),

                2
            )

            print(

                "ID:",
                int(tag_id),

                "Center:",
                center.tolist()
            )


    cv2.imshow(
        "AprilTags",
        frame
    )


    if cv2.waitKey(1)==27:
        break


cap.release()
cv2.destroyAllWindows()