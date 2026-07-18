"""
Enhanced Vision System for ROT3U Robotic Arm Pick and Place
Detects workspace calibration tags, object tag, and destination tags.

Author: Enhanced for Pick and Place functionality
"""

import cv2
import numpy as np
import os


# ###################################################
# Known workspace tag locations (cm) - These are the reference points
# ###################################################
KNOWN_WORKSPACE_TAGS = {
    0: (0, 0),      # Reference origin
    1: (65, 0),     # X-axis reference
    2: (0, 40),     # Y-axis reference
    3: (65, 40),    # Opposite corner
    5: (32.5, 20)   # Center point
}

# Object and destination tag IDs
OBJECT_TAG_ID = 10      # 5x5x5 cm cube with AprilTag
BASE_TAG_ID = 15        # Base reference tag on robot arm
DESTINATION_TAG_START = 20  # Starting ID for destination tags

# ###################################################
# AprilTag detector (OpenCV aruco AprilTag)
# ###################################################
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
PARAMETERS = cv2.aruco.DetectorParameters()
DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, PARAMETERS)


def open_camera_with_fallback(max_index=4):
    """
    Open camera with fallback to different backends.

    Args:
        max_index (int): Maximum camera index to try

    Returns:
        cv2.VideoCapture: Camera object or None if failed
    """
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


def pixel_to_world(H, u, v):
    """
    Convert pixel coordinates to world coordinates using homography matrix.

    Args:
        H (np.ndarray): 3x3 homography matrix
        u (float): pixel x coordinate
        v (float): pixel y coordinate

    Returns:
        tuple: (x, y) world coordinates in cm
    """
    if H is None:
        return None, None

    p = np.array([u, v, 1.0])
    w = H @ p
    w = w / w[2]  # Normalize
    return w[0], w[1]


def detect_tags(frame):
    """
    Detect AprilTags in the frame and return structured detection data.

    Args:
        frame (np.ndarray): Input image frame

    Returns:
        dict: Detection results containing:
            - 'workspace': dict of {tag_id: (x, y)} for known workspace tags
            - 'object': (x, y) position of object tag or None
            - 'base': (x, y) position of base tag or None
            - 'destinations': dict of {tag_id: (x, y)} for destination tags
            - 'homography': 3x3 homography matrix or None
            - 'frame': annotated frame with detections drawn
    """
    # Convert to grayscale for detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags
    corners_list, ids, _ = DETECTOR.detectMarkers(gray)

    # Initialize return data
    detections = {
        'workspace': {},
        'object': None,
        'base': None,
        'destinations': {},
        'homography': None,
        'frame': frame.copy()  # Work on a copy for drawing
    }

    # Process detected tags
    if ids is not None and len(ids) > 0:
        ids = ids.flatten()

        # Store all detections for drawing
        all_detections = {}

        for corners, tag_id in zip(corners_list, ids):
            tag_id = int(tag_id)
            corners = corners.reshape((4, 2))
            center = np.mean(corners, axis=0)

            # Store detection data
            all_detections[tag_id] = {
                'corners': corners,
                'center': center,
                'id': tag_id
            }

            # Categorize the tag
            if tag_id in KNOWN_WORKSPACE_TAGS:
                # This is a workspace calibration tag
                detections['workspace'][tag_id] = tuple(center)

            elif tag_id == OBJECT_TAG_ID:
                # This is the object we want to pick
                detections['object'] = tuple(center)

            elif tag_id == BASE_TAG_ID:
                # This is the base reference tag
                detections['base'] = tuple(center)

            elif tag_id >= DESTINATION_TAG_START:
                # This is a destination tag
                detections['destinations'][tag_id] = tuple(center)

    # Calculate homography if we have enough workspace tags
    if len(detections['workspace']) >= 4:
        # Prepare points for homography calculation
        pixel_points = []
        world_points = []

        for tag_id, pixel_pos in detections['workspace'].items():
            if tag_id in KNOWN_WORKSPACE_TAGS:
                pixel_points.append([pixel_pos[0], pixel_pos[1]])
                world_points.append(list(KNOWN_WORKSPACE_TAGS[tag_id]))

        if len(pixel_points) >= 4:
            pixel_array = np.array(pixel_points, dtype=np.float32)
            world_array = np.array(world_points, dtype=np.float32)
            homography, _ = cv2.findHomography(pixel_array, world_array)
            detections['homography'] = homography

    # Draw all detections on the frame
    for tag_id, detection in all_detections.items():
        corners = detection['corners'].astype(int)
        center = detection['center']

        # Draw tag boundary
        cv2.polylines(detections['frame'], [corners], True, (0, 255, 0), 2)
        cv2.circle(detections['frame'], (int(center[0]), int(center[1])), 4, (0, 0, 255), -1)

        # Label the tag based on its type
        label = f"ID:{tag_id}"
        if tag_id in KNOWN_WORKSPACE_TAGS:
            label += " (W)"
        elif tag_id == OBJECT_TAG_ID:
            label += " (OBJ)"
        elif tag_id == BASE_TAG_ID:
            label += " (BASE)"
        elif tag_id >= DESTINATION_TAG_START:
            label += f" (DEST-{tag_id})"

        cv2.putText(detections['frame'], label,
                   (int(center[0]) + 10, int(center[1])),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # If we have homography, show world coordinates for object/base/destinations
        if detections['homography'] is not None:
            world_x, world_y = pixel_to_world(detections['homography'], center[0], center[1])
            if world_x is not None and world_y is not None:
                coord_text = f"({world_x:.1f},{world_y:.1f})cm"
                cv2.putText(detections['frame'], coord_text,
                           (int(center[0]) + 10, int(center[1]) + 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

    return detections


def main():
    """Main function for testing the vision system."""
    print("Starting enhanced vision system for pick and place...")
    print("Press ESC to exit")

    # Initialize camera
    cap = open_camera_with_fallback()
    if cap is None:
        print("Error: Could not open any camera")
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to read frame")
                break

            # Detect tags in the frame
            detections = detect_tags(frame)

            # Print detection info
            if detections['object']:
                print(f"Object detected at: {detections['object']}")
            if detections['base']:
                print(f"Base detected at: {detections['base']}")

            # Show destination tags if any
            if detections['destinations']:
                dest_str = ", ".join([f"ID{k}:{v}" for k, v in detections['destinations'].items()])
                print(f"Destinations: {dest_str}")

            # Display the annotated frame
            cv2.imshow('Enhanced Vision System - Press ESC to exit', detections['frame'])

            # Exit on ESC key
            key = cv2.waitKey(1)
            if key == 27:  # ESC key
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()