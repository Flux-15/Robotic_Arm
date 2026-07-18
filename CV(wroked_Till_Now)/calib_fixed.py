import cv2
import numpy as np
import glob
import os

# SETTINGS
PATTERN = (8,5)  # inner corners (cols, rows)
SQUARE_SIZE = 30.0  # mm

# Prepare object points
objp = np.zeros((PATTERN[0]*PATTERN[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []
imgpoints = []

# Paths
script_dir = os.path.dirname(os.path.abspath(__file__))
check_pix_dir = os.path.join(script_dir, "Check_pix")
image_pattern = os.path.join(check_pix_dir, "*.jpg")
images = glob.glob(image_pattern)

print(f"Found {len(images)} images in {check_pix_dir}")

if len(images) == 0:
    print("No images found. Put chessboard photos into the Check_pix folder and retry.")
    exit(1)

image_shape = None

for f in images:
    img = cv2.imread(f)
    if img is None:
        print(f"Warning: can't read {f}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    image_shape = gray.shape[::-1]

    # Try SB detector first
    ret, corners = cv2.findChessboardCornersSB(gray, PATTERN)
    if not ret:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        ret, corners = cv2.findChessboardCorners(gray, PATTERN, flags)
        if ret:
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )

    if ret:
        objpoints.append(objp.copy())
        imgpoints.append(corners)
        vis = img.copy()
        cv2.drawChessboardCorners(vis, PATTERN, corners, ret)
        cv2.imshow("Corners", vis)
        cv2.waitKey(200)
        print("Detected:", os.path.basename(f))
    else:
        print("No pattern:", os.path.basename(f))

cv2.destroyAllWindows()

if len(objpoints) == 0:
    print("No chessboard detections. Calibration cannot proceed.")
    exit(1)

if image_shape is None:
    print("Could not determine image shape. Calibration cannot proceed.")
    exit(1)

# Calibrate
ret, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, image_shape, None, None)

print("\nCalibration complete")
print("Camera matrix:\n", K)
print("Distortion:\n", D)
print("Reprojection error:\n", ret)

out_file = os.path.join(script_dir, "camera.npz")
np.savez(out_file, K=K, D=D)
print(f"Saved {out_file}")

# Undistort preview using first readable image
first_img = None
for f in images:
    first_img = cv2.imread(f)
    if first_img is not None:
        break

if first_img is not None:
    h, w = first_img.shape[:2]
    newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    und = cv2.undistort(first_img, K, D, None, newK)
    cv2.imshow("Original", first_img)
    cv2.imshow("Undistorted", und)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Could not load an image to preview undistortion.")
