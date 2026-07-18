import cv2
import numpy as np
import glob
import os


# =====================================================
# SETTINGS
# =====================================================

PATTERN = (8,5)
SQUARE_SIZE = 30.0     # mm


# =====================================================
# OBJECT POINTS
# =====================================================

objp = np.zeros((PATTERN[0]*PATTERN[1],3), np.float32)

objp[:,:2] = np.mgrid[
                0:PATTERN[0],
                0:PATTERN[1]
            ].T.reshape(-1,2)

objp *= SQUARE_SIZE



objpoints=[]
imgpoints=[]



# =====================================================
# LOAD IMAGES
# =====================================================


script_dir = os.path.dirname(os.path.abspath(__file__))
folder = os.path.join(script_dir, "Check_pix")

images = glob.glob(os.path.join(folder, "*.jpg"))

print()
print("Found", len(images), "images in", folder)
print()


image_shape = None


# =====================================================
# DETECT CORNERS
# =====================================================


for file in images:

    img = cv2.imread(file)
    if img is None:
        print("Warning: could not read", file)
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    image_shape = gray.shape[::-1]

    # First try the robust SB detector, then fall back to the legacy detector
    ret, corners = cv2.findChessboardCornersSB(gray, PATTERN)
    if not ret:
        # try legacy detector with common flags and subpixel refinement
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
        cv2.waitKey(300)

        print("✓", os.path.basename(file))
    else:
        print("✗", os.path.basename(file))



cv2.destroyAllWindows()



# =====================================================
# CALIBRATE
# =====================================================


ret,K,D,rvecs,tvecs = cv2.calibrateCamera(


                            objpoints,
                            imgpoints,
                            image_shape,

                            None,
                            None

                            )



# =====================================================
# RESULTS
# =====================================================


print()
print("================================")
print("CALIBRATION COMPLETE")
print("================================")
print()



print("Camera Matrix")
print()

print(K)


print()
print()


print("Distortion")
print()

print(D)



print()
print()

print("Reprojection Error")
print()

print(ret)


# =====================================================
# SAVE
# =====================================================



np.savez(

        "camera.npz",

        K=K,
        D=D

        )



print()
print("Saved camera.npz")



# =====================================================
# TEST UNDISTORTION
# =====================================================


img=cv2.imread(images[0])


h,w=img.shape[:2]



newK,_=cv2.getOptimalNewCameraMatrix(

            K,
            D,
            (w,h),
            1,
            (w,h)

            )




und = cv2.undistort(

            img,

            K,
            D,

            None,

            newK

            )



cv2.imshow("Original",img)
cv2.imshow("Undistorted",und)

cv2.waitKey(0)

cv2.destroyAllWindows()