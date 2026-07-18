import cv2
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
check_pix_dir = os.path.join(script_dir, "Check_pix")

# Display the first image for inspection
first_image = os.path.join(check_pix_dir, "WIN_20260625_14_05_52_Pro.jpg")

if os.path.exists(first_image):
    img = cv2.imread(first_image)
    if img is not None:
        print(f"Image shape: {img.shape}")
        print(f"Image path: {first_image}")
        print("\nDisplaying image. Press any key to close.")
        cv2.imshow("Image Inspector", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print(f"Could not read image: {first_image}")
else:
    print(f"Image not found: {first_image}")
