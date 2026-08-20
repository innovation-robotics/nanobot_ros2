import cv2
import numpy as np
import urllib.request

# ================= CONFIGURATION =================
# 1. Update with your ESP32-CAM Stream URL
STREAM_URL = "http://192.168.1.5:81/stream"  

# 2. Count internal grid intersections on your checkerboard!
# (e.g., an 8x6 square board usually has 7x5 internal corners)
CHECKERBOARD = (7, 5)  

# 3. Real physical square size in meters (e.g., 0.025 = 2.5 cm)
SQUARE_SIZE = 0.027  
# =================================================

# Termination criteria for sub-pixel accuracy
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare 3D object points (0,0,0), (1,0,0), (2,0,0) ... scaled by SQUARE_SIZE
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

objpoints = []  # 3d points in real world space
imgpoints = []  # 2d points in image plane

# Open stream connection
stream = urllib.request.urlopen(STREAM_URL)
bytes_data = b''

captured_count = 0
print("\n--- ESP32-CAM CALIBRATION TOOL ---")
print("1. Hold the board steady at different positions and angles.")
print("2. Press 'SPACEBAR' to capture a frame (Aim for 15-20 frames).")
print("3. Press 'ENTER' to compute calibration and finish.")
print("4. Press 'ESC' to exit without saving.\n")

while True:
    bytes_data += stream.read(1024)
    a = bytes_data.find(b'\xff\xd8') # JPEG start
    b = bytes_data.find(b'\xff\xd9') # JPEG end
    
    if a != -1 and b != -1:
        jpg = bytes_data[a:b+2]
        bytes_data = bytes_data[b+2:]
        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        display_frame = frame.copy()

        if ret:
            # Draw overlay corners in green/red
            cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners, ret)
            cv2.putText(display_frame, "Board Detected! Press SPACE to capture", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(display_frame, "Board Not Detected", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.putText(display_frame, f"Captures: {captured_count}/20", (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.imshow("ESP32-CAM Calibration", display_frame)
        key = cv2.waitKey(1)

        # SPACEBAR: Capture current frame
        if key == 32:
            if ret:
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                objpoints.append(objp)
                imgpoints.append(corners2)
                captured_count += 1
                print(f"Captured frame #{captured_count}")
            else:
                print("Cannot capture: Checkerboard corners not clearly visible!")

        # ENTER: Run calibration math
        elif key == 13:
            if captured_count < 10:
                print("Warning: Recommend taking at least 10-15 captures for good accuracy!")
            else:
                break

        # ESC: Quit
        elif key == 27:
            print("Exiting without saving.")
            cv2.destroyAllWindows()
            exit()

cv2.destroyAllWindows()

print("\nProcessing calibration math...")
h, w = gray.shape[:2]
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, None)

print("\n================ CALIBRATION RESULTS ================")
print("Camera Matrix (K):")
print(camera_matrix)
print("\nDistortion Coefficients (D):")
print(dist_coeffs)
print("======================================================")

# Save to file for your ArUco localization node!
np.savez("esp32_cam_calibration.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
print("\nSaved calibration values to 'esp32_cam_calibration.npz'!")