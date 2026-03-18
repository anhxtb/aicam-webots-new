from controller import Robot, Camera
import math

BASELINE_CM     = 50.0
CONVERGENCE_DEG = 10.0
CAM_FOV_DEG     = 60.0
TIMESTEP        = 32

robot     = Robot()
cam_left  = robot.getDevice("cam_left")
cam_right = robot.getDevice("cam_right")
cam_left.enable(TIMESTEP)
cam_right.enable(TIMESTEP)

W = cam_left.getWidth()
focal_px = W / (2 * math.tan(math.radians(CAM_FOV_DEG / 2)))
vergence = BASELINE_CM / (2 * math.tan(math.radians(CONVERGENCE_DEG)))

print(f"[AICAM] Baseline={BASELINE_CM}cm | FOV={CAM_FOV_DEG}° | Vergence~{vergence:.0f}cm")

frame = 0
while robot.step(TIMESTEP) != -1:
    frame += 1
    if frame % 30 != 0:
        continue

    img_l = cam_left.getImage()
    img_r = cam_right.getImage()

    # Tìm cx của vật sáng nhất
    def find_cx(img, w, h):
        best, bx = 0, None
        for x in range(0, w, 8):
            for y in range(h//3, h*2//3, 8):
                i = (y * w + x) * 4
                if i + 2 < len(img):
                    br = img[i] + img[i+1] + img[i+2]
                    if br > best:
                        best, bx = br, x
        return bx if best > 200 else None

    H = cam_left.getHeight()
    cx_l = find_cx(img_l, W, H)
    cx_r = find_cx(img_r, W, H)

    if cx_l and cx_r:
        cx    = W / 2.0
        theta = math.radians(CONVERGENCE_DEG)
        aL    = math.atan2(cx_l - cx, focal_px)
        aR    = math.atan2(cx_r - cx, focal_px)
        denom = math.tan(theta + aL) - math.tan(aR - theta)
        if abs(denom) > 1e-6:
            Z = BASELINE_CM / denom
            if 20 < Z < 500:
                print(f"[Frame {frame:04d}] cx_L={cx_l} cx_R={cx_r} => Z={Z:.1f} cm")
    else:
        print(f"[Frame {frame:04d}] Khong tim thay vat the")
