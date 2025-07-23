import cv2
import numpy as np

qrcode_len = 2.35 # 邮件表面二维码边长
standard_translation_vector = np.array([[-3.5], [-1.0], [25.0]], dtype=np.float32)
camera_intrinsic = {
    "mtx": np.array([[539.4118538, 0.00000000e+00, 321.1254354],
                     [0.00000000e+00, 540.02901063, 234.35062125],
                     [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]],
                    dtype=np.double),
    "dist": np.array([[-0.06488393, 0.14645391, -0.0042577, -0.00099087, 0.01829749]], dtype=np.double),
}

def pnp(img_points):
    # 根据二维码角点坐标计算error_x和error_y
    img_points = img_points[0]
    temp_point = img_points[2]
    img_points[2] = img_points[3]
    img_points[3] = temp_point
    object_points = np.array([[0, 0, qrcode_len], [0, qrcode_len, qrcode_len], [0, 0, 0], [0, qrcode_len, 0]], dtype=np.float32)
    image_points = np.array(img_points, dtype=np.float32)
    success, rotation_vector, translation_vector = cv2.solvePnP(object_points, image_points, camera_intrinsic["mtx"], camera_intrinsic["dist"])
    if not success:
        return None
    error_x = translation_vector[0] - standard_translation_vector[0]
    error_y = translation_vector[2] - standard_translation_vector[2]
    return error_x, error_y