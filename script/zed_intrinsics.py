import pyzed.sl as sl

zed = sl.Camera()
zed.open()



calibration_params = zed.get_camera_information().camera_configuration.calibration_parameters
print(calibration_params)
# Focal length of the left eye in pixels
focal_left_x = calibration_params.left_cam.fx
# First radial distortion coefficient
k1 = calibration_params.left_cam.disto[0]
# Translation between left and right eye on x-axis
tx = calibration_params.stereo_transform.get_translation().get()[0]
# Horizontal field of view of the left eye in degrees
h_fov = calibration_params.left_cam.h_fov
tx = calibration_params.stereo_transform.get_translation().get()[0]
# Horizontal field of view of the left eye in degrees
h_fov = calibration_params.left_cam.h_fov

fx, fy, cx, cy = calibration_params.left_cam.fx, calibration_params.left_cam.fy, calibration_params.left_cam.cx, calibration_params.left_cam.cy
width, height = calibration_params.left_cam.image_size.width, calibration_params.left_cam.image_size.height

print(f"fx: {fx}, fy: {fy}, cx: {cx}, cy: {cy}"
      f", width: {width}, height: {height}")

[LEFT_CAM_HD]
fx=532.92
fy=532.945
cx=641.475
cy=361.982
k1=-0.0617541
k2=0.0416635
p1=-0.000341977
p2=0.000270675
k3=-0.0174146

[RIGHT_CAM_HD]
fx=533.895
fy=533.94
cx=647.57
cy=358.221
k1=-0.0652578
k2=0.0465106
p1=-0.000160699
p2=1.03514e-05
k3=-0.0193139

