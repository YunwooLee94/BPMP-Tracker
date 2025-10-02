import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import rosbag
from tf.transformations import euler_from_quaternion
from tqdm import tqdm


def get_grid_lineset(h_min_val, h_max_val, w_min_val, w_max_val, ignore_axis, grid_length=1, nth_line=5):
    if (h_min_val%2!=0):
        h_min_val -= 1
    if (h_max_val%2!=0):
        h_max_val += 1
    if (w_min_val%2!=0):
        w_min_val -= 1
    if (w_max_val%2!=0):
        w_max_val += 1
    
    num_h_grid = int(np.ceil((h_max_val - h_min_val) / grid_length))
    num_w_grid = int(np.ceil((w_max_val - w_min_val) / grid_length))
    
    num_h_grid_mid = num_h_grid // 2
    num_w_grid_mid = num_w_grid // 2
    
    grid_vertexes_order = np.zeros((num_h_grid, num_w_grid)).astype(np.int16)
    grid_vertexes = []
    vertex_order_index = 0
    
    for h in range(num_h_grid):
        for w in range(num_w_grid):
            grid_vertexes_order[h][w] = vertex_order_index
            if ignore_axis == 0:
                grid_vertexes.append([0, grid_length*w + w_min_val, grid_length*h + h_min_val])
            elif ignore_axis == 1:
                grid_vertexes.append([grid_length*h + h_min_val, 0, grid_length*w + w_min_val])
            elif ignore_axis == 2:
                grid_vertexes.append([grid_length*w + w_min_val, grid_length*h + h_min_val, 0])
            else:
                pass                
            vertex_order_index += 1       
            
    next_h = [0, 1]
    next_w = [1, 0]
    grid_lines = []
    grid_nth_lines = []
    for h in range(num_h_grid):
        for w in range(num_w_grid):
            here_h = h
            here_w = w
            for i in range(2):
                there_h = h + next_h[i]
                there_w = w +  next_w[i]   
                if (0 <= there_h and there_h < num_h_grid) and (0 <= there_w and there_w < num_w_grid):
                    if ((here_h % nth_line) == 0) and ((here_w % nth_line) == 0):
                        grid_nth_lines.append([grid_vertexes_order[here_h][here_w], grid_vertexes_order[there_h][there_w]])
                    elif ((here_h % nth_line) != 0) and ((here_w % nth_line) == 0) and i == 1:
                        grid_nth_lines.append([grid_vertexes_order[here_h][here_w], grid_vertexes_order[there_h][there_w]])
                    elif ((here_h % nth_line) == 0) and ((here_w % nth_line) != 0) and i == 0:
                        grid_nth_lines.append([grid_vertexes_order[here_h][here_w], grid_vertexes_order[there_h][there_w]])
                    else:
                        grid_lines.append([grid_vertexes_order[here_h][here_w], grid_vertexes_order[there_h][there_w]])

    color = (0.8, 0.8, 0.8)
    colors = [color for i in range(len(grid_lines))]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(grid_vertexes),
        lines=o3d.utility.Vector2iVector(grid_lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)
    
    color = (255, 0, 0)
    colors = [color for i in range(len(grid_nth_lines))]
    line_nth_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(grid_vertexes),
        lines=o3d.utility.Vector2iVector(grid_nth_lines),
    )
    line_nth_set.colors = o3d.utility.Vector3dVector(colors)
    
    return line_set, line_nth_set

def show_open3d_pcd(raw, show_origin=True, origin_size=3, 
                    show_grid=True, grid_len=1, 
                    voxel_size=0, 
                    range_min_xyz=(-80, -80, -80), range_max_xyz=(80, 80, 80)):
    '''
    - raw : numpy 2d array (size : (n, 3)) or o3d.geometry.PointCloud
    - show_origin : show origin XYZ coordinate. (X=red, Y=green, Z=Blue)
    - origin_size : size of origin coordinate.
    - show_grid : if true, show grid in xy, yz, zx plane with 'grid_len' length (default : gray line) and 5 times of 'grid_len' (default : red line)
    - voxel_size : voxel size to downsampling
    - range_min_xyz : grid min range of xyz orientation
    - range_max_xyz : grid max range of xyz orientation

    '''
    pcd = o3d.geometry.PointCloud()    
    
    if isinstance(raw, type(pcd)):
        pass
    elif isinstance(raw, np.ndarray):
        pcd.points = o3d.utility.Vector3dVector(raw)        
    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        
    pcd_point = np.array(pcd.points)
    inrange_inds = (pcd_point[:, 0] > range_min_xyz[0]) & \
                    (pcd_point[:, 1] > range_min_xyz[1]) & \
                    (pcd_point[:, 2] > range_min_xyz[2]) & \
                    (pcd_point[:, 0] < range_max_xyz[0]) & \
                    (pcd_point[:, 1] < range_max_xyz[1]) & \
                    (pcd_point[:, 2] < range_max_xyz[2])    
    
    pcd_point = pcd_point[inrange_inds]
    filtered_raw = pcd_point
    pcd.points = o3d.utility.Vector3dVector(filtered_raw)
        
    x_min_val, y_min_val, z_min_val = range_min_xyz
    x_max_val, y_max_val, z_max_val = range_max_xyz
    
    coord = o3d.geometry.TriangleMesh().create_coordinate_frame(size=origin_size, origin=np.array([0.0, 0.0, 0.0]))
    
    ##################################### grid 생성 코드 ######################################
    lineset_yz, lineset_nth_yz = get_grid_lineset(z_min_val, z_max_val, y_min_val, y_max_val, 0, grid_len)
    lineset_zx, lineset_nth_zx = get_grid_lineset(x_min_val, x_max_val, z_min_val, z_max_val, 1, grid_len)
    lineset_xy, lineset_nth_xy = get_grid_lineset(y_min_val, y_max_val, x_min_val, x_max_val, 2, grid_len) 
    ###########################################################################################
    
    # set front, lookat, up, zoom to change initial view
    if show_origin and show_grid:
        o3d.visualization.draw_geometries([pcd, coord,
                                           lineset_nth_yz, lineset_nth_zx, lineset_nth_xy,
                                           lineset_xy, lineset_yz, lineset_zx
                                          ])  
    elif show_origin and not show_grid:
        o3d.visualization.draw_geometries([pcd, coord])
    elif not show_origin and show_grid:
        o3d.visualization.draw_geometries([pcd,
                                           lineset_nth_yz, lineset_nth_zx, lineset_nth_xy,
                                           lineset_xy, lineset_yz, lineset_zx
                                          ])  
    else:
        o3d.visualization.draw_geometries([pcd])  

def pose2d_to_mat4(x, y, yaw):
    """
    2D pose (x, y, yaw) → 4x4 homogeneous transform (SE3)
    yaw 단위: rad
    """
    c, s = np.cos(yaw), np.sin(yaw)
    T = np.eye(4)
    T[0,0], T[0,1] = c, -s
    T[1,0], T[1,1] = s,  c
    T[0,3], T[1,3] = x, y
    return T

# ===============================
# 1. 데이터 로드 (사용자 준비)
# ===============================
# 예시: rgb_list, depth_list, vo_poses는 미리 준비된 리스트
#  - rgb_list: (H, W, 3) numpy array
#  - depth_list: (H, W) numpy array, 사람 영역은 mask된 depth
#  - vo_poses: 각 프레임의 VO pose (4x4 numpy array)

rgb_list = []      # 사용자 데이터
rgb_time = []  # 사용자 데이터
depth_list = []    # 사용자 데이터
depth_time = []  # 사용자 데이터
vo_poses = []      # VO trajectory (각각 4x4 pose)
vo_time = []  # 사용자 데이터

bagfile = "/media/seungwoo/T5_JSW/rosbag_alchemist_2025_08_01/office/1/2025-08-01-20-10-58.bag"
odom_topic = "/zed2/zed_node/odom"
depth_topic = "/zed_client/image_depth_masked"
rgb_topic = "/zed2/zed_node/rgb/image_rect_color"

# fig, ax = plt.subplots(1, 1)
# plt.ion()
# ax.set_title("Depth Image")
# ax.set_xlabel("X")
# ax.set_ylabel("Y")
# img_plot = ax.imshow(np.zeros((480, 640)), cmap='gray')
# colorbar = plt.colorbar(img_plot, ax=ax)
# plt.pause(0.1)  # 잠시 표시

with rosbag.Bag(bagfile) as bag:
    for topic, msg, t in tqdm(bag.read_messages(topics=[rgb_topic, depth_topic, odom_topic])):
        if topic == rgb_topic:
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            rgb_list.append(rgb)
            rgb_time.append(t)
        elif topic == depth_topic:
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            # img_plot.set_data(depth)
            # img_plot.set_clim(vmin=np.nanmin(depth), vmax=np.nanmax(depth))
            # plt.pause(0.001)
            depth_list.append(depth)
            depth_time.append(t)
        elif topic == odom_topic:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = 0  # 2D이므로 z=0 고정

            # orientation quaternion -> yaw
            q = msg.pose.pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            roll, pitch, yaw = euler_from_quaternion(quat)
            T = pose2d_to_mat4(x, y, yaw)
            vo_poses.append(T)
            vo_time.append(t)

# 시간 동기화 (가장 가까운 시간의 depth/rgb/vo 매칭)
synced_rgb = []
synced_depth = []
synced_vo_poses = []
for i in range(len(depth_time)):
    depth_t = depth_time[i]
    # 가장 가까운 rgb 찾기
    rgb_diffs = [abs((depth_t - rt).to_sec()) for rt in rgb_time]
    rgb_idx = np.argmin(rgb_diffs)
    # 가장 가까운 vo 찾기
    vo_diffs = [abs((depth_t - vt).to_sec()) for vt in vo_time]
    vo_idx = np.argmin(vo_diffs)

    synced_rgb.append(rgb_list[rgb_idx])
    synced_depth.append(depth_list[i])  # depth는 현재 인덱스 사용
    synced_vo_poses.append(vo_poses[vo_idx])

rgb_list = synced_rgb
depth_list = synced_depth
vo_poses = synced_vo_poses

# ZED intrinsics (예시 값, 실제 카메라 파라미터로 교체하세요)
# fx, fy, cx, cy = 350, 350, 320, 240
fx=532.92
fy=532.945
cx=641.475
cy=361.982
width, height = 640, 360
intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

# ===============================
# 2. ICP 기반 trajectory 보정
# ===============================
def refine_trajectory_with_icp(rgb_list, depth_list, vo_poses, intrinsic, voxel_size=0.01, keyframe_step=5):
    refined_poses = [vo_poses[0]]
    residual = []
    prev_pcd = None
    prev_idx = 0

    # 첫 번째 프레임으로 prev_pcd 초기화
    rgb_o3d = o3d.geometry.Image(rgb_list[0].astype(np.uint8))
    depth_o3d = o3d.geometry.Image(depth_list[0].astype(np.float32))
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgb_o3d, depth_o3d, depth_scale = 1.0, depth_trunc=5.0,
        convert_rgb_to_intensity=False
    )
    prev_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

    # visualize
    show_open3d_pcd(prev_pcd, show_origin=False, show_grid=False)

    prev_pcd = prev_pcd.voxel_down_sample(voxel_size)

    show_open3d_pcd(prev_pcd, show_origin=False)
    # Normal vector 계산
    prev_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    for i in range(1, len(rgb_list)):
        # RGBD 변환
        rgb_o3d   = o3d.geometry.Image(rgb_list[i].astype(np.uint8))
        depth_o3d = o3d.geometry.Image((depth_list[i]).astype(np.float32))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb_o3d, depth_o3d,
            depth_trunc=5.0, depth_scale=1.0,
            convert_rgb_to_intensity=False
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
        print(f"Frame {i}: Original points = {len(pcd.points)}")

        # # 시각화 (선택사항)
        o3d.visualization.draw_geometries([pcd])

        pcd = pcd.voxel_down_sample(voxel_size)

        # # 시각화 (선택사항)
        o3d.visualization.draw_geometries([pcd],)



        print(f"Frame {i}: Downsampled points = {len(pcd.points)}")
        # Normal vector 계산
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        # if (i - prev_idx) >= keyframe_step:
        # VO relative transform
        T_guess = np.linalg.inv(vo_poses[prev_idx]) @ vo_poses[i]

        # ICP
        reg = o3d.pipelines.registration.registration_icp(
            pcd, prev_pcd,
            max_correspondence_distance=0.1,
            init=T_guess,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )

        # refined pose 업데이트
        T_refined = refined_poses[prev_idx] @ reg.transformation
        refined_poses.append(T_refined)

        residual.append(reg.inlier_rmse)
        print(f"Frame {i}: ICP inlier RMSE = {reg.inlier_rmse}")

        # keyframe 업데이트
        prev_pcd = pcd
        prev_idx = i
        # else:
        #     # keyframe이 아니면 VO pose 그대로 사용
        #     T_refined = vo_poses[i]
        #     refined_poses.append(T_refined)

    return refined_poses

refined_poses = refine_trajectory_with_icp(rgb_list, depth_list, vo_poses, intrinsic)

# ===============================
# 3. Point cloud 누적
# ===============================
all_pcds = []
for i in range(len(rgb_list)):
    rgb_o3d   = o3d.geometry.Image(rgb_list[i].astype(np.uint8))
    depth_o3d = o3d.geometry.Image(depth_list[i].astype(np.uint16))

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgb_o3d, depth_o3d,
        depth_trunc=5.0,
        convert_rgb_to_intensity=False
    )

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    pcd = pcd.voxel_down_sample(0.05)

    # 보정된 pose 적용
    pcd.transform(refined_poses[i])
    all_pcds.append(pcd)

# 전체 환경 map
map_pcd = o3d.geometry.PointCloud()
for pcd in all_pcds:
    map_pcd += pcd
map_pcd = map_pcd.voxel_down_sample(0.05)

print(f"Final map points = {len(map_pcd.points)}")
print(f"Map bounding box: {map_pcd.get_axis_aligned_bounding_box()}")

# ===============================
# 4. Trajectory 시각화
# ===============================
# Trajectory 라인
traj_points = [pose[:3, 3] for pose in refined_poses]
traj_lines = [[i, i+1] for i in range(len(traj_points)-1)]
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(traj_points),
    lines=o3d.utility.Vector2iVector(traj_lines),
)
line_set.paint_uniform_color([1, 0, 0])  # 빨간색 trajectory

# ===============================
# 5. 시각화 실행
# ===============================
# o3d.visualization.draw_geometries([map_pcd, line_set])
# ===============================
# 5. 고급 시각화 실행
# ===============================
vis = o3d.visualization.Visualizer()
vis.create_window("Point Cloud Map with Trajectory")

# Point cloud 추가
vis.add_geometry(map_pcd)

# Trajectory 추가
vis.add_geometry(line_set)

# 카메라 위치 설정 (선택사항)
ctr = vis.get_view_control()
ctr.set_front([0, 0, -1])
ctr.set_up([0, -1, 0])

# 실행
vis.run()
vis.destroy_window()

# ===============================
# 6. Matplotlib로 2D Trajectory Plot (논문용)
# ===============================
traj_np = np.array(traj_points)
plt.figure()
plt.plot(traj_np[:,0], traj_np[:,1], 'r-', linewidth=2)
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.axis("equal")
plt.title("Refined Trajectory")
plt.show()



