# import pywavefront

# from pywavefront.mesh import Mesh

# from pywavefront import visualization

# # Load the OBJ file
# scene = pywavefront.Wavefront('/home/seungwoo/alchemist_ws/src/BPMP-Tracker/script/AllStaticMesh.OBJ')


# # Access data like vertices, faces, etc.
# # For example, to access the first mesh's vertices:


# # visualize
# visualization.draw(scene)



from typing import List, Tuple, Optional
import numpy as np
from tqdm import tqdm


from open3d.visualization.rendering import Material

def _fix_idx(idx: int, n: int) -> int:
    """OBJ 인덱스 보정: 1-based, 음수는 뒤에서부터."""
    return idx-1 if idx > 0 else n + idx

def load_obj(path: str):
    V: List[Tuple[float,float,float]] = []
    VT: List[Tuple[float,...]] = []
    VN: List[Tuple[float,float,float]] = []
    Fv: List[Tuple[int,int,int]] = []
    Fvt: List[Tuple[int,int,int]] = []
    Fvn: List[Tuple[int,int,int]] = []

    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in tqdm(f, desc="Loading OBJ file"):
            if not line or line.startswith('#'): 
                continue
            parts = line.strip().split()
            if not parts:
                continue
            tag, vals = parts[0], parts[1:]

            if tag == 'v':
                V.append(tuple(map(float, vals[:3])))
            elif tag == 'vt':
                # u v [w] 지원
                UVW = tuple(map(float, vals[:3]))
                VT.append(UVW)
            elif tag == 'vn':
                VN.append(tuple(map(float, vals[:3])))
            elif tag == 'f':
                # 다각형 지원 -> 삼각 팬 분할
                # 각 토큰은 i, i/j, i//k, i/j/k 형태
                verts = []
                texs  = []
                norms = []
                for tok in vals:
                    a = tok.split('/')
                    vi = int(a[0]) if a[0] else 0
                    ti = int(a[1]) if len(a) > 1 and a[1] else 0
                    ni = int(a[2]) if len(a) > 2 and a[2] else 0
                    verts.append(_fix_idx(vi, len(V)))
                    texs.append(_fix_idx(ti, len(VT)) if ti else None)
                    norms.append(_fix_idx(ni, len(VN)) if ni else None)

                # 팬 삼각화: (0, k-1, k)
                for k in range(2, len(verts)):
                    Fv.append((verts[0], verts[k-1], verts[k]))
                    if any(t is not None for t in texs):
                        Fvt.append((texs[0], texs[k-1], texs[k]))
                    if any(n is not None for n in norms):
                        Fvn.append((norms[0], norms[k-1], norms[k]))

    V  = np.asarray(V, dtype=np.float32) if V else np.zeros((0,3), np.float32)
    VT = np.asarray(VT, dtype=np.float32) if VT else np.zeros((0,2), np.float32)
    VN = np.asarray(VN, dtype=np.float32) if VN else np.zeros((0,3), np.float32)
    Fv = np.asarray(Fv, dtype=np.int32) if Fv else np.zeros((0,3), np.int32)
    Fvt = np.asarray(Fvt, dtype=object) if Fvt else None
    Fvn = np.asarray(Fvn, dtype=object) if Fvn else None

    # vt가 3D로 온 경우(uvw) → uv만 사용 원하면 아래 주석 해제
    # if VT.shape[1] == 3:
    #     VT = VT[:, :2]

    return V, VT, VN, Fv, Fvt, Fvn


# if __name__ == '__main__':
import pickle
obj_path = '/home/seungwoo/alchemist_ws/src/BPMP-Tracker/script/AllStaticMesh.OBJ'
V, VT, VN, Fv, Fvt, Fvn = load_obj(obj_path)

# visualize using trimesh
# import trimesh
# mesh = trimesh.Trimesh(vertices=V, faces=Fv, vertex_normals=VN if len(VN) == len(V) else None)
# mesh.show()

import numpy as np
import open3d as o3d
from PIL import Image
import matplotlib.pyplot as plt

# V, VT, Fv, Fvt 는 이미 당신 파서 결과라고 가정
# 1) Mesh 구성
mesh_origin = o3d.geometry.TriangleMesh(
    vertices=o3d.utility.Vector3dVector(V.astype(float)),
    triangles=o3d.utility.Vector3iVector(Fv.astype(int))
)
# transform the mesh to align with the coordinate system if needed
mesh_origin.transform([[0, 0, 1, 0],
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 0, 1]])

coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50.0, origin=[0, 0, 0])



import copy
mesh = copy.deepcopy(mesh_origin)    
# mesh에서 특정 높이만큼 자르기
z_min, z_max = 0, 300
x_min, x_max = -1100, 1800
y_min, y_max = -1200, 750
bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(x_min, y_min, z_min), max_bound=(x_max, y_max, z_max))
mesh = mesh.crop(bbox)

# mesh 에 높이에 따른 색 입히기
z_values = np.asarray(mesh.vertices)[:, 2]
z_norm = (z_values - z_min) / (z_max - z_min + 1e-8)  # 0~1 정규화
# colors = plt.cm.viridis(z_norm)[:, :3]  # viridis 컬러맵에서 RGB 추출
colors = plt.cm.get_cmap('jet')(z_norm)[:, :3]  # jet 컬러맵에서 RGB 추출
mesh.vertex_colors = o3d.utility.Vector3dVector(colors)


x_offset = 717.464 - (-215.0)
y_offset = -287.73 - 68.4938

mesh = mesh.translate((x_offset, y_offset, 0))



center = mesh.get_center()
bbox = mesh.get_axis_aligned_bounding_box()
extent = bbox.get_extent()
half_w, half_h = extent[0]/2, extent[1]/2
near, far = 0.1, extent[2] + 10.0

app = o3d.visualization.gui.Application.instance
app.initialize()

w = o3d.visualization.gui.Application.instance.create_window("Ortho Top-Down", 1280, 800)
scene_widget = o3d.visualization.gui.SceneWidget()
scene_widget.scene = o3d.visualization.rendering.Open3DScene(w.renderer)
scene_widget.scene.add_geometry("mesh", mesh, o3d.visualization.rendering.Material())

w.add_child(scene_widget)

# 카메라 위치/방향: z+ 위에서 아래로
cam = scene_widget.scene.camera

eye = center + np.array([0, 0, extent[2]*2.0])   # 위쪽(z+)에서 내려다봄
look = center
up   = [0, 1, 0]
cam.look_at(look, eye, up)

# 정사영 설정 (Projection enum, left, right, bottom, top, near, far)
cam.set_projection(o3d.visualization.rendering.Camera.Projection.Ortho,  # Ortho projection
                   -half_w, half_w,    # left, right
                   -half_h, half_h,    # bottom, top  
                   near, far)          # near, far

# 배경/조명 등 옵션(선택)
scene_widget.scene.show_axes(True)

app.run()

