import mujoco
import mujoco.viewer
import numpy as np
import utils


def evaluateT(data):
    data.ctrl[:] = target_positions

    p0 = data.site_xpos[0]
    R0 = data.site_xmat[0].reshape(3, 3)
    mujoco_T0 = utils.mergeRp(R0, p0)

    p1 = data.site_xpos[1]
    R1 = data.site_xmat[1].reshape(3, 3)
    mujoco_T1 = utils.mergeRp(R1, p1)

    PoE_T = utils.PoE(theta, w, v, M)
    print("mujoco_T:")
    print(utils.relative_pose(mujoco_T1, mujoco_T0))
    print("PoE:")
    print(PoE_T)

def evaluateGeoJacobi(data):
    data.ctrl[:] = target_positions
    mujoco_geoJacobi(model, data)
    PoE_geoJacobi(theta, w, v)

def mujoco_geoJacobi(model, data):
    nq = model.nq  # 机器人关节的自由度数
    jacp_end = np.zeros((3, nq))  # 线速度部分的雅可比矩阵 (3 x N)
    jacr_end = np.zeros((3, nq))  # 角速度部分的雅可比矩阵 (3 x N)
    mujoco.mj_jacSite(model, data, jacp_end, jacr_end, 1)
    J_end = np.vstack((jacr_end, jacp_end))
    print("mujoco_geoJacobi:")
    print(J_end)


def PoE_geoJacobi(theta, w, v):
    J = utils.space_jacobi(theta, w, v)
    p0 = data.site_xpos[0]
    R0 = data.site_xmat[0].reshape(3, 3)
    mujoco_T0 = utils.mergeRp(R0, p0)
    print("PoE_geoJacobi:")
    print(J)


xml_path = "src/robot.xml"
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)
l1 = 0.2
l2 = 0.2
M = np.array([[0, 0, 1, l1],
              [0, 1, 0, 0],
              [-1, 0, 0, -l2],
              [0, 0, 0, 1]])

w = np.array([[0, 0, 1],
              [0, -1, 0],
              [1, 0, 0]])

v = np.array([[0, 0, 0],
              [0, 0, -l1],
              [0, -l2, 0]])

w_test = np.array([[0, 1, 0],
              [0, 0, 0],
              [0, 0, 1]])

v_test = np.array([[0, 0, 0],
              [0, 1, 0],
              [4, 0, 0]])

theta = np.array([np.pi/6, np.pi/6, np.pi/6])
# theta = np.array([0, 0, 0])
target_positions = theta

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        evaluateT(data)
        # evaluateGeoJacobi(data)
        # PoE_geoJacobi(theta, w_test, v_test)
        # 在视图中同步显示
        viewer.sync()
