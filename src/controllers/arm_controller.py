from .basic_controllers import PIDController
import numpy as np
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser
from robot.definitions import *
import quaternion
from .utils import *

# import rospkg
# rospack = rospkg.RosPack()
# PACKAGE_WS_PATH = rospack.get_path(
#     'ballbot_arm_description').split("/ballbot_arm_description")[0]

class ArmController(object):
    def __init__(self):
        self.nJoints = 7
        self._joint_controllers = [PIDController() for j in range(self.nJoints)]
        for j in range(self.nJoints):
            self._joint_controllers[j].set_antiwindup(True,10)
            self._joint_controllers[j].set_max_output(1000)
        self.desiredAngles = np.array([0.0 for j in range(self.nJoints)])
        self.desiredAngleVel = np.array([0.0 for j in range(self.nJoints)])
        self.gravity_torque = np.array([0.0 for j in range(self.nJoints)])
        self.set_gains([[100,0,1],[50,0,1],[50,0,1],[30,0,1],[30,0,1],[30,0,1],[10,0,0.1]])
        self.set_gains([[1000,0,1],[500,0,1],[500,0,1],[300,0,1],[300,0,1],[300,0,1],[100,0,0.1]])

    def set_gains(self, gains):
        for j in range(self.nJoints):
            self._joint_controllers[j].set_pid_gains(gains[j][0],gains[j][1],gains[j][2])

    def set_desired_angles(self, q):
        self.desiredAngles = q

    def set_desired_angle_vel(self, qdot):
        self.desiredAngleVel = qdot

    def set_gravity_torque(self,tau):
        self.gravity_torque = tau

    def update_current_state(self, qCurrent, qdotCorrent):
        self.currentAngles = qCurrent
        self.currentAngleVel = qdotCorrent

    def update(self, time_period_s):
        # PosErr = self.desiredAngles-self.currentAngles
        # VelErr = self.desiredAngleVel-self.currentAngleVel
        for j in range(self.nJoints):
            self._joint_controllers[j].calculate_error_values(self.currentAngles[j], self.desiredAngles[j],time_period_s) 
        self.armTorques = [self._joint_controllers[j].get_pid_output() + self.gravity_torque[j] for j in range(self.nJoints)]


class TaskSpaceArmController(object):
    def __init__(self, arm='right'):
        """
        Task space control for 7-dof ballbot arms
        Tasks as input position and orientation in shoulder frame
        """
        self.arm = arm
        self.desiredVel = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        self.desiredAcc = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

        # TODO tune gains
        self.set_gains([[10000.0, 10000.0, 10000, 10., 10., 10.],
                        [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]])
        URDF_NAME = "/home/saumyas/ballbot_sim_py3_ws/src/ballbot_pybullet_sim/urdf/ballbot_pybullet_wBarrettHands_toolLR_revolute.urdf"
        if arm == 'right':
            (status, self.tree) = kdl_parser.treeFromFile(URDF_NAME)
            self.chain = self.tree.getChain("RArm0", "toolR")
            self.desiredPos = np.array([[0.0, -0.4, 0.4]])
        elif arm == 'left':
            (status, self.tree) = kdl_parser.treeFromFile(URDF_NAME)
            self.chain = self.tree.getChain("LArm0", "toolL")
            self.desiredPos = np.array([[0.0, 0.4, 0.4]])
        else:
            print('TaskSpaceArmController: incorrect arm configuration')
            return
        
        self.M = kdl.JntSpaceInertiaMatrix(2)
        self.dynamic_param_solver = kdl.ChainDynParam(
            self.chain, kdl.Vector(0, 0, -9.8))
        self.num_joints = self.chain.getNrOfJoints()
        # print("\n*** This robot has %s joints *** \n" % self.num_joints)
        self.jacobian_solver = kdl.ChainJntToJacSolver(self.chain)
        self.M = kdl.JntSpaceInertiaMatrix(self.num_joints)
        self.C = kdl.JntArray(self.num_joints)

        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        print("-----------------------------------")
        print(arm, "arm: TaskSpaceArmController initialized!")
        print("-----------------------------------")
        print(self.num_joints)

    def set_gains(self, gains):
        self.Kp = np.diag(gains[0])
        self.Kd = np.diag(gains[1])

    def set_desired_pose(self, pos, quat): # input is desired pose in shoulder frame
        self.desiredPos = np.array([pos])
        self.desiredQuat = quat

    def set_desired_vel(self, vel):
        self.desiredVel = np.array([vel])

    def set_desired_acc(self, acc):
        self.desiredAcc = np.array([acc])

    # def set_gravity_torque(self, tau):
    #     self.gravity_torque = tau

    def update_current_state(self, qCurrent, qdotCurrent):
        self.currentAngles = np.array([qCurrent])
        self.currentAngleVel = np.array([qdotCurrent])
        qCurrent = np.append(qCurrent,0.) # Hack to make sure the input is of length 8 and not 7
        self.compute_J(qCurrent, qdotCurrent)
        self.compute_FK(qCurrent, self.num_joints)

    def update(self, time_period_s):

        pos_err = self.desiredPos - self.currPos
        # pos_err = np.zeros(3)
        rot_err = angle_axis_between_quats(self.currQuat, self.desiredQuat)
        rot_err = np.zeros(3)
        x_err = np.append(pos_err.flatten(), rot_err).reshape(1,6)
        dx_err = self.desiredVel - self.currVel
        torque_feedforward = self.J.T @ (self.desiredAcc.T) + kdl_to_vec(self.C)[:7].reshape(7,1)

        tau = self.J.T @ (self.Kp @ x_err.T + self.Kd @ dx_err.T)
        self.armTorques = np.array(tau.T).flatten() 

    def compute_J(self, q, dq):
        J = kdl.Jacobian(self.num_joints)
        theta = joint_list_to_kdl(q)
        theta_dot = joint_list_to_kdl(dq)
        self.jacobian_solver.JntToJac(
            theta, J)
        
        self.J = kdl_to_mat(J)[:,:7]
        # print('J: ', self.J)
        self.dynamic_param_solver.JntToMass(theta, self.M)
        self.dynamic_param_solver.JntToCoriolis(theta, theta_dot, self.C)

    def compute_FK(self, q, link_number):
        endeffec_frame = kdl.Frame()
        
        kinematics_status = self.fk_solver.JntToCart(joint_list_to_kdl(q),
                                                     endeffec_frame,
                                                     link_number)
        if kinematics_status >= 0:
            p = endeffec_frame.p
            M = endeffec_frame.M
            self.currPose = np.mat([[M[0, 0], M[0, 1], M[0, 2], p.x()],
                                    [M[1, 0], M[1, 1], M[1, 2], p.y()],
                                    [M[2, 0], M[2, 1], M[2, 2], p.z()],
                                    [0,      0,      0,     1]])
            self.currPos = np.array([p.x(), p.y(), p.z()])
            self.currQuat = quaternion.from_rotation_matrix(self.currPose[:3,:3])
        else:
            print('error in fk solver')
        self.currVel = np.array((self.J@self.currentAngleVel.T).T) # shape (1,6)

def joint_list_to_kdl(q):
    if q is None:
        return None
    if type(q) == np.matrix and q.shape[1] == 0:
        q = q.T.tolist()[0]
    q_kdl = kdl.JntArray(len(q))
    for i, q_i in enumerate(q):
        q_kdl[i] = q_i
    return q_kdl


def kdl_to_mat(m):
    mat = np.mat(np.zeros((m.rows(), m.columns())))
    for i in range(m.rows()):
        for j in range(m.columns()):
            mat[i, j] = m[i, j]
    return mat


def kdl_to_vec(m):
    mat = np.zeros(m.rows())
    for i in range(m.rows()):
        mat[i] = m[i]
    return mat