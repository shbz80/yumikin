# import pykdl_utils
import hrl_geom.transformations as trans
# from hrl_geom.pose_converter import PoseConv
from urdf_parser_py.urdf import Robot
from pykdl_utils.kdl_kinematics3 import KDLKinematics
# from pykdl_utils.kdl_parser import *
import copy
import numpy as np

class YumiKinematics(object):
    def __init__(self, params):
        self.file = params['urdf']
        f = open(self.file, 'r')
        robot = Robot.from_xml_string(f.read())
        self.euler_string = params['euler_string']
        # self.base_link = robot.get_root()
        self.base_link = params['base_link']
        self.end_link = params['end_link']
        self.kdl_kin = KDLKinematics(robot, self.base_link, self.end_link)
        self.goal_joint = params['goal']
        self.goal_cart = self.fwd_pose(params['goal'])
        self.dQ = 7
        self.ik_cl_alpha = 0.1
        self.ik_cl_max_itr = 100
        self.od, self.Rd = self.get_fwd_mat(self.goal_joint)

    def fwd_pose(self, q):
        Tr = self.kdl_kin.forward(q, end_link=self.end_link, base_link=self.base_link)
        epos = np.array(Tr[:3, 3])
        epos = epos.reshape(-1)
        erot = np.array(Tr[:3, :3])
        tmp = trans.euler_from_matrix(erot, self.euler_string)
        erot = tmp
        ep = np.append(epos, erot)
        return ep

    def get_fwd_mat(self, q):
        Tr = self.kdl_kin.forward(q, end_link=self.end_link, base_link=self.base_link)
        epos = np.array(Tr[:3, 3])
        epos = epos.reshape(-1)
        erot = np.array(Tr[:3, :3])
        return epos, erot

    def forward(self, X):
        X = X.reshape(-1, self.dQ*2)
        EX = np.zeros((X.shape[0], 12))
        dQ = self.dQ
        for i in range(X.shape[0]):
            x = X[i]
            q = x[:dQ]
            q_dot = x[dQ:]
            ep = self.fwd_pose(q)
            J_A = self.get_analytical_jacobian(q)
            ep_dot = J_A.dot(q_dot)
            ex = np.concatenate((ep,ep_dot))
            EX[i] = ex
        return EX

    def predict(self, Q, return_std=True):
        '''

        :param X: Qs
        :param return_std: always true
        :return: Xs
        '''
        dQ = self.dQ
        Q = Q.reshape(-1, dQ)
        X = np.zeros((Q.shape[0], 6))
        for i in range(Q.shape[0]):
            q = Q[i]
            x = self.fwd_pose(q)
            X[i] = x
        return X, np.zeros(X.shape)

    def get_analytical_jacobian(self, q):
        ep = self.fwd_pose(q)
        J_G = np.array(self.kdl_kin.jacobian(q))
        J_G = J_G.reshape((6, 7))
        J_A = YumiKinematics.jacobian_geometric_to_analytic(J_G, ep[3:], self.euler_string)
        return J_A

    def get_analytical_jacobian_d(self, q, phi_d_e, Rd):
        J_G = np.array(self.kdl_kin.jacobian(q))
        J_G = J_G.reshape((6, 7))
        Rd_mat = np.block([[Rd.T, np.zeros((3,3))], [np.zeros((3,3)), Rd.T]])
        Rd_mat_dot_J_G = Rd_mat.dot(J_G)
        J_A_d = YumiKinematics.jacobian_geometric_to_analytic(Rd_mat_dot_J_G, phi_d_e, self.euler_string)
        return J_A_d

    def get_cart_error_frame_terms(self, q, q_dot):
        # od, Rd = self.get_fwd_mat(self.goal_joint)
        od = self.od
        Rd = self.Rd
        oe, Re = self.get_fwd_mat(q)

        o_d_e = Rd.T.dot(oe-od)
        R_d_e = Rd.T.dot(Re)
        phi_d_e = trans.euler_from_matrix(R_d_e, self.euler_string)
        phi_d_e = np.array(phi_d_e).reshape(-1)
        x_d_e = np.append(o_d_e, phi_d_e)

        J_A_d = self.get_analytical_jacobian_d(q, phi_d_e, Rd)
        x_dot_d_e = J_A_d.dot(q_dot)

        return x_d_e, x_dot_d_e, J_A_d

    def get_cart_error_frame_list(self, Q_Q_dot):
        dQ = self.dQ
        assert(Q_Q_dot.shape[1]==dQ*2)
        N = Q_Q_dot.shape[0]
        X_X_dot = np.zeros((N,12))
        for i in range(N):
            q_q_dot = Q_Q_dot[i]
            q = q_q_dot[:dQ]
            q_dot = q_q_dot[dQ:]
            x, x_dot, _ = self.get_cart_error_frame_terms(q, q_dot)
            x_x_dot = np.concatenate((x, x_dot))
            X_X_dot[i] = x_x_dot
        return X_X_dot



    # @staticmethod
    # def jacobian_analytic_to_geometric(J_A, phi):
    #     '''
    #     assumes xyz Euler convention
    #     phi is Euler angle vector
    #     '''
    #     s = np.sin
    #     c = np.cos
    #
    #     assert (phi.shape == (3,))
    #     x = phi[0]
    #     y = phi[1]
    #     z = phi[2]
    #
    #     Tang = np.array([[1., 0., s(y)],
    #                      [0., c(x), -c(y) * s(x)],
    #                      [0., s(x), c(x) * c(y)]
    #                      ])
    #     Ttrans = np.diag(np.ones(3))
    #
    #     T_A = np.block([[Ttrans, np.zeros((3, 3))],
    #                     [np.zeros((3, 3)), Tang]
    #                     ])
    #     J_G = T_A.dot(J_A)
    #     return J_G

    @staticmethod
    def jacobian_geometric_to_analytic(J_G, phi, euler_string='sxyz'):
        '''
        phi is Euler angle vector
        '''
        s = np.sin
        c = np.cos

        assert (phi.shape == (3,))
        if euler_string=='szyx':
            z = phi[0]
            y = phi[1]
            x = phi[2]
            Tang_inv = np.array([[c(z) * s(y) / c(y), s(y) * s(z) / c(y), 1.0],
                                 [-s(z), c(z), 0.],
                                 [c(z) / c(y), s(z) / c(y), 0.]
                                 ])
        elif euler_string=='sxyz':
            x = phi[0]
            y = phi[1]
            z = phi[2]
            Tang_inv = np.array([[1., s(x) * s(y) / c(y), -c(x) * s(y) / c(y)],
                                 [0., c(x), s(x)],
                                 [0., -s(x) / c(y), c(x) / c(y)]
                                 ])
        else:
            pass

        Ttrans_inv = np.diag(np.ones(3))
        T_A_inv = np.block([[Ttrans_inv, np.zeros((3, 3))],
                            [np.zeros((3, 3)), Tang_inv]
                            ])
        J_A = T_A_inv.dot(J_G)
        return J_A

    def closed_loop_IK(self, x, q0):
        alpha = self.ik_cl_alpha
        max_itr = self.ik_cl_max_itr
        q_k = q0
        for itr in range(max_itr):
            x_k = self.fwd_pose(q_k)
            J_A = self.get_analytical_jacobian(q_k)
            J_A_inv = np.linalg.pinv(J_A)
            dq = alpha*J_A_inv.dot(x - x_k)
            q_k += dq
            if np.linalg.norm(dq)<1e-6:
                return q_k
        return None
