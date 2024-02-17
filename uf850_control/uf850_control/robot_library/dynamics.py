"""
dynamics Module - Contains code for:
- Dynamic SerialArm class
- RNE Algorithm
- Euler - Lagrange formulation

John Morrell, Jan 28 2022
Tarnarmour@gmail.com

modified by: 
Marc Killpack, October 25, 2022
"""

import numpy as np
from uf850_control.robot_library.transforms import inv
from uf850_control.robot_library.kinematics import SerialArm
from uf850_control.robot_library.utility import skew

eye = np.eye(4)

class SerialArmDyn(SerialArm):
    """
    SerialArmDyn class represents serial arms with dynamic properties and is used to calculate forces, torques, accelerations,
    joint forces, etc. using the Newton-Euler and Euler-Lagrange formulations. It inherits from the previously defined kinematic
    robot arm class "SerialArm". 
    """

    def __init__(self, 
                 dh, 
                 jt=None, 
                 base=eye, 
                 tip=eye, 
                 joint_limits=None,
                 mass=None,
                 r_com=None,
                 link_inertia=None,
                 motor_inertia=None,
                 joint_damping=None):

        SerialArm.__init__(self, dh, jt, base, tip, joint_limits)
        self.mass = mass
        self.r_com = r_com
        self.link_inertia = link_inertia
        self.motor_inertia = motor_inertia
        if joint_damping is None:
            self.B = np.zeros((self.n, self.n))
        else:
            self.B = np.diag(joint_damping)

    def rne(self, q, qd, qdd, 
            Wext=np.zeros((6,1)),
            g=np.zeros((3, 1)),
            omega_base=np.zeros((3, 1)),
            alpha_base=np.zeros((3, 1)),
            v_base=np.zeros((3, 1)),
            acc_base=np.zeros((3, 1))):

        """
        tau, W = RNE(q, qd, qdd):
        returns the torque in each joint (and the full wrench at each joint) given the joint configuration, velocity, and accelerations
        Args:
            q:
            qd:
            qdd:
        
        Returns:
            taus: list (length n)
            Wrenches: list (length n)
        
        We start with the velocity and acceleration of the base frame, v0 and a0, and the joint positions, joint velocities,
        and joint accelerations (q, qd, qdd).
        
        For each joint, we find the new angular velocity, w_i = w_(i-1) + z * qdot_(i-1)
        v_i = v_(i-1) + w_i x r_(i-1, com_i)
        
        if motor inertia is None, we don't consider it. Solve for now without motor inertia. The solution will provide code for motor inertia as well. 
        """

        omegas = []
        alphas = []
        v_ends = []
        v_coms = []
        acc_ends = []
        acc_coms = []

        # The index matches the joint number. Therefore, omegas[0] is the base
        omegas.append(np.hstack(omega_base))
        alphas.append(np.hstack(alpha_base))
        v_ends.append(np.hstack(v_base))
        v_coms.append(np.hstack(v_base))
        acc_ends.append(np.hstack(acc_base))
        acc_coms.append(np.hstack(acc_base))

        # TEMP
        #print("q: ", q)
        #print("qd: ", qd)
        #print("qdd: ", qdd)

        ## KINEMATICS: Solve for needed angular velocities, angular accelerations, and linear accelerations
        def rne_kinem(i, omega, alpha, ve, ae):
            # i - link number
            # omega, alpha, ve, ae, ac - for link (i-1) in frame (i-1)
            T__i_im1 = inv(self.fk(q, index=(i-1, i)))
            R__i_im1 = T__i_im1[0:3,0:3]
            z__i_im1 = T__i_im1[0:3,2]
            #R__i_im1 = self.fk(q, index=(i-1, i))[0:3,0:3].T
            #z__0_im1 = self.fk(q, index=(0, i-1))[0:3,3]
            #z__i_im1 = R_i__0 @ z__0_im1
            qdoti = qd[i-1] # qdot of joint i is qd[i-1]
            omega_i = R__i_im1 @ omega + z__i_im1 * qdoti
            qddoti = qdd[i-1] # qddot of joint i is qdd[i-1]
            alpha_i = R__i_im1 @ alpha + z__i_im1 * qddoti + np.cross(omega_i, z__i_im1 * qdoti)
            r_im1e = -T__i_im1[0:3,3] # r from frame (i-1) to end
            r_im1c = r_im1e + self.r_com[i-1]
            ve_i = ve + np.cross(omega_i, r_im1e)
            vc_i = ve + np.cross(omega_i, r_im1c)
            ae_i = R__i_im1 @ ae + np.cross(alpha_i, r_im1e) + np.cross(omega_i, np.cross(omega_i, r_im1e))
            ac_i = R__i_im1 @ ae + np.cross(alpha_i, r_im1c) + np.cross(omega_i, np.cross(omega_i, r_im1c))
            #print(f"omega: {omega_i}, \nalpha: {alpha_i}, \nve: {ve_i}, \nvc: {vc_i}, \nae: {ae_i}, \nac: {ac_i}")
            return (omega_i, alpha_i, ve_i, vc_i, ae_i, ac_i)

        # Index i = link #
        for i in range(1, self.n+1):
            (omega_i, alpha_i, ve_i, vc_i, ae_i, ac_i) = rne_kinem(i, omegas[i-1], alphas[i-1], v_ends[i-1], acc_ends[i-1])
            omegas.append(omega_i)
            alphas.append(alpha_i)
            v_ends.append(ve_i)
            v_coms.append(vc_i)
            acc_ends.append(ae_i)
            acc_coms.append(ac_i)

        #print("acc_coms: ", acc_coms)

        ## KINETICS: Now solve Kinetic equations by starting with forces at last link and going backwards
        Wrenches = [np.zeros((6,1))] * (self.n + 1)
        Wrenches[self.n] = np.hstack(Wext)

        for i in range(self.n, 0, -1):  # Index i from n to 1
            # Set up variables needed in kinetics equations
            mi = self.mass[i-1]
            Ii = self.link_inertia[i-1]
            f_ip1 = np.hstack(Wrenches[i][0:3])
            tau_ip1 = np.hstack(Wrenches[i][3:6])
            if i == self.n:
                # For link n, i+1 is the tool
                R__i_ip1 = self.tip[0:3,0:3]
            else:
                R__i_ip1 = self.fk(q, index=(i, i+1))[0:3,0:3]
            # Rotate g into the i frame
            R__i_0 = self.fk(q, index=(0, i))[0:3,0:3].T
            gi = R__i_0 @ np.hstack(g)
            # Position vector r^i_{i-1,com}
            T__i_im1 = inv(self.fk(q, index=(i-1, i)))
            r_im1e = -T__i_im1[0:3,3] # r from frame (i-1) to end
            r_ic = self.r_com[i-1]
            r_im1c = r_im1e + r_ic
            # Kinetics equations
            f_i = R__i_ip1@f_ip1 - mi*gi + mi*acc_coms[i]
            tau_i = R__i_ip1@tau_ip1 - np.cross(f_i, r_im1c) + np.cross(R__i_ip1@f_ip1, r_ic) + Ii@alphas[i] + np.cross(omegas[i], Ii@omegas[i])
            Wrenches[i-1] = np.concatenate((f_i, tau_i))

        #print("Wrenches:\n", Wrenches)
        # Extract the joint torques
        taus = [0] * self.n
        for i in range(1, self.n+1):
            tau__i_i = Wrenches[i-1][3:6]
            R__im1_i = self.fk(q, index=(i-1, i))[0:3,0:3]
            taus[i-1] = (R__im1_i @ tau__i_i)[2]

        return taus, Wrenches


if __name__ == '__main__':

    ## this just gives an example of how to define a robot, this is a planar 3R robot.
    dh = [[0, 0, 1, 0],
          [0, 0, 1, 0],
          [0, 0, 1, 0]]

    joint_type = ['r', 'r', 'r']

    link_masses = [1, 1, 1]

    # defining three different centers of mass, one for each link
    r_coms = [np.array([-0.5, 0, 0]), np.array([-0.5, 0, 0]), np.array([-0.5, 0, 0])]

    link_inertias = []
    for i in range(len(joint_type)):
        iner = link_masses[i] / 12 * dh[i][2]**2

        # this inertia tensor is only defined as having Iyy, and Izz non-zero
        link_inertias.append(np.array([[0, 0, 0], [0, iner, 0], [0, 0, iner]]))


    arm = SerialArmDyn(dh,
                       jt=joint_type,
                       mass=link_masses,
                       r_com=r_coms,
                       link_inertia=link_inertias)

    # once implemented, you can call arm.RNE and it should work. 
    q = [np.pi/4.0]*3
    qd = [0.2]*3
    qdd = [0.05]*3
    arm.rne(q, qd, qdd)