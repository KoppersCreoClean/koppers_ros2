"""
Kinematics Module - Contains code for:
- Forward Kinematics, from a set of DH parameters to a serial linkage arm with callable forward kinematics
- Inverse Kinematics
- Jacobian

John Morrell, Jan 26 2022
Tarnarmour@gmail.com

modified by: 
Marc Killpack, Sept 21, 2022
"""

import numpy as np
from uf850_control.robot_library.transforms import *
import time # For updating viz
from uf850_control.robot_library.utility import skew # For Z shift function


class dh2AFunc:
    """
    A = dh2AFunc(dh, joint_type="r")
    Description:
    Accepts one link of dh parameters and returns a function "f" that will generate a
    homogeneous transform "A" given "q" as an input. A represents the transform from 
    link i to link i+1

    Parameters:
    dh - 1 x 4 list or iterable of floats, dh parameter table for one transform from link i to link i+1,
    in the order [theta d a alpha] - THIS IS NOT THE CONVENTION IN THE BOOK!!! But it is the order of operations. 

    Returns:
    f(q) - a function that can be used to generate a 4x4 numpy matrix representing the transform from one link to the next
    """
    def __init__(self, dh, jt='r'):

        self.theta = dh[0]
        self.d = dh[1]
        self.a = dh[2]
        self.alpha = dh[3]
        self.jt = jt

    def A(self, q):
        theta = self.theta
        d = self.d
        a = self.a
        alpha = self.alpha
        # if joint is revolute
        if self.jt == 'r':
            theta += q
        # if joint is prismatic
        else:
            d += q
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
        
    """ I didn't want to do it this way. I hope this is still right
    
    # if joint is revolute implement correct equations here:
    if jt == 'r':
        def A(q):
            theta = dh[0] + q
            d = dh[1]
            a = dh[2]
            alpha = dh[3]

            # See eq. (2.52), pg. 64
            # TODO - complete code that defines the "A" homogenous matrix for a given set of DH parameters. 
            # Do this in terms of theta, d, a, and alpha variables as defined above. 

            return np.array()


    # if joint is prismatic implement correct equations here:
    else:
        def A(q):
            theta = dh[0]
            d = dh[1] + q
            a = dh[2]
            alpha = dh[3]

            # See eq. (2.52), pg. 64
            # TODO - complete code that defines the "A" homogenous matrix for a given set of DH parameters. 
            # Do this in terms of theta, d, a, and alpha variables as defined above. 

            return np.array()
    """

    #self.A = A


class SerialArm:
    """
    SerialArm - A class designed to represent a serial link robot arm

    SerialArms have frames 0 to n defined, with frame 0 located at the first joint and aligned with the robot body
    frame, and frame n located at the end of link n.

    """

    def __init__(self, dh, jt=None, base=np.eye(4), tip=np.eye(4), joint_limits=None):
        """
        arm = SerialArm(dh, joint_type, base=I, tip=I, radians=True, joint_limits=None)
        :param dh: n length list or iterable of length 4 list or iterables representing dh parameters, [theta d a alpha]
        :param jt: n length list or iterable of strings, 'r' for revolute joint and 'p' for prismatic joint
        :param base: 4x4 numpy or sympy array representing SE3 transform from world frame to frame 0
        :param tip: 4x4 numpy or sympy array representing SE3 transform from frame n to tool frame
        :param joint_limits: 2 length list of n length lists, holding first negative joint limit then positive, none for
        not implemented
        """
        self.dh = dh
        self.n = len(dh)

        # we will use this list to store the A matrices for each set/row of DH parameters. 
        self.transforms = []

        # assigning a joint type
        if jt is None:
            self.jt = ['r'] * self.n
        else:
            self.jt = jt
            if len(self.jt) != self.n:
                print("WARNING! Joint Type list does not have the same size as dh param list!")
                return None

        # generating the function A(q) for each set of DH parameters
        for i in range(self.n):
            # TODO use the class definition above (dh2AFunc), and the dh parameters and joint type to
            # make a function and then append that function to the "transforms" list.
            f = dh2AFunc(self.dh[i], self.jt[i])
            self.transforms.append(f.A)

        # assigning the base, and tip transforms that will be added to the default DH transformations.
        self.base = base
        self.tip = tip
        self.qlim = joint_limits

        self.reach = 0
        for i in range(self.n):
            self.reach += np.sqrt(self.dh[i][1]**2 + self.dh[i][2]**2)

        self.max_reach = 0.0
        for dh in self.dh:
            self.max_reach += norm(np.array([dh[1], dh[2]]))

    def __str__(self):
        """
            This function just provides a nice interface for printing information about the arm. 
            If we call "print(arm)" on an SerialArm object "arm", then this function gets called.
            See example in "main" below. 
        """
        dh_string = """DH PARAMS\n"""
        dh_string += """theta\t|\td\t|\ta\t|\talpha\t|\ttype\n"""
        dh_string += """---------------------------------------\n"""
        for i in range(self.n):
            dh_string += f"{self.dh[i][0]}\t|\t{self.dh[i][1]}\t|\t{self.dh[i][2]}\t|\t{self.dh[i][3]}\t|\t{self.jt[i]}\n"
        return "Serial Arm\n" + dh_string

    def jacob(self, q, index=None, base=False, tip=False):
        """
        J = arm.jacob(q)
        Description: 
        Returns the geometric jacobian for the end effector frame of the arm in a given configuration

        Parameters:
        q - list or numpy array of joint positions
        index - integer, which joint frame at which to calculate the Jacobian

        Returns:
        J - numpy matrix 6xN, geometric jacobian of the robot arm
        """

        if index is None:
            index = self.n
        elif index > self.n:
            print("WARNING: Index greater than number of joints!")
            print(f"Index: {index}")

        # Is this right? That there is one column for each joint included?
        J = np.zeros([6, self.n])

        ## Current position of the end-effector
        #Te = self.fk(q, index=self.n, base=base, tip=tip)
        #pe = Te[0:3, 3] # Pull off position vector

        # Current position of the point of interest
        T_n = self.fk(q, index=index, base=base, tip=tip)
        p_n = T_n[0:3, 3] # Pull off position vector

        # Fill the Jacobian column by column
        for i in range(index):
            T_prev = self.fk(q, index=i, base=base, tip=tip)
            z_prev = T_prev[0:3, 2] # z_{i-1}
            # check if joint is revolute
            if self.jt[i] == 'r':
                p_prev = T_prev[0:3, 3]
                J[0:3,i] = np.cross(z_prev, (p_n - p_prev))
                J[3:6,i] = z_prev
            # if not assume joint is prismatic
            else:
                J[0:3,i] = z_prev

        return J

    def fk(self, q, index=None, base=True, tip=False):
        """
            T = arm.fk(q, index=None, base=False, tip=False)
            Description: 
                Returns the transform from a specified frame to another given a 
                set of joint inputs q and the index of joints

            Parameters:
                q - list or iterable of floats which represent the joint positions
                index - integer or list of two integers. If a list of two integers, the first integer represents the starting JOINT 
                    (with 0 as the first joint and n as the last joint) and the second integer represents the ending FRAME
                    If one integer is given only, then the integer represents the ending Frame and the FK is calculated as starting from 
                    the first joint
                base - bool, if True then if index starts from 0 the base transform will also be included
                tip - bool, if true and if the index ends at the nth frame then the tool transform will be included
            
            Returns:
                T - the 4 x 4 homogeneous transform from frames determined from "index" variable
        """

        # the following lines of code are data type and error checking. You don't need to understand
        # all of it, but it is helpful to keep. 

        if not hasattr(q, '__getitem__'):
            q = [q]

        if len(q) != self.n:
            print("WARNING: q (input angle) not the same size as number of links!")
            return None

        if isinstance(index, (list, tuple)):
            start_frame = index[0]
            end_frame = index[1]
        elif index == None:
            start_frame = 0
            end_frame = self.n
        else:
            start_frame = 0
            if index < 0:
                print("WARNING: Index less than 0!")
                print(f"Index: {index}")
                return None
            end_frame = index

        if end_frame > self.n:
            print("WARNING: Ending index greater than number of joints!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None
        if start_frame < 0:
            print("WARNING: Starting index less than 0!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None
        if start_frame > end_frame:
            print("WARNING: starting frame must be less than ending frame!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None

        # TODO complete each of the different cases below. If you don't like the 
        # current setup (in terms of if/else statements) you can do your own thing.
        # But the functionality should be the same.
        if base and start_frame == 0:
            T = self.base
        else:
            T = np.eye(4)

        for i in range(start_frame, end_frame):
            T = T @ self.transforms[i](q[i])

        if tip and end_frame == self.n:
            T = T @ self.tip

        return T

    def manipulability(self, q, index=None, base=False, tip=False, dim=(0,1,2)):
        """
        m = arm.manipulability(q, index=None, base=False, tip=False)
        Description: 
        Returns the manipulability of the robot arm in a given configuration

        Parameters:
        q - list or numpy array of joint positions
        index - integer, which joint frame at which to calculate the Jacobian

        Returns:
        m - float, manipulability of the robot arm
        """
        J = self.jacob(q, index=index, base=base, tip=tip)[dim,:]
        m = np.sqrt(max(0,np.linalg.det(J @ J.T)))
        return m

    def ik_position(self, target, q0=None, method='J_T', force=True, tol=1e-4, 
            K=None, kd=0.001, max_iter=100, debug=False, viz=None):
        """
        (qf, ef, iter, reached_max_iter, status_msg) = arm.ik2(target, q0=None, method='jt', force=False, tol=1e-6, K=None)
        Description:
            Returns a solution to the inverse kinematics problem finding
            joint angles corresponding to the position (x y z coords) of target

        Args:
            target: 3x1 numpy array that defines the target location. 

            q0: length of initial joint coordinates, defaults to q=0 (which is
            often a singularity - other starting positions are recommended)

            method: String describing which IK algorithm to use. Options include:
                - 'pinv': damped pseudo-inverse solution, qdot = J_dag * e * dt, where
                J_dag = J.T * (J * J.T + kd**2)^-1
                - 'J_T': jacobian transpose method, qdot = J.T * K * e

            force: Boolean, if True will attempt to solve even if a naive reach check
            determines the target to be outside the reach of the arm

            tol: float, tolerance in the norm of the error in pose used as termination criteria for while loop

            K: 3x3 numpy matrix. For both pinv and J_T, K is the positive definite gain matrix used for both. 

            kd: is a scalar used in the pinv method to make sure the matrix is invertible. 

            max_iter: maximum attempts before giving up.

        Returns:
            qf: 6x1 numpy matrix of final joint values. If IK fails to converge the last set
            of joint angles is still returned

            ef: 3x1 numpy vector of the final error

            count: int, number of iterations

            flag: bool, "true" indicates successful IK solution and "false" unsuccessful

            status_msg: A string that may be useful to understanding why it failed. 
        """
        # Fill in q if none given, and convert to numpy array 
        if isinstance(q0, np.ndarray):
            q = q0.astype(np.float64)
        elif q0 == None:
            q = np.array([0.0]*self.n)
        else:
            q = np.array(q0, dtype=np.float64)

        # initializing some variables in case checks below don't work
        err = None
        count = 0

        # Try basic check for if the target is in the workspace.
        # Maximum length of the arm is sum(sqrt(d_i^2 + a_i^2)), distance to target is norm(A_t)
        maximum_reach = 0
        for i in range(self.n):  # Add max length of each link
            maximum_reach = maximum_reach + np.sqrt(self.dh[i][1] ** 2 + self.dh[i][2] ** 2)

        pt = target  # Find distance to target
        target_distance = np.sqrt(pt[0] ** 2 + pt[1] ** 2 + pt[2] ** 2)

        if target_distance > maximum_reach and not force:
            print("WARNING: Target outside of reachable workspace!")
            return q, err, count, False, "Failed: Out of workspace"
        else:
            if target_distance > maximum_reach:
                pass
                # print("Target out of workspace, but finding closest solution anyway")
            else:
                pass
                # print("Target passes naive reach test, distance is {:.1} and max reach is {:.1}".format(
                    # float(target_distance), float(maximum_reach)))

        if not isinstance(K, np.ndarray):
            return q, err, count, False,  "No gain matrix 'K' provided"



        # you may want to define some functions here to help with operations that you will 
        # perform repeatedly in the while loop below. Alternatively, you can also just define 
        # them as class functions and use them as self.<function_name>.

        def get_error(q):
            cur_position = self.fk(q)[0:3,3] # Current position vector in base frame
            return target - cur_position

        def get_qdot(q, err):
            """ err: Error vector from current position to target (in cartesian space)
                For now this is a 3x1 because we're only doing position
            """
            J = self.jacob(q)[0:3,:] # Could specify other prms
            if method == "J_T":
                return J.T @ K @ err
            elif method == "pinv":
                #return np.linalg.lstsq(J, err) # Same as pinv???
                return np.linalg.pinv(J) @ K @ err
                #pinv = J.T @ np.linalg.inv(J @ J.T + kd*eye(3)) # Nearly same as below
                #J_dag = J.T @ np.linalg.inv(J @ J.T + kd**2*np.eye(3)) # I don't love this
                #return J_dag @ K @ err # TODO: Check that this is how the gain matrix K works

        # Starting error
        err = get_error(q)

        while np.linalg.norm(err) > tol and count < max_iter:
            
            # In this while loop you will update q for each iteration, and update, then
            # your error to see if the problem has converged. You may want to print the error
            # or the "count" at each iteration to help you see the progress as you debug. 
            # You may even want to plot an arm initially for each iteration to make sure 
            # it's moving in the right direction towards the target. 

            qdot = get_qdot(q, err)

            # Advance q, err, & count
            #q += qdot
            q += qdot
            err = get_error(q)
            count += 1

            if debug:
                print(f"Iteration {count}: q = {q}")
                # Update viz
                viz.update(qs=[q])
                if method=="pinv":
                    time.sleep(1.0/10)

        # when "while" loop is done, return the relevant info. 
        return (q, err, count, count < max_iter, 'No errors noted')

    def Z_shift(self, R=np.eye(3), p=np.zeros((3,)), p_frame='i'):
        """
        Z = Z_shift(R, p, p_frame_order)
        Description: 
            Generates a shifting operator (rotates and translates) to move twists and Jacobians 
            from one point to a new point defined by the relative transform R and the translation p. 

        Parameters:
            R - 3x3 numpy array, the rotation matrix R^j_i
            p - 3x1 numpy array length 3 iterable, the translation from the initial Jacobian point to the final point, expressed in the frame as described by the next variable.
            p_frame - is either 'i', or 'j'. Allows us to define if "p" is expressed in frame "i" or "j", and where the skew symmetrics matrix should show up. 

        Returns:
            Z - 6x6 numpy array, can be used to shift a Jacobian, or a twist
        """

        # generate our skew matrix
        S = skew(p)

        # Initialize left and right matrices
        A = np.zeros((6,6))
        B = np.zeros((6,6))

        if p_frame == 'i':
            A[0:3, 0:3] = R
            A[3:6, 3:6] = R
            B[0:3, 0:3] = np.eye(3)
            B[3:6, 3:6] = np.eye(3)
            B[0:3, 3:6] = -S
            Z = A @ B
        elif p_frame == 'j':
            A[0:3, 0:3] = np.eye(3)
            A[3:6, 3:6] = np.eye(3)
            A[0:3, 3:6] = -S
            B[0:3, 0:3] = R
            B[3:6, 3:6] = R
            Z = A @ B
        else:
            Z = None

        return Z



if __name__ == "__main__":
    from visualization import VizScene
    import time

    # Defining a table of DH parameters where each row corresponds to another joint.
    # The order of the DH parameters is [theta, d, a, alpha] - which is the order of operations. 
    # The symbolic joint variables "q" do not have to be explicitly defined here. 
    # This is a two link, planar robot arm with two revolute joints. 
    dh = [[0, 0, 0.3, 0],
          [0, 0, 0.3, 0]]

    # make robot arm (assuming all joints are revolute)
    arm = SerialArm(dh)

    # defining joint configuration
    q = [np.pi/4.0, np.pi/4.0]  # 45 degrees and 45 degrees

    # show an example of calculating the entire forward kinematics
    Tn_in_0 = arm.fk(q)
    print("Tn_in_0:\n", Tn_in_0, "\n")
    #show_frame('0', '2', Tn_to_0) # this will only work if all of values are numeric

    # show an example of calculating the kinematics between frames 0 and 1
    T1_in_0 = arm.fk(q, index=[0,1])
    print("T1_in 0:\n", T1_in_0, "\n")
    #show_frame('0', '1', T1_to_0)
    
    print(arm)

    viz = VizScene()

    viz.add_frame(arm.base, label='base')
    viz.add_frame(Tn_in_0, label="Tn_in_0")
    viz.add_frame(T1_in_0, label="T1_in_0")

    time_to_run = 1
    refresh_rate = 60

    # for i in range(refresh_rate * time_to_run):
    #     viz.update()
    #     time.sleep(1.0/refresh_rate)
    
    input("ENTER TO EXIT.")
    viz.close_viz()
    