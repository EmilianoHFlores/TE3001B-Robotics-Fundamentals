import numpy as np
import time
## Get the jacobian matrix from the D_Hanterberg matrix for a robot

def jacobian(D_Hanterberg):
    # get the number of joints
    n = len(D_Hanterberg)
    # get the number of rows and columns
    rows, cols = D_Hanterberg.shape
    # create a list of jacobian matrices
    jacobians = []
    dn = 1
    dns = []
    R = 1
    Rs = []
    for j in range(n):
        theta = D_Hanterberg[j][0]
        alpha = D_Hanterberg[j][1]
        a = D_Hanterberg[j][2]
        d = D_Hanterberg[j][3]

        # Construct Position Vector
        dn = dn * np.array([[a*np.cos(theta)], [a*np.sin(theta)], [d]])
        dns.append(dn)
        holder = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha)],
                          [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha)],
                          [0, np.sin(alpha), np.cos(alpha)]])
        
        #print(holder)

        # Construct Rotation Matrix
        R = R * holder
        Rs.append(R)

    # iterate through the number of joints
    #for i in range(n):
    # create a jacobian matrix
    jacobian = np.zeros((6, n))
    #print(jacobian)
    # iterate through the number of joints
    for j in range(n):
        # get the rotation matrix
        R = Rs[j]
        # get the position vector
        p = dns[j]
        # get the position vector of the end effector
        p_end = dns[n-1]
        # get the position vector of the joint
        p_joint = p_end - p

        # dot product of the rotation matrix and the z axis
        #z = np.product(R, np.array([[0],[0],[1]]))
        #print(R)
        holder = np.array([[0],[0],[1]])
        
        z = np.matmul(R, holder)
        #print(z)
        # get the cross product of the z axis and the position vector of the joint
        cross = np.cross(z.T, p_joint.T).T

        #print(cross)
        # set the values of the jacobian matrix
        for k in range(3):
            jacobian[k, j] = cross[k]
            jacobian[k+3, j] = z[k]
        # append the jacobian matrix to the list of jacobian matrices
        #jacobians.append(jacobian)
    return jacobian  

theta_ = 0
theta_increment = np.pi/8
# Example
while True:

  D_Hanterberg = np.array([[theta_, 0, 15, 0], [0, 0, 40, 0], [0, 0, 20, 0]])
  jacobians = jacobian(D_Hanterberg)
  np.set_printoptions(suppress=True)
  print(jacobians)
  #print("\n")
  theta_ += theta_increment

  time.sleep(1)
# Output: