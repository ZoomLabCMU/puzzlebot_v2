import numpy as np

class Utils:
    # Util functions
    def inv_g(self, g):
        #  assert(g.shape[0] == 3 or g.shape[0] == 4)
        #  assert(g.shape[1] == 3 or g.shape[1] == 4)

        if g.shape[0] != 3 or g.shape[1] != 3:
            print "We only handle 3x3 now"
            return
        
        g_inv = np.eye(3, 3)
        g_inv[0:2, 0:2] = g[0:2, 0:2].T
        g_inv[0:2, 2] = -g[0:2, 0:2].T.dot(g[0:2, 2])
        return g_inv

    def adjoint(self, g):
        if g.shape[0] != 3 or g.shape[1] != 3:
            print "We only handle 3x3 now"
            return
        
        ad = g.copy()
        ad[0, 2] = g[1, 2]
        ad[1, 2] = -g[0, 2]
        return ad

    def getR(self, th):
        return np.array([[np.cos(th), -np.sin(th)],
                        [np.sin(th), np.cos(th)]])

    def get_g(self, x):
        '''
        x is a 1d vector
        '''
        g = np.eye(3, 3)
        g[0:2, 0:2] = getR(x[2])
        g[0:2, 2] = x[0:2]
        return g

    def get_pair_dis(self, x, p_dict):
        M = len(p_dict.keys())
        dis = np.zeros(M)

        for m in range(M):
            ids = p_dict.keys()[m]
            cp = p_dict[ids]
            x0 = self.getR(x[2, ids[0]]).dot(cp[:,0])
            x0 += x[0:2, ids[0]]
            x1 = self.getR(x[2, ids[1]]).dot(cp[:,1])
            x1 += x[0:2, ids[1]]
            dis[m] = np.linalg.norm(x0 - x1)

        return dis

    def update_contact_bc(self, md, R):
        md_bc = np.zeros(md.shape)
        th = np.arctan2(md[1, :], md[0, :])
        md_bc[0, :] = R * np.cos(th)
        md_bc[1, :] = R * np.sin(th)
        return md_bc

    def is_in_pi(self, a):
        return (a<np.pi and a>-np.pi)

    def is_in_pi_2(self, a):
        return (a<np.pi/2 and a>-np.pi/2)
        
    def wrap_pi(self, a):
        return ((a + np.pi) % (2*np.pi) - np.pi)

    def wrap_pi_2(self, a):
        return ((a + np.pi/2) % np.pi - np.pi/2)

    def wrap_goal(self, goal, theta):
        '''
        goal: nd array, theta: nd array
        '''
        ng = goal.copy()

        tmp = self.wrap_pi(goal+np.pi)
        if not np.isscalar(ng):
            idx = np.abs(tmp - theta) < np.pi/2
            ng[idx] = tmp[idx]
        elif np.abs(tmp - theta) < np.pi/2:
            ng = tmp

        return ng

    def project_robot_dis(self, t, cp):
        '''
        t: len=2 vector
        cp: 2-by-2 matrix, colm i is contact pair for ri
        '''
        x = np.zeros([2, 2])
        for i in range(2):
            x[:, i] = - self.getR(t[i]).dot(cp[:, i])
        return np.linalg.norm(x[:, 0] - x[:, 1])

    def get_vw_ks(self, vw_pt):
        '''
        vw_pt: 2-by-2 matrix of [x0,y0; x1,y1]
        return: [k0,b0; k1,b1] for quadprog constraints
        '''
        assert(vw_pt.shape == (2, 2))
        x0, y0 = vw_pt[0, :]
        x1, y1 = vw_pt[1, :]
        assert(y0 < 0 and x0 > 0 and x1 > 0 and y1 == 0)

        k0 = - y0 / x0
        b0 = 0
        k1 = (y1 - y0) / (x1 - x0)
        b1 = -k1 * x0 + y0

        return np.array([[k0, b0], [-k1, b1]])

    def connection_bias(self, x, conn):
        '''
        x: 3-by-N array of poses
        conn: dict of connection pairs
        '''
        N = x.shape[1]
        diff = np.zeros([3, N])
        for k in conn.keys():
            k0, k1 = k
            diff[:, k0] = x[:, k1] - x[:, k0]
            diff[:, k1] = x[:, k0] - x[:, k1]

        return diff[0:2, :]
