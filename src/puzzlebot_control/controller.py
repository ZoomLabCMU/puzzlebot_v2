import numpy as np
import quadprog 
from planner import Planner
from utils import Utils
from cvxopt import matrix
from cvxopt.blas import dot
from cvxopt.solvers import qp, options
from cvxopt import matrix, sparse

class Controller:
    def __init__(self, N, vlim=0.12, wlim=3.4, p_vlim=0.12, p_wlim=0.2, 
            vcf=1.0, wcf=1.0, 
            knob_length=7.5, body_length=34, padding_length=1, 
            vw_pt=[[0.037,-2.19],[0.1164,0]], vw_pt_pilot=[[0.412,-1.571],[0.548,0]]):
        self.N = N
        self.vlim = vlim
        self.wlim = wlim
        self.p_vlim = p_vlim
        self.p_wlim = p_wlim
        self.vcf = vcf
        self.wcf = wcf

        self.vw_pt = np.array(vw_pt)
        self.vw_pt_pilot = np.array(vw_pt_pilot)

        self.knob_length = knob_length
        self.body_length = body_length
        self.padding_length = padding_length
        self.Lx = (self.knob_length*1.5 + self.padding_length) / 1e3
        self.L = self.body_length / 1e3
        self.r = self.knob_length / 2 / 1e3

        self.planner = Planner(N, L=self.L, r=self.r, 
                                pad=self.padding_length/1e3)
        self.utils = Utils()

        self.prev_du = np.zeros([2, N])

        self.id_pair = None
        self.contact_pair = None
        self.mid_pair = None
        self.mcontact_pair = None

        # params for behavior pair pool
        self.pair_dict = None
        self.colm_idx = 0
        self.connected_dict = {}
        self.disconn_dict = {}
        self.pairs_exec = {}
        self.robot_busy = np.zeros(N, dtype=bool)

    def contact_dis(self, x, goal, p_dict, goal_wt, dx_cf=1e-4, dt_cf=1e-4, dis_th=2e-3, dts=[], pilot_ids=[]):
        '''
        goal: target du, 2-by-N;
        p_dict: dict of contact pairs to maintain
                keys: id pair tuple, values: contact array - 2-by-2 matrix
        goal_wt: weights of goals, N dim vector
        dx_cf: weight of linear velocity in goal
        dt_cf: weight of angular velocity in goal
        dis_th: threshold of distance between contact pairs
        dts: the time difference
        pilot_ids: ids of the pilot robots in the system
        '''
        N = x.shape[1]
        M = len(p_dict)
        if M == 0: return goal
        du = goal.copy()

        # back up du
        contact_pairs = ()
        id_pairs = []
        for p in p_dict:
            contact_pairs += (p_dict[p],)
            id_pairs.append(list(p))
        id_pairs = np.array(id_pairs).T
        cdu = self.go_to_contact_pairs(x, p_dict, dif_th=dis_th)
        cdu = self.cap_constr(cdu, pilot_ids=pilot_ids)

        dis_arr = self.utils.get_pair_dis(x, p_dict)
        disconn_ps = []

        Q = np.zeros([2*N, 2*N])
        Q = Q + np.eye(2*N, 2*N)*1e-4
        diag = np.zeros(2*N)
        diag[::2] = dx_cf * goal_wt
        diag[1::2] = dt_cf * goal_wt
        np.fill_diagonal(Q, diag)

        P = np.zeros(2*N)
        P[::2] = goal[0, :] * dx_cf * goal_wt
        P[1::2] = goal[1, :] * dt_cf * goal_wt

        A = np.zeros([4*M+4*N+4*N, 2*N])
        b = np.zeros(4*M+4*N+4*N)
        #  A[4*M:4*(M+N), :]= np.vstack((np.eye(2*N, 2*N), -np.eye(2*N, 2*N)))
        A[4*(M+N):(4*M+8*N), :]= np.vstack((np.eye(2*N, 2*N), -np.eye(2*N, 2*N)))
        vw_pt = self.utils.get_vw_ks(self.vw_pt)
        k0n, b0n = vw_pt[0, :]
        k1n, b1n = vw_pt[1, :]
        vw_pt_pilot = self.utils.get_vw_ks(self.vw_pt_pilot)
        k0p, b0p = vw_pt_pilot[0, :]
        k1p, b1p = vw_pt_pilot[1, :]
        k0, b0, k1, b1 = [k0n, b0n, k1n, b1n]

        # for vw constraints
        for m in range(M):
            key = p_dict.keys()[m]
            md = p_dict[key]
            if dis_arr[m] > dis_th*1.5: 
                print 'disconnected:', key, " md:", md, ' dis:', dis_arr[m]
                disconn_ps += list(key)
                continue
            for i in range(2):
                ii = key[i]
                th = x[2, ii]
                dx, dy = md[:, i]
                sn = i*2-1
                A[m*4:(4*m+2), 2*ii] = dts[i] * sn * np.array([np.cos(th), np.sin(th)])
                A[m*4:(4*m+2), 2*ii+1] = dts[i] * sn * np.array([
                                        -dx*np.sin(th) - dy*np.cos(th),
                                        dx*np.cos(th) - dy*np.sin(th)
                                        ])
                b[m*4:(4*m+2)] += -sn * np.array([
                                x[0,ii] + dx*np.cos(th) - dy*np.sin(th),
                                x[1,ii] + dx*np.sin(th) + dy*np.cos(th)
                                ])
            A[(4*m+2):(4*m+4)] = - A[m*4:(4*m+2)]
            b[(4*m+2):(4*m+4)] = - b[m*4:(4*m+2)]
            b[4*m:(4*m+4)] -= dis_th
        #  b[4*M:(4*M+4*N):2] = -self.vlim
        #  b[(4*M+1):(4*M+4*N):2] = -self.wlim

        # for vw constraints
        b[4*M:4*M+2*N] = b0
        b[4*M+2*N:4*M+4*N] = b1

        for i in range(N):
            if i in pilot_ids:
                k0, b0, k1, b1 = [k0p, b0p, k1p, b1p]
                b[4*M+2*i:4*M+2*(i+1)] = b0
                b[4*M+2*(i+N):4*M+2*(i+N+1)] = b1
            else:
                k0, b0, k1, b1 = [k0n, b0n, k1n, b1n]

            A[4*M+i*2, 2*i:2*(i+1)] = [k0, 1]
            A[4*M+i*2+1, 2*i:2*(i+1)] = [k0, -1]
            A[4*M+i*2+2*N, 2*i:2*(i+1)] = [k1, 1]
            A[4*M+i*2+2*N+1, 2*i:2*(i+1)] = [k1, -1]

            if goal[0, i] < 0:
                A[4*M+2*i:4*M+2*(i+1), 2*i:2*(i+1)] *= -1
                A[4*M+2*(i+N):4*M+2*(i+N+1), 2*i:2*(i+1)] *= -1

        # velocity limits
        b[(4*M+4*N):(4*M+8*N):2] = -self.vlim
        b[(4*M+4*N+1):(4*M+8*N):2] = -self.wlim
        pilot_ids = np.array(pilot_ids, dtype=int)
        b[4*M+4*N+2*pilot_ids] = -self.p_vlim
        b[4*M+4*N+2*pilot_ids+1] = -self.p_wlim
        b[4*M+6*N+2*pilot_ids] = -self.p_vlim
        b[4*M+6*N+2*pilot_ids+1] = -self.p_wlim

        np.set_printoptions(suppress=True)
        print 'goal:', goal

        try:
            A = -A
            b = -b
            options['show_progress'] = False
            prob = qp(matrix(Q), matrix(-P), matrix(A), matrix(b), initvals={'x': matrix(du.T.flatten())})
            #  print 'status:', prob['status']
            if prob['status'] is not 'optimal':
                raise Exception("status: " + prob['status'])
            #  print 'obj value:', prob['primal objective']
            res = prob['x']
            res = np.array(res)
            du[:, :] = res.reshape([N, 2]).T

        except Exception as e:
            print(e)
            du += cdu
            du = self.cap_constr(du, pilot_ids=pilot_ids)

        du[:, disconn_ps] = cdu[:, disconn_ps]
        norm = np.linalg.norm(du, axis=0)
        du[:, norm<1e-5] = 0
        return du

    def go_to_contact_pairs(self, x, p_dict, dif_th=1.0e-3, angle_en=True, use_bc=False, bc_th=10):
        '''
        Assume no conflicting pairs for now
        p_dict: dict of contact pairs to maintain
                keys: id pair tuple, values: contact array - 2-by-2 matrix
        thth: target angle difference in config
        angle_cf: enable angle vector pointing to the other robot
        '''
        N = self.N
        du = np.zeros([2, N])
        M = len(p_dict.keys())
        if M == 0: return du

        J1 = np.zeros([3, 2])
        J2 = np.zeros([3, 2])
        J1[2, 1] = 1
        J2[2, 1] = 1
        angle_mat = np.array([[0.5, -0.5], [-0.5, 0.5]])

        for m in range(M):
            i0, i1 = p_dict.keys()[m]
            md = p_dict[(i0, i1)]

            angle_cf = 1 if angle_en else 0

            gt0, gt1 = x[2, [i0, i1]]

            x0 = x[:, i0].copy()
            x1 = x[:, i1].copy()
            x0[0:2] = self.utils.getR(x[2,i0]).dot(md[:, 0]) + x[0:2, i0]
            x1[0:2] = self.utils.getR(x[2,i1]).dot(md[:, 1]) + x[0:2, i1]
            J1[0:2, 0] = [np.cos(gt0), np.sin(gt0)]
            J2[0:2, 0] = [np.cos(gt1), np.sin(gt1)]

            dif_q = x1 - x0
            if np.linalg.norm(dif_q[0:2]) < dif_th:
                continue

            vec_norm = np.linalg.norm(dif_q[0:2])
            dis_diff = np.linalg.norm(x[0:2, i0] - x[0:2, i1])

            angle_th = np.zeros(2)
            md_dis = np.linalg.norm(md, axis=0)
            if np.any(md_dis < 0.7*self.L):
                for ag in angle_mat:
                    dis = self.utils.project_robot_dis(ag, md)
                    if dis < self.L: continue
                    angle_th[:] = ag
                    angle_cf = 0

            # first robot in the pair
            dif_q[2] = angle_cf * np.arctan2(dif_q[1], dif_q[0])
            dif_q0 = dif_q.copy()
            dif_q0[2] += (1 - angle_cf) * angle_th[0]
            dif_q0[2] = self.utils.wrap_pi_2(dif_q0[2])
            dif_q0[2] -= gt0
            du[:, i0] += np.linalg.pinv(J1).dot(dif_q0)

            # second robot in the pair
            dif_q1 = - dif_q
            dif_q1[2] = angle_cf * np.arctan2(dif_q1[1], dif_q1[0])
            dif_q1[2] += (1 - angle_cf) * angle_th[1]
            dif_q1[2] = self.utils.wrap_pi_2(dif_q1[2])
            dif_q1[2] -= gt1
            du[:, i1] += np.linalg.pinv(J2).dot(dif_q1)

        return du

    def go_to_pair_pool(self, x, eth=1.5e-3, use_bc=False, pilot_ids=[], dts=[], gth=5e-2):
        N = self.N
        du = np.zeros([2, N])
        pilot_num = len(pilot_ids)
        robot_busy = self.robot_busy
        pair_dict = self.pair_dict
        pairs_exec = self.pairs_exec
        connected_dict = self.connected_dict
        if pair_dict is None:
            pair_dict = self.planner.generate_pair_pool([2, 2], x[0:2, :])
            self.pair_dict = pair_dict
            
            # check number of column formation pairs
            for k in pair_dict.keys():
                cp = pair_dict[k]
                if np.min(np.abs(cp)) > (self.L/2 - eth):
                    self.colm_idx += 1
        if len(pair_dict) == len(connected_dict) and len(pairs_exec) == 0:
            return du
        print 'pair_dict:', pair_dict
        print 'connected_dict:', connected_dict 

        # update already connected pairs
        connected_dict = self.planner.update_contact_with_ids(x, connected_dict)

        # decide which pairs to execute
        for k in pair_dict.keys():
            if np.any(robot_busy[list(k)]):
                continue
            if k in connected_dict:
                continue
            cp = pair_dict[k]
            if np.min(np.abs(cp)) < (self.L/2 - eth):
                if len(connected_dict) < self.colm_idx:
                    continue
            robot_busy[list(k)] = True
            pairs_exec[k] = cp
        print 'pairs_exec:', pairs_exec

        # execute pair
        has_goal = np.zeros(N, dtype=bool)
        goal_conn_id = np.arange(N)
        conn_list = np.array(connected_dict.keys())
        for p in pairs_exec.keys():
            cp = pairs_exec[p]
            if np.min(np.abs(cp)) > (self.L/2 -eth):
                pairs_exec[p] = self.planner.update_contact_with_ids(x, {p: cp})[p]
            for pi in p:
                has_goal[pi] = True
                count = np.argwhere(conn_list == pi)
                for cidx in count:
                    goal_conn_id[conn_list[cidx[0], cidx[1]-1]] = pi

        du = self.go_to_contact_pairs(x, pairs_exec, dif_th=eth, use_bc=use_bc)
        du = self.p_control(du)
        du = self.cap_constr(du, pilot_ids=pilot_ids)
        goal_wt = np.zeros(N) + 0.5
        goal_wt[has_goal] = 1
        du[:, ~has_goal] = self.si_to_du(x[:, ~has_goal],
                            x[0:2, goal_conn_id[~has_goal]] - x[0:2, ~has_goal])
        du[:, :] = du[:, goal_conn_id]
        du = self.cap_constr(du, pilot_ids=pilot_ids)
        print 'du before:', du
        conn_bias = self.utils.connection_bias(x, connected_dict)
        conn_bias = self.cap_constr(conn_bias, pilot_ids=pilot_ids)
        du = (1 - gth) * du + gth * self.si_to_du(x, conn_bias)

        du = self.contact_dis(x, du, connected_dict, goal_wt, 
                            dx_cf=1, dt_cf=1e-3, dis_th=eth, dts=dts, pilot_ids=pilot_ids)

        # avoid moving pilot robots in the initial stage
        if len(connected_dict) < len(pilot_ids):
            du[:, pilot_ids] = 0
    
        # check which pair is done execute
        pdis = self.utils.get_pair_dis(x, pairs_exec)
        print 'pdis:', pdis
        done_idx = np.where(pdis < eth)[0]
        done_pairs = []
        for idx in done_idx:
            ip = pairs_exec.keys()[idx]
            connected_dict[ip] = pairs_exec[ip]
            done_pairs.append(ip)
            robot_busy[list(ip)] = 0
            du[:, list(ip)] = 0

        for ip in done_pairs:
            pairs_exec.pop(ip)
            return np.zeros([2, N])

        done_idx = np.where(pdis < 3*eth)[0]
        for idx in done_idx:
            ip = pairs_exec.keys()[idx]
            du[0, list(ip)] *= 50
            du[1, list(ip)] *= 3

        #  du = self.p_control(du)
        du = self.cap_constr(du, pilot_ids=pilot_ids)
        
        # update saved variables
        self.connected_dict = connected_dict
        self.robot_busy = robot_busy
        self.pairs_exec = pairs_exec
        return du

    def cap(self, _du):
        du = np.array(_du, copy=True)
        idx = np.abs(du[0, :]) > self.vlim
        du[0, idx] = self.vlim * np.sign(du[0, idx])
        idx = np.abs(du[1, :]) > self.wlim
        du[1, idx] = self.wlim * np.sign(du[1, idx])

        return du

    def p_control(self, du):
        du[0, :] *= self.vcf
        du[1, :] *= self.wcf
        return du

    def cap_constr(self, _du, pilot_ids=[]):
        assert(_du.shape[1] == self.N)
        N = self.N
        du = np.array(_du, copy=True)
        du = self.cap(du)

        vw_pt = self.utils.get_vw_ks(self.vw_pt)
        k0, b0 = vw_pt[0, :]
        k1, b1 = vw_pt[1, :]

        Q = np.eye(2*N)
        P = np.zeros(2*N)
        P[::2] = du[0, :]
        P[1::2] = du[1, :]

        A = np.zeros([8*N, 2*N])
        A[4*N:8*N, :]= np.vstack((np.eye(2*N, 2*N), -np.eye(2*N, 2*N)))
        b = np.zeros(8*N)

        # for vw constraints
        b[0:2*N] = b0
        b[2*N:4*N] = b1
        b[4*N:8*N:2] = -self.vlim
        b[(4*N+1):(8*N):2] = -self.wlim

        # for vw constraints
        for i in range(N):
            A[i*2, 2*i:2*(i+1)] = [k0, 1]
            A[i*2+1, 2*i:2*(i+1)] = [k0, -1]
            A[i*2+2*N, 2*i:2*(i+1)] = [k1, 1]
            A[i*2+2*N+1, 2*i:2*(i+1)] = [k1, -1]

            if du[0, i] < 0:
                A[2*i:2*(i+1), 2*i:2*(i+1)] *= -1
                A[2*(i+N):2*(i+N+1), 2*i:2*(i+1)] *= -1
            
            if i in pilot_ids:
                b[4*N+2*i] = - self.p_vlim
                b[4*N+2*i+1] = - self.p_wlim
                b[6*N+2*i] = - self.p_vlim
                b[6*N+2*i+1] = - self.p_wlim

        np.set_printoptions(suppress=True)
        try:
            res = quadprog.solve_qp(Q, P, A.T, b)
            res = res[0]
            
            du[:, :] = res.reshape([N, 2]).T
        except:
            print 'error qp in cap_constr'

        du[np.abs(du) < 1e-5] = 0
        return du

    def go_to_goal(self, x, goal, weights=[1,1], eth=1e-3):
        '''
        x: 3-by-N matrix, goal: 2-by-N matrix or 3-by-N
        '''
        N = x.shape[1]
        weights = np.array([weights]).T
        dis_th = 0.1
        du = np.zeros([2, N])

        diff = goal[0:2, :] - x[0:2, :]
        if np.linalg.norm(diff * weights) < eth:
            return du
        if goal.shape[0] < 3:
            goal = np.vstack((goal, np.arctan2(diff[1, :], diff[0, :])))
        for i in range(N):
            th = x[2, i]
            dif_i = goal[:, i] - x[:, i]
            dif_i[2] = np.arctan2(dif_i[1], dif_i[0])
            dif_i[2] = self.utils.wrap_pi_2(goal[2, i])
            dif_i[2] -= th
            J_inv = np.array([[np.cos(th), np.sin(th), 0], [0, 0, 1]])
            du[:, i] = J_inv.dot(dif_i)

        du[:, np.linalg.norm(diff[0:2, :] * weights) < eth] = 0
        return du

    def forward_y(self, x, eth=1.5e-3, dts=None, gth=1e-2, pilot_ids=[], stop_x=0.5, angle_bias=-1.0):
        '''
        Input: x: 3-by-N matrix 
            gth: coeff for connection pair attraction
        Output: du: 1-by-2 vector
        '''
        N = self.N
        du = np.zeros([2, N])
        if np.min(x[0, :]) > stop_x: return du
        du[0, :] = self.vlim * 0.5
        du[1, :] = -x[2, :] + angle_bias
        du = self.cap(du)
        connected_dict = self.connected_dict
        conn_bias = self.utils.connection_bias(x, connected_dict)
        conn_bias = self.cap_constr(conn_bias, pilot_ids=pilot_ids)
        du = (1 - gth) * du + gth * self.si_to_du(x, conn_bias)
        du = self.cap_constr(du, pilot_ids=pilot_ids)

        goal_wt = np.ones(N)
        du = self.contact_dis(x, du, connected_dict, goal_wt, dx_cf=1e-2, dt_cf=1e-3, dis_th=eth, pilot_ids=pilot_ids, dts=dts)
        du = self.p_control(du)
        du = self.cap_constr(du, pilot_ids=pilot_ids)

        return du

    def anti_rendezvous(self, x, eth=1e-2, dts=None, gth=None, pilot_ids=[], stop_x=0.0):
        N = self.N
        du = np.zeros([2, N])
        pair_dict = self.pair_dict
        connected_dict = self.connected_dict
        disconn_dict = self.disconn_dict
        if len(pair_dict) == 0:
            return du
        if len(disconn_dict) == len(pair_dict):
            return du

        pair_dict = self.planner.update_contact_with_ids(x, pair_dict)

        center = np.mean(x[0:2, :], axis=1)
        dx = center[:, np.newaxis] - x[0:2, :]
        dx = - dx
        du = self.si_to_du(x, dx)

        pdis = self.utils.get_pair_dis(x, pair_dict)
        done_idx = np.where(pdis > eth)[0]
        done_pairs = []
        for idx in done_idx:
            ip = pair_dict.keys()[idx]
            disconn_dict[ip] = pair_dict[ip]
            done_pairs.append(ip)
            du[:, list(ip)] = 0

        for ip in done_pairs:
            if ip in connected_dict:
                connected_dict.pop(ip)
            du[:, list(ip)] = 0

        du = self.p_control(du)
        du = self.cap_constr(du, pilot_ids=pilot_ids)
        return du

    def du_to_si(self, x, du):
        N = du.shape[1]
        assert(x.shape[1] == N)
        dx = np.zeros([2, N])
        dx[0, :] = np.cos(x[2, :] + du[1, :]) * du[0, :]
        dx[1, :] = np.sin(x[2, :] + du[1, :]) * du[0, :]
        return dx

    def si_to_du(self, x, dx):
        N = dx.shape[1]
        du = self.go_to_goal(x, x[0:2, :] + dx)
        return du
    
