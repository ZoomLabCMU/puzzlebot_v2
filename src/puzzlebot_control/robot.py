import time
import numpy as np
import cvxpy as cp
from controller import Controller

class Robot:
    def __init__(self, N, ips=[]):
        assert N == len(ips), "ip list and N not match"
        self.N = N
        self.ips = ips
        self.vlim = 1.0
        self.wlim = 6.0

        self.pose = np.zeros([3, N])
        self.prev_pose = np.ones([3, N])
        self.prev_cmd_pose = np.zeros([3, N])
        self.pose_time = [None] * N
        self.vel = np.zeros([3, N])
        self.prev_cmd_vel = np.zeros([2, N]) + 1e-4
        self.dts = None

        self.ctl = Controller(N, 
                        vlim=0.2, wlim=1.5, p_vlim=0.15, p_wlim=0.2,
                        vcf=1.0, wcf=1.0,
                        knob_length=10,
                        body_length=50,
                        padding_length=2.5)

        self.status = np.zeros(N)

    def update_prev_cmd_vel(self, data, i):
        try:
            vl, vr = data.split(',')
            vl, vr = [int(vl), int(vr)]
            self.prev_cmd_vel[:, i] = [vl, vr]
        except Exception as e:
            print e

    def fit_command_vel(self, gdu, th, cx=1.0, ct=2.0, v_min=60, v_max=180, pilot_ids=[], enable_diff=False):
        N = self.N
        dx = np.zeros([3, N])
        vel = np.zeros([2, N]) + 1 # +1: avoid jumping to next state
        print 'gdu:', gdu

        if not self.dts: return vel
        if np.linalg.norm(gdu) < th: return vel

        #  (n_linear, n_angular, p_linear, p_angular, pcf) = (0.0008, 0.0149, 0.00267, 0.002, 1e-6)
        (n_linear, n_angular, p_linear, p_angular, pcf) = (0.0007, 0.01, 0.00267, 0.01, 1e-6)
        pilot_idx = np.append(pilot_ids, pilot_ids).astype(int) * 2
        pilot_idx[0:len(pilot_ids)] += 1
        v_max_ls = np.zeros(2*N) + v_max
        v_min_ls = np.zeros(2*N) + v_min
        v_max_ls[pilot_idx] /= 2.0
        v_min_ls[pilot_idx] /= 1.0
        v_max_ls[v_max_ls < v_min_ls] = v_min_ls[v_max_ls < v_min_ls]
        M = v_max + v_min

        v = cp.Variable(2*N)
        s = cp.Variable(2*N, integer=True)
        A = np.zeros([4*N, 2*N])
        b = np.zeros(4*N)
        constrt = []

        for i in range(N):
            dt = self.dts[i]*5
            if not dt: return vel
            dt = 1

            linear, angular = [n_linear, n_angular]
            if i in pilot_ids:
                linear, angular = [p_linear, p_angular]

            A[4*i, (2*i):(2*i+2)] = dt*cx * linear * (np.zeros(2)+1)
            #  A[4*i+1, (2*i):(2*i+2)] = [-r/L, r/L]
            #  A[4*i+1, (2*i):(2*i+2)] *= dt*8*ct
            A[4*i+1, (2*i):(2*i+2)] = dt*ct* np.array([-1,1]) * angular
            A[(4*i+2):(4*i+4), (2*i):(2*i+2)] = np.eye(2, 2) * pcf
            b[4*i:(4*i+2)] = [cx*gdu[0,i], gdu[1,i]*ct]
            b[(4*i+2):(4*i+4)] = self.prev_cmd_vel[:, i] * pcf

        if enable_diff:
            v_max_ls = v_max_ls.tolist()
            v_min_ls = v_min_ls.tolist()
            constrt = [v_min <= v+cp.multiply(M, s), v+cp.multiply(M, s) <= v_max, s<=1, s>=0]
        else:
            neg_idxs = np.where(gdu[0, :] < 0)[0]
            neg_idxs = np.array([[neg_idxs*2],[neg_idxs*2+1]]).T.flatten()
            v_min_ls_cp = v_min_ls.copy()
            v_min_ls[neg_idxs] = - v_max_ls[neg_idxs]
            v_max_ls[neg_idxs] = - v_min_ls_cp[neg_idxs]
            v_max_ls = v_max_ls.tolist()
            v_min_ls = v_min_ls.tolist()
            constrt = [v_min_ls <= v, v <= v_max_ls]

        obj = cp.Minimize(cp.sum_squares(A * v - b))
        prob = cp.Problem(obj, constrt)
        prob.solve(verbose=False)

        vel = v.value.astype(int).reshape([N,2]).T
        print 'vel:', vel
        vel[:, pilot_ids] = np.flipud(vel[:, pilot_ids])
        vel[:, np.linalg.norm(gdu, axis=0)<th] = 0

        return vel

    def behav_sequence(self, state):
        N = self.N
        vel = np.zeros([2, N])
        x = self.pose

        behav_seq = [self.ctl.go_to_pair_pool, self.ctl.go_to_pair_pool, self.ctl.go_to_pair_pool, self.ctl.go_to_pair_pool, self.ctl.forward_y, self.ctl.anti_rendezvous]
        #  behav_seq = [self.ctl.forward_y, self.ctl.anti_rendezvous]
        bc_seq = []
        pilot_ids = []

        if state >= len(behav_seq):
            state -= 1
            #  return vel, state

        bhav = behav_seq[state]
        if state in bc_seq:
            du = bhav(x, use_bc=True, eth=5e-3)
            vel = self.fit_command_vel(du, 1e-3, cx=1, ct=1.0, pilot_ids=pilot_ids)
        elif state < 2:
            #  du = bhav(x, eth=5e-3, dts=np.mean(self.dts))
            du = bhav(x, eth=2.0e-3, pilot_ids=pilot_ids, dts=np.array(self.dts)/10, gth=1e-1)
            #  du = bhav(x, eth=1.3e-3, pilot_ids=pilot_ids, dts=np.array(self.dts), gth=1e-1, stop_x=0.078)
            vel = self.fit_command_vel(du, 1e-3, cx=1, ct=1.0, v_min=70, pilot_ids=pilot_ids)
            vel[:, pilot_ids] = 40
        elif state == 2 or state == 3:
            self.ctl.vcf = 1.0
            self.ctl.wcf = 1.0
            #  du = bhav(x, eth=5e-3, gth=0.05, pilot_ids=pilot_ids, dts=np.array(self.dts)/2)
            du = bhav(x, eth=2.5e-3, gth=0.1, pilot_ids=pilot_ids, dts=np.zeros(N)+1e-2)
            vel = self.fit_command_vel(du, 1e-3, cx=1, ct=2.0, v_min=60, v_max=180, pilot_ids=pilot_ids)
        elif state == 4:
            self.ctl.wcf = 2.0
            du = bhav(x, eth=2.5e-3, gth=0.5, pilot_ids=pilot_ids, dts=np.array(self.dts), stop_x=0.078, angle_bias=-0.2)
            vel = self.fit_command_vel(du, 1e-3, cx=2, ct=1.0, v_min=110, pilot_ids=pilot_ids)
        elif state == 5:
            self.ctl.wcf = 5.0
            self.ctl.wlim = 3.0
            self.ctl.vcf = 5.0
            du = bhav(x, eth=3e-2, pilot_ids=pilot_ids, dts=np.array(self.dts)/10)
            vel = self.fit_command_vel(du, 1e-3, cx=1.0, ct=1.0, v_min=65, pilot_ids=pilot_ids, enable_diff=True)

        if np.linalg.norm(du) < 1e-4:
            state += 1
        return vel, state

    def keyboard_input_lr(self):
        du = np.zeros([2, self.N])
        txt = raw_input("Input cmd_vel in format [left right]: ")
        try:
            txt = txt.split()
            vs = [int(tx) for tx in txt]
            if len(vs) == 1:
                du[:, :] = vs[0]
            elif len(vs) == 2:
                du[0, :] = vs[0]
                du[1, :] = vs[1]
        except:
            return du

        # for calibration
        if self.pose_time[0] is None:
            self.pose_time[0] = time.time()
            return du

        time_now = time.time()
        pose_now = self.pose[:, 0].T.copy()
        elap_time = time_now - self.pose_time[0]
        v = np.linalg.norm(pose_now[0:2] - self.prev_cmd_pose[0:2, 0]) / elap_time
        w = (pose_now[2] - self.prev_cmd_pose[2, 0]) / elap_time
        print 'elap_time: ', elap_time
        print 'pose:', pose_now
        print 'pose diff: ', pose_now - self.prev_cmd_pose[:, 0]
        print 'v: ', v, ', w: ', w
        self.pose_time[0] = time_now
        self.prev_cmd_pose[:, 0] = pose_now

        return du

    def keyboard_input_each(self, pilot_ids=[]):
        N = self.N
        vel = np.zeros([2, N])
        txt = raw_input("Input cmd_vel in format [r0 left, r0 right, r1 left, ...]: ")
        v_list = []
        try:
            txt = txt.split()
            for tx in txt:
                v_list.append(int(tx))
        except:
            return vel

        if len(v_list) == 0:
            return vel

        if len(v_list) == 1:
            vel[:, :] =  v_list[0]
            return vel
        elif len(v_list) != 2*N and len(v_list) > 1:
            print 'Number of input velocity != 2*N'
            return vel

        v_list = np.array(v_list)
        vel[0, :] = v_list[0:2*N:2]
        vel[1, :] = v_list[1:2*N:2]
        vel[:, pilot_ids] = np.flipud(vel[:, pilot_ids])

        return vel
    
    def stop(self):
        return np.zeros([2, self.N])

