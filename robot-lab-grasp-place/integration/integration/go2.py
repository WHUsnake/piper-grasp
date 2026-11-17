"""Torque-driven Go2 and the author's 45-D/history-10 MoE-CTS observation contract.

No hardware interface, arm emulation, navigation planner or automatic route replay.
"""
import json
import math
from pathlib import Path


class Go2World:
    def __init__(self, assets):
        import mujoco
        import numpy as np
        import torch

        self.mj, self.np, self.torch = mujoco, np, torch
        self.assets = Path(assets)
        self.config = json.loads((self.assets / 'robot/policy_config.json').read_text())
        self.start = json.loads((self.assets / 'start.json').read_text())
        self.model = mujoco.MjModel.from_xml_path(str(self.assets / 'scene.xml'))
        self.data = mujoco.MjData(self.model)
        c = self.config
        self.model.opt.timestep = c['simulation_dt']
        self.model.opt.integrator = getattr(mujoco.mjtIntegrator, 'mjINT_' + c['integrator'].upper())
        self.model.opt.solver = getattr(mujoco.mjtSolver, 'mjSOL_' + c['solver'].upper())
        self.model.opt.iterations = c['solver_iterations']
        self.model.opt.ls_iterations = c['line_search_iterations']
        joints = [self.model.joint(n).id for n in c['policy_joint_names']]
        self.qi, self.vi = self.model.jnt_qposadr[joints], self.model.jnt_dofadr[joints]
        actuators = [np.flatnonzero(self.model.actuator_trnid[:, 0] == j) for j in joints]
        if len(joints) != 12 or any(len(a) != 1 for a in actuators):
            raise ValueError('Go2 policy requires twelve uniquely actuated joints')
        self.ai = np.asarray([a[0] for a in actuators])
        self.defaults = np.asarray(c['default_angles'], dtype=np.float32)
        torch.set_num_threads(1)
        self.policy = torch.jit.load(str(self.assets / 'robot/policy.pt'), map_location='cpu').eval()
        self.reset()

    def reset(self):
        self.stance_target = None
        self.mj.mj_resetData(self.model, self.data)
        self.data.qpos[:3] = self.start['position']
        yaw = self.start['yaw']
        self.data.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        self.data.qpos[self.qi] = self.defaults
        self.policy.reset()
        self.action = self.np.zeros(12, dtype=self.np.float32)
        self.target = self.defaults.copy()
        self.counter = 0
        self.mj.mj_forward(self.model, self.data)

    def infer(self, command):
        if self.stance_target is not None:
            return
        np, c, d = self.np, self.config, self.data
        w, x, y, z = d.qpos[3:7]
        gravity = np.array([2 * (-z*x+w*y), -2 * (z*y+w*x),
                            1-2*(w*w+z*z)], dtype=np.float32)
        obs = np.r_[d.qvel[3:6]*c['ang_vel_scale'], gravity,
                    np.asarray(command)*np.asarray(c['command_scale']),
                    d.qpos[self.qi]-self.defaults, d.qvel[self.vi]*c['dof_vel_scale'],
                    self.action].astype(np.float32)
        if obs.shape != (45,) or not np.isfinite(obs).all():
            raise ValueError('Invalid MoE observation')
        with self.torch.inference_mode():
            action = self.policy(self.torch.from_numpy(obs).unsqueeze(0))
        if isinstance(action, tuple):
            action = action[0]
        self.action = np.clip(action.numpy().reshape(12), -c['action_clip'], c['action_clip'])
        self.target = self.defaults + c['action_scale'] * self.action

    def step(self, command, substeps=1, before_step=None, after_step=None):
        np, d, c = self.np, self.data, self.config
        # Preserve the locomotion/trajectory control period while resolving
        # small gripper contacts at a finer numerical integration interval.
        timestep = self.model.opt.timestep
        try:
            self.model.opt.timestep = timestep/substeps
            for _ in range(substeps):
                checkpoint = before_step() if before_step else None
                velocity = d.qvel[self.vi]
                target = self.target if self.stance_target is None else self.stance_target
                kp = np.asarray(c['kps']) if self.stance_target is None else 80.
                kd = np.asarray(c['kds']) if self.stance_target is None else 3.
                effort = (target-d.qpos[self.qi])*kp - velocity*kd
                motor = c['motor']
                if motor['use_torque_speed_curve']:
                    limit = np.where(velocity*effort > 0, motor['Y1'], motor['Y2'])
                    falling = limit*(1-(np.abs(velocity)-motor['X1'])/(motor['X2']-motor['X1']))
                    limit = np.where(np.abs(velocity) < motor['X1'], limit, np.maximum(falling, 0))
                    effort = np.clip(effort, -limit, limit)
                d.ctrl[self.ai] = effort
                self.auxiliary_control()
                self.mj.mj_step(self.model, d)
                if after_step and after_step(checkpoint) is False:
                    return
        finally:
            self.model.opt.timestep = timestep
        self.counter += 1
        if self.counter % c['control_decimation'] == 0:
            self.infer(command if d.time > 1 else [0, 0, 0])

    def auxiliary_control(self):
        """Optional non-leg actuators; the locomotion policy still has 12 actions."""
        pass

    def begin_stance_hold(self):
        self.stance_target = self.data.qpos[self.qi]+(self.data.ctrl[self.ai]+3.*self.data.qvel[self.vi])/80.

    def end_stance_hold(self):
        if self.stance_target is not None:
            self.target = self.data.qpos[self.qi]+(self.data.ctrl[self.ai]+
                self.np.asarray(self.config['kds'])*self.data.qvel[self.vi])/self.np.asarray(self.config['kps'])
            self.action = ((self.target-self.defaults)/self.config['action_scale']).astype(self.np.float32)
            self.policy.reset()
            self.stance_target = None

    def restore_state(self, path):
        import hashlib
        saved = json.loads(Path(path).read_text())
        if saved.get('model_sha256') != hashlib.sha256((self.assets/'scene.xml').read_bytes()).hexdigest():
            raise ValueError('Snapshot does not match this physics model')
        positions = self.np.asarray(saved['qpos'], dtype=float)
        if positions.shape != (self.model.nq,) or not self.np.isfinite(positions).all():
            raise ValueError('Invalid snapshot joint positions')
        for index in range(self.model.njnt):
            address = self.model.jnt_qposadr[index]
            if self.model.jnt_type[index] == 0:
                if abs(self.np.linalg.norm(positions[address+3:address+7])-1.) > 1e-5:
                    raise ValueError('Invalid snapshot free-body orientation')
                xyz = positions[address:address+3]
                if self.np.max(self.np.abs(xyz[:2])) > 10 or not -1 <= xyz[2] <= 4:
                    raise ValueError('Snapshot free body is outside the simulation room')
            if self.model.jnt_limited[index] and not self.model.jnt_range[index, 0]-.02 <= positions[address] <= self.model.jnt_range[index, 1]+.02:
                raise ValueError('Snapshot joint exceeds model limits')
        self.data.qpos[:] = positions
        self.data.qvel[:] = 0
        self.data.time = 2.
        self.target = positions[self.qi].copy()
        if hasattr(self, 'arm_qi'):
            self.arm_target = positions[self.arm_qi].copy()
            self.jaw_target = [float(positions[self.model.jnt_qposadr[self.model.joint('joint'+str(index)).id]]) for index in (7, 8)]
            self.data.ctrl[self.jaw_ai] = self.jaw_target
        self.mj.mj_forward(self.model, self.data)

    def snapshot(self):
        np, d = self.np, self.data
        self.mj.mj_kinematics(self.model, d)
        return {'ready': True, 't': float(d.time), 'speed': float(np.linalg.norm(d.qvel[:2])),
                'bodies': np.c_[d.xpos, d.xquat].round(6).tolist(),
                'pos': d.qpos[:3].tolist(), 'simulated': True}


class Go2PiperMoeWorld(Go2World):
    """The existing Go2/Piper rigid bodies with MoE-CTS legs and held arm."""
    def reset(self):
        super().reset()
        c = json.loads((self.assets/'robot/arm_hold.json').read_text())
        np, m, d = self.np, self.model, self.data
        joints = [m.joint('joint'+str(i)).id for i in range(1,7)]
        self.arm_qi, self.arm_vi = m.jnt_qposadr[joints], m.jnt_dofadr[joints]
        self.arm_ai = np.asarray([np.flatnonzero(m.actuator_trnid[:,0] == j)[0] for j in joints])
        self.arm_target = np.asarray(c['angles'])
        self.arm_kp, self.arm_kd = np.asarray(c['kps']), np.asarray(c['kds'])
        d.qpos[self.arm_qi] = self.arm_target
        jaw_joints = [m.joint('joint'+str(i)).id for i in (7,8)]
        self.jaw_ai = [m.actuator('joint'+str(i)).id for i in (7,8)]
        self.jaw_target = [c['jaw_opening_m']/2,-c['jaw_opening_m']/2]
        d.qpos[m.jnt_qposadr[jaw_joints]] = self.jaw_target
        d.ctrl[self.jaw_ai] = self.jaw_target
        self.mj.mj_forward(m,d)

    def auxiliary_control(self):
        np, m, d = self.np, self.model, self.data
        effort = (self.arm_kp*(self.arm_target-d.qpos[self.arm_qi])
                  -self.arm_kd*d.qvel[self.arm_vi]+d.qfrc_bias[self.arm_vi])
        d.ctrl[self.arm_ai] = np.clip(effort,m.actuator_ctrlrange[self.arm_ai,0],m.actuator_ctrlrange[self.arm_ai,1])
        d.ctrl[self.jaw_ai] = self.jaw_target
