import os
import torch, numpy as np, json, glob, sys
from dataclasses import asdict
import mjlab.tasks           # noqa
import src.tasks             # noqa
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls
from mjlab.utils.torch import configure_torch_backends

TASK='Unitree-G1-Tracking-No-State-Estimation'
RUN=os.environ["RUN"]
MOTION=os.environ["MOTION"]
NENV=16

ck=sorted(glob.glob(RUN+'/model_*.pt'), key=lambda p:int(p.split('_')[-1].split('.')[0]))
CKPT=(RUN+"/model_%s.pt"%os.environ["CKPT_ITER"]) if os.environ.get("CKPT_ITER") else ck[-1]; print('[eval] checkpoint:',CKPT,flush=True)

configure_torch_backends()
dev='cuda:0'
env_cfg=load_env_cfg(TASK, play=True)      # play=True -> sin push_robot
agent_cfg=load_rl_cfg(TASK)
env_cfg.terminations={}                     # correr el motion COMPLETO
mc=env_cfg.commands['motion']
mc.motion_file=MOTION
mc.sampling_mode='start'                    # todos los envs arrancan en el frame 0
env_cfg.scene.num_envs=NENV
base=ManagerBasedRlEnv(cfg=env_cfg, device=dev, render_mode=None)
env=RslRlVecEnvWrapper(base, clip_actions=agent_cfg.clip_actions)
rc=load_runner_cls(TASK) or MjlabOnPolicyRunner
runner=rc(env, asdict(agent_cfg), device=dev)
runner.load(CKPT, load_cfg={'actor':True}, strict=True, map_location=dev)
policy=runner.get_inference_policy(device=dev)

cmd=base.command_manager.get_term('motion')
NST=int(cmd.motion.time_step_total)
print('[eval] pasos del motion: %d  (%.2f s a 50 Hz)'%(NST,NST/50.0),flush=True)

_o=env.get_observations()
obs=_o[0] if isinstance(_o,(tuple,list)) else _o
REF=[];ROB=[];EJP=[];EBP=[];RBP=[];RBQ=[];MBP=[];MBQ=[]
with torch.inference_mode():
    for i in range(NST):
        act=policy(obs)
        _s=env.step(act)
        obs=_s[0] if isinstance(_s,(tuple,list)) else _s
        REF.append(cmd.joint_pos.detach().cpu().numpy().copy())
        ROB.append(cmd.robot_joint_pos.detach().cpu().numpy().copy())
        RBP.append(cmd.robot_body_pos_w.detach().cpu().numpy().copy()); RBQ.append(cmd.robot_anchor_quat_w.detach().cpu().numpy().copy())
        MBP.append(cmd.body_pos_w.detach().cpu().numpy().copy()); MBQ.append(cmd.anchor_quat_w.detach().cpu().numpy().copy())
        m=cmd.metrics
        EJP.append(float(m['error_joint_pos'].mean())); EBP.append(float(m['error_body_pos'].mean()))
        if i%500==0: print('  paso %d/%d'%(i,NST),flush=True)
ref=np.array(REF); rob=np.array(ROB)      # (T, NENV, NJ)
sys.path.insert(0,'/workspace/cr'); from cuerpos_comun import relativo, resumen
RBP=np.array(RBP);RBQ=np.array(RBQ);MBP=np.array(MBP);MBQ=np.array(MBQ)
CUERPOS_REL=resumen(relativo(RBP,RBQ),relativo(MBP,MBQ))
e=ref-rob
NJ=e.shape[2]
G={'all':slice(0,NJ),'legs':slice(0,12),'waist':slice(12,15),'arms':slice(15,NJ)}
res={'n_joints':int(NJ),'n_envs':int(e.shape[1]),'n_steps':int(e.shape[0]),
     'checkpoint':CKPT,'motion':MOTION,
     'error_joint_pos_mean_L2':float(np.mean(EJP)),
     'error_joint_pos_final_L2':float(np.mean(EJP[-50:])),
     'error_body_pos_mean_m':float(np.mean(EBP))}
for k,sl in G.items():
    res['rmse_%s_rad'%k]=float(np.sqrt(np.mean(e[:,:,sl]**2)))
    res['rmse_%s_deg'%k]=float(np.degrees(np.sqrt(np.mean(e[:,:,sl]**2))))
res['cuerpos_rel_ancla_m']=CUERPOS_REL
res['rmse_from_L2_mean_rad']=float(np.mean(EJP)/np.sqrt(NJ))
print(json.dumps(res,indent=2))
json.dump(res,open("/workspace/cr/out/"+os.environ["TAG"]+".json",'w'),indent=2)
print('[eval] OK',flush=True)
