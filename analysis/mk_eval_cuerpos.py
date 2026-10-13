"""Genera eval_sim_cuerpos.py a partir de eval_sim_cr.py: misma evaluacion, y ademas guarda
el error de posicion de cuerpos relativo al ancla (misma definicion que fk_cuerpos.py)."""
import os
d = os.path.dirname(os.path.abspath(__file__))
s = open(os.path.join(d, "eval_sim_cr.py"), encoding="utf-8").read()
s = s.replace("REF=[];ROB=[];EJP=[];EBP=[]", "REF=[];ROB=[];EJP=[];EBP=[];RBP=[];RBQ=[];MBP=[];MBQ=[]")
s = s.replace("        m=cmd.metrics\n",
              "        RBP.append(cmd.robot_body_pos_w.detach().cpu().numpy().copy()); RBQ.append(cmd.robot_anchor_quat_w.detach().cpu().numpy().copy())\n"
              "        MBP.append(cmd.body_pos_w.detach().cpu().numpy().copy()); MBQ.append(cmd.anchor_quat_w.detach().cpu().numpy().copy())\n"
              "        m=cmd.metrics\n")
i = s.index("np.savez_compressed"); j = s.index("e=ref-rob")
s = (s[:i] + "sys.path.insert(0,'/workspace/cr'); from cuerpos_comun import relativo, resumen\n"
     "RBP=np.array(RBP);RBQ=np.array(RBQ);MBP=np.array(MBP);MBQ=np.array(MBQ)\n"
     "CUERPOS_REL=resumen(relativo(RBP,RBQ),relativo(MBP,MBQ))\n" + s[j:])
s = s.replace("res['rmse_from_L2_mean_rad']", "res['cuerpos_rel_ancla_m']=CUERPOS_REL\nres['rmse_from_L2_mean_rad']")
assert "CUERPOS_REL" in s and "RBP.append" in s
open(os.path.join(d, "eval_sim_cuerpos.py"), "w", encoding="utf-8", newline="\n").write(s)
print("ok")
