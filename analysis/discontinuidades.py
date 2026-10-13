#!/usr/bin/env python3
"""Cota formal del umbral de discontinuidad a partir de los limites de
velocidad de los actuadores, y lista de saltos de baileaxe57 y bailemix57."""
import os,re,sys,json
import numpy as np
sys.stdout.reconfigure(encoding="utf-8",errors="replace")
SP=os.path.join(os.path.dirname(os.path.abspath(__file__)),"insumos")
PAPER=os.environ.get("PAPER_DIR", os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))  # carpeta con data/ (disposicion original del proyecto)

# --- orden de articulaciones tal como aparece en el MJCF (= orden de mjlab) ---
xml=open(os.path.join(SP,"g1.xml"),encoding="utf-8").read()
joints=re.findall(r'<joint name="([^"]+)"',xml)
joints=[j for j in joints if j!="floating_base_joint"]
assert len(joints)==29, (len(joints),joints)

# --- actuador por articulacion, segun g1_constants.py ---
ACT={"5020":(37.0,25.0),"7520_14":(32.0,88.0),"7520_22":(20.0,139.0),
     "4010":(22.0,5.0),"waist_par":(37.0,50.0),"ankle_par":(37.0,50.0)}
def clase(j):
    if re.search(r"_(elbow|shoulder_pitch|shoulder_roll|shoulder_yaw|wrist_roll)_joint$",j): return "5020"
    if re.search(r"_(hip_pitch|hip_yaw)_joint$",j) or j=="waist_yaw_joint": return "7520_14"
    if re.search(r"_(hip_roll|knee)_joint$",j): return "7520_22"
    if re.search(r"_(wrist_pitch|wrist_yaw)_joint$",j): return "4010"
    if j in ("waist_pitch_joint","waist_roll_joint"): return "waist_par"
    if re.search(r"_ankle_(pitch|roll)_joint$",j): return "ankle_par"
    raise ValueError(j)
cls=[clase(j) for j in joints]
vlim=np.array([ACT[c][0] for c in cls])
FPS=50.0
dq_max=vlim/FPS          # salto maximo por cuadro admisible, rad

print("="*104)
print("TABLA A — limite de velocidad por articulacion y cota de salto por cuadro (referencia a 50 Hz)")
print("="*104)
print(f"{'#':>3} {'articulacion':30s} {'actuador':10s} {'v_lim rad/s':>11s} {'tau_lim Nm':>10s} {'dq_max rad/cuadro':>18s}")
for i,(j,c) in enumerate(zip(joints,cls)):
    print(f"{i:3d} {j:30s} {c:10s} {ACT[c][0]:11.1f} {ACT[c][1]:10.1f} {dq_max[i]:18.3f}")
print(f"\nCOTA FORMAL = min sobre las 29 articulaciones = {dq_max.min():.3f} rad/cuadro "
      f"({joints[int(dq_max.argmin())]} y demas del mismo actuador, v_lim={vlim.min():.0f} rad/s)")
print(f"Cota mas laxa = {dq_max.max():.3f} rad/cuadro (actuadores de {vlim.max():.0f} rad/s)")

res={"joints":joints,"clase_actuador":cls,"v_limit_rad_s":vlim.tolist(),
     "dq_max_rad_por_cuadro":dq_max.round(4).tolist(),"fps_referencia":FPS,
     "cota_formal_rad":round(float(dq_max.min()),4)}

# --- saltos en cada referencia ---
FUENTES=[("baileaxe57 (DESPLEGADA, funciono)",  os.path.join(PAPER,"data","baileaxe57.npz")),
         ("bailemix57 (NO convergio)",          os.path.join(SP,"bailemix57.npz"))]
res["referencias"]={}
for etiqueta,path in FUENTES:
    z=np.load(path); q=z["joint_pos"].astype(np.float64); fps=float(z["fps"][0])
    d=np.diff(q,axis=0); ad=np.abs(d)
    vel=ad*fps
    frac=vel/vlim[None,:]                       # fraccion del limite del actuador
    print("\n"+"="*104)
    print(f"TABLA B — {etiqueta}   [{os.path.basename(path)}]  {q.shape[0]} cuadros @ {fps:.0f} Hz = {q.shape[0]/fps:.2f} s")
    print("="*104)
    for umbral,nombre in [(0.5,"umbral EMPIRICO 0,5 rad"),(None,"cota FORMAL por actuador")]:
        if umbral is not None:
            fr,jt=np.nonzero(ad>umbral)
        else:
            fr,jt=np.nonzero(ad>dq_max[None,:])
        print(f"\n  -- {nombre}: {len(fr)} saltos --")
        if len(fr):
            orden=np.argsort(ad[fr,jt])[::-1]
            print(f"     {'cuadro':>7} {'t (s)':>7} {'articulacion':30s} {'salto rad':>10s} {'grados':>8s} "
                  f"{'rad/s':>8s} {'% v_lim':>8s}")
            for k in orden:
                f_,j_=int(fr[k]),int(jt[k])
                print(f"     {f_:7d} {f_/fps:7.2f} {joints[j_]:30s} {ad[f_,j_]:10.4f} "
                      f"{np.degrees(ad[f_,j_]):8.1f} {vel[f_,j_]:8.1f} {100*frac[f_,j_]:7.0f}%")
        key="empirico_0.5rad" if umbral is not None else "cota_formal_actuador"
        res["referencias"].setdefault(etiqueta,{})[key]={
            "n_saltos":int(len(fr)),
            "lista":[{"cuadro":int(fr[k]),"t_s":round(float(fr[k]/fps),2),
                      "articulacion":joints[int(jt[k])],
                      "salto_rad":round(float(ad[fr[k],jt[k]]),4),
                      "salto_deg":round(float(np.degrees(ad[fr[k],jt[k]])),1),
                      "vel_rad_s":round(float(vel[fr[k],jt[k]]),1),
                      "pct_v_lim":round(float(100*frac[fr[k],jt[k]]),0)}
                     for k in np.argsort(ad[fr,jt])[::-1]]}
    res["referencias"][etiqueta]["max_salto_rad"]=round(float(ad.max()),4)
    res["referencias"][etiqueta]["max_salto_articulacion"]=joints[int(np.unravel_index(ad.argmax(),ad.shape)[1])]
    res["referencias"][etiqueta]["max_pct_v_lim"]=round(float(100*frac.max()),0)
    res["referencias"][etiqueta]["n_cuadros"]=int(q.shape[0])
    print(f"\n  MAXIMO: {ad.max():.4f} rad ({np.degrees(ad.max()):.1f} deg) en "
          f"{joints[int(np.unravel_index(ad.argmax(),ad.shape)[1])]}, "
          f"{ad.max()*fps:.1f} rad/s = {100*frac.max():.0f}% del limite de ese actuador")

json.dump(res,open(os.path.join(PAPER,"analysis","outputs","discontinuidades.json"),"w",encoding="utf-8"),
          indent=2,ensure_ascii=False)
print("\nescrito analysis/outputs/discontinuidades.json")
