import os,sys,json
import numpy as np
sys.stdout.reconfigure(encoding="utf-8",errors="replace")
SP=os.path.join(os.path.dirname(os.path.abspath(__file__)),"insumos")
PAPER=os.environ.get("PAPER_DIR", os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))  # carpeta con data/ (disposicion original del proyecto)
F=[("baileaxe57 (desplegada, OK)",os.path.join(PAPER,"data","baileaxe57.npz")),
   ("bailemix57 (no convergio)",  os.path.join(SP,"bailemix57.npz"))]
out={}
print(f"{'metrica':48s} {'baileaxe57':>16s} {'bailemix57':>16s}")
print("-"*84)
rows={}
for et,p in F:
    z=np.load(p); q=z["joint_pos"].astype(np.float64); fps=float(z["fps"][0])
    ad=np.abs(np.diff(q,axis=0))
    bp=z["body_pos_w"].astype(np.float64)   # (T,30,3) cuerpo 0 = pelvis
    pelvis_z=bp[:,0,2]
    d={}
    d["cuadros con algun salto >0,5 rad"]=int(np.sum(ad.max(axis=1)>0.5))
    d["pares (cuadro,articulacion) >0,5 rad"]=int(np.sum(ad>0.5))
    d["salto maximo (rad)"]=round(float(ad.max()),4)
    d["saltos >0,5 rad en PIERNAS (0-11)"]=int(np.sum(ad[:,:12]>0.5))
    d["saltos >0,5 rad en BRAZOS (15-28)"]=int(np.sum(ad[:,15:]>0.5))
    d["salto maximo en PIERNAS (rad)"]=round(float(ad[:,:12].max()),4)
    d["altura pelvis min (m)"]=round(float(pelvis_z.min()),4)
    d["altura pelvis max (m)"]=round(float(pelvis_z.max()),4)
    d["recorrido vertical pelvis (m)"]=round(float(pelvis_z.max()-pelvis_z.min()),4)
    d["desv. tipica altura pelvis (m)"]=round(float(pelvis_z.std()),4)
    d["|dz pelvis| max por cuadro (m)"]=round(float(np.abs(np.diff(pelvis_z)).max()),4)
    # velocidad vertical de pelvis
    d["vel. vertical pelvis max (m/s)"]=round(float(np.abs(np.diff(pelvis_z)).max()*fps),3)
    rows[et]=d
for k in rows[F[0][0]]:
    a=rows[F[0][0]][k]; b=rows[F[1][0]][k]
    print(f"{k:48s} {a:>16} {b:>16}")
json.dump(rows,open(os.path.join(PAPER,"analysis","outputs","comparacion_referencias.json"),"w",encoding="utf-8"),indent=2,ensure_ascii=False)
print("\nescrito analysis/outputs/comparacion_referencias.json")
