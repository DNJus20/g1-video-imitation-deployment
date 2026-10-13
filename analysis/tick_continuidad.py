#!/usr/bin/env python3
"""Prueba decisiva: el reloj del robot (tick, ms) es continuo aunque nuestra
captura tenga huecos? Un tick ausente = el robot no publico ese milisegundo."""
import json, os, sys, datetime as dt
import numpy as np
sys.stdout.reconfigure(encoding="utf-8", errors="replace")

P = os.environ.get("PAPER_DIR", os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))  # carpeta con data/ (disposicion original del proyecto)
RUNS = [("run00_prueba","evidencia_no_protocolo"),
        ("run01_baileaxe_arnes","data"),
        ("run02_baileaxe_sinarnes","evidencia_no_protocolo"),
        ("run03_baileaxe_sinarnes","data"),
        ("run04_baileaxe_sinarnes","data"),
        ("run05_baileaxe_sinarnes","data"),
        ("run06_baileaxe_sinarnes","data"),
        ("run07_baileaxe_sinarnes","data")]
BTN_R1,BTN_L1,BTN_B = 1<<0,1<<1,1<<9

res={}
hdr = f"{'corrida':26s} {'ticks span':>11s} {'unicos':>8s} {'faltan':>7s} {'%':>7s} {'max racha':>9s} {'en baile':>9s}"
print(hdr); print("-"*len(hdr))
for name,sub in RUNS:
    z=np.load(os.path.join(P,sub,name+".npz"))
    tick=z["tick"].astype(np.int64); t=z["t"].astype(np.float64); t-=t[0]
    lo,hi=int(tick.min()),int(tick.max())
    span=hi-lo+1
    uniq=np.unique(tick)
    present=np.zeros(span,bool); present[uniq-lo]=True
    missing=np.flatnonzero(~present)          # milisegundos que el robot nunca entrego
    # rachas de ms ausentes consecutivos
    if len(missing):
        brk=np.flatnonzero(np.diff(missing)>1)
        starts=np.r_[missing[0],missing[brk+1]]
        ends=np.r_[missing[brk],missing[-1]]
        runs_len=ends-starts+1
        maxr=int(runs_len.max())
        top=np.argsort(runs_len)[::-1][:5]
        peores=[{"tick_ms_desde_inicio":int(starts[i]),"ms_ausentes":int(runs_len[i])} for i in sorted(top)]
    else:
        maxr=0; peores=[]; starts=np.array([],int); runs_len=np.array([],int)

    # cuantos ms ausentes caen DENTRO de la ventana del baile
    m=((z["buttons"]&BTN_R1)!=0)&((z["buttons"]&BTN_B)!=0)&((z["buttons"]&BTN_L1)==0)
    edges=np.flatnonzero(m & ~np.r_[False,m[:-1]])
    en_baile="-"
    if len(edges):
        i0=int(edges[0]); tk0=int(tick[i0])-lo; tk1=tk0+int(56.98*1000)
        en_baile=int(np.sum((missing>=tk0)&(missing<=tk1)))
    res[name]={"tick_span_ms":span,"ticks_unicos":int(len(uniq)),
               "ms_ausentes":int(len(missing)),
               "pct_ausentes":round(100*len(missing)/span,3),
               "max_racha_ms":maxr,"ms_ausentes_en_ventana_baile":en_baile,
               "peores_rachas":peores}
    print(f"{name:26s} {span:11d} {len(uniq):8d} {len(missing):7d} {100*len(missing)/span:6.2f}% {maxr:8d}  {str(en_baile):>9s}")
print()
for k,v in res.items():
    if v["peores_rachas"]:
        print(k, "peores rachas de ms ausentes:", v["peores_rachas"])
json.dump(res,open(os.path.join(P,"analysis","outputs","tick_continuidad.json"),"w",encoding="utf-8"),indent=2,ensure_ascii=False)
print("\nescrito analysis/outputs/tick_continuidad.json")
