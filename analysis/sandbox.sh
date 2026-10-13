#!/usr/bin/env bash
# Crea ~/g1_ws/cr/sim2sim/cfg: copia del config de deploy + estado Mimic_Tinkus (id 7, RB+X).
# No toca el arbol original de deploy.
set -e
WS=$HOME/g1_ws; SRC=$WS/unitree_rl_mjlab/deploy/robots/g1/config
DST=$WS/cr/sim2sim/cfg
RUN=$WS/unitree_rl_mjlab/logs/rsl_rl/g1_tracking/2026-09-08_17-04-34
rm -rf $DST; mkdir -p $DST; cp -r $SRC $DST/config
T=$DST/config/policy/mimic/tinkus; mkdir -p $T/exported $T/params
cp $RUN/policy.onnx $T/exported/policy.onnx
cp $SRC/policy/mimic/baileaxe/params/deploy.yaml $T/params/deploy.yaml
cp $WS/unitree_rl_mjlab/src/assets/motions/g1/bailetinkus30.npz $T/params/bailetinkus30.npz
python3 - "$DST/config/config.yaml" <<'PY'
import sys
p=sys.argv[1]; s=open(p).read()
s=s.replace("    Mimic_Baileaxe:\n      id: 6\n      type: Mimic\n",
            "    Mimic_Baileaxe:\n      id: 6\n      type: Mimic\n\n    Mimic_Tinkus:\n      id: 7\n      type: Mimic\n",1)
s=s.replace("      Mimic_Baileaxe: RB + B.on_pressed\n",
            "      Mimic_Baileaxe: RB + B.on_pressed\n      Mimic_Tinkus: RB + X.on_pressed\n",1)
blk=s[s.index("\n  Mimic_Baileaxe:\n"):]
blk=blk[:blk.index("time_end")]; blk=blk+s[s.index("time_end",s.index("\n  Mimic_Baileaxe:\n")):].split("\n")[0]+"\n"
nb=blk.replace("Mimic_Baileaxe","Mimic_Tinkus").replace("mimic/baileaxe/params/baileaxe57.npz","mimic/tinkus/params/bailetinkus30.npz").replace("mimic/baileaxe/","mimic/tinkus/")
s=s.rstrip("\n")+"\n"+nb
open(p,"w").write(s)
assert s.count("Mimic_Tinkus")==3, s.count("Mimic_Tinkus")
PY
tail -12 $DST/config/config.yaml
