#!/usr/bin/env bash
# Re-corrida G4 de bailetinkus tras corregir el montaje del config (g1_ctrl usa ruta absoluta).
WS=$HOME/g1_ws; CR=$WS/cr; LOGS=$CR/logs; ST=$LOGS/estado.txt
SIM="docker run --rm -v $WS:/workspace g1-mujoco:latest"
paso() { echo "$(date -u +%FT%TZ) $1 $2" >> $ST; }
paso INICIO g4b
for i in 1 2 3; do TAG=tinkus_r$i
  paso G4 "sim2sim $TAG (config montado)"
  timeout 200 bash $CR/sim2sim_run.sh X unitree_rl_mjlab/src/assets/motions/g1/bailetinkus30.npz $TAG >> $LOGS/sim2sim_runs.log 2>&1
  $SIM python /workspace/cr/sim2sim_eval.py /workspace/cr/out/sim2sim_$TAG.npz /workspace/cr/logs/g1ctrl_$TAG.log Mimic_Tinkus /workspace/cr/out/sim2sim_$TAG.json >> $LOGS/sim2sim_eval.log 2>&1 \
    && paso G4 "ok $TAG" || paso G4 "FALLO $TAG"
done
paso FIN g4b
