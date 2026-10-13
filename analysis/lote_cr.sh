#!/usr/bin/env bash
# LOTE CR (camera-ready HRFEST 2026) — todo virtual, sin robot, SIN entrenar.
# Autorizado por el usuario 2026-09-22: reusar bailetinkus (ya entrenado) como segundo movimiento;
# las lineas base se toman de corridas existentes. Cada paso falla aislado (no detiene el lote).
WS=$HOME/g1_ws; CR=$WS/cr; OUT=$CR/out; LOGS=$CR/logs
M=/workspace/unitree_rl_mjlab/src/assets/motions/g1
L=/workspace/unitree_rl_mjlab/logs/rsl_rl/g1_tracking
MJ="docker run --rm --gpus all -v $WS:/workspace -e PYTHONPATH=/workspace/unitree_rl_mjlab -e PYTHONUNBUFFERED=1 -w /workspace/unitree_rl_mjlab mjlab:latest"
SIM="docker run --rm -v $WS:/workspace g1-mujoco:latest"
ST=$LOGS/estado.txt
paso() { echo "$(date -u +%FT%TZ) $1 $2" >> $ST; }

paso INICIO lote
# --- G3: evaluacion en el simulador de entrenamiento (misma metrica que sim_eval_baileaxe.json)
ev() { # tag run motion [iter]
  paso G3 "eval $1"
  RUN=$L/$2 MOTION=$M/$3 CKPT_ITER=$4 TAG=$1 \
  $MJ bash -c "RUN=$L/$2 MOTION=$M/$3 CKPT_ITER=$4 TAG=$1 python /workspace/cr/eval_sim_cr.py" > $LOGS/$1.log 2>&1 \
    && paso G3 "ok $1" || paso G3 "FALLO $1 (ver logs/$1.log)"
}
ev sim_eval_tinkus_final    2026-09-08_17-04-34 bailetinkus30.npz ""
ev sim_eval_tinkus_3000     2026-09-08_17-04-34 bailetinkus30.npz 3000
ev sim_eval_baileaxe_3000   2026-07-21_19-47-02 baileaxe57.npz    3000
ev sim_eval_bailemix_3000   2026-07-22_17-19-12 bailemix57.npz    3000
ev sim_eval_baileaxe_final  2026-07-21_19-47-02 baileaxe57.npz    ""

# --- G2/G5: metricas de entrenamiento de las corridas existentes
paso G2 tb
$MJ python /workspace/cr/tb_extract.py > $LOGS/tb_extract.log 2>&1 && paso G2 "ok tb" || paso G2 "FALLO tb"

# --- G1/G4: continuidad y dinamismo de las referencias
paso G1 motion_stats
$SIM python /workspace/cr/motion_stats.py > $LOGS/motion_stats.log 2>&1 && paso G1 "ok motion_stats" || paso G1 "FALLO motion_stats"

# --- G4: sim-to-sim con el binario de despliegue (3 repeticiones por baile)
for i in 1 2 3; do
  for spec in "X bailetinkus30.npz Mimic_Tinkus tinkus" "B baileaxe57.npz Mimic_Baileaxe baileaxe"; do
    set -- $spec; TAG=${4}_r$i
    paso G4 "sim2sim $TAG"
    timeout 200 bash $CR/sim2sim_run.sh $1 unitree_rl_mjlab/src/assets/motions/g1/$2 $TAG >> $LOGS/sim2sim_runs.log 2>&1
    $SIM python /workspace/cr/sim2sim_eval.py /workspace/cr/out/sim2sim_$TAG.npz /workspace/cr/logs/g1ctrl_$TAG.log $3 /workspace/cr/out/sim2sim_$TAG.json >> $LOGS/sim2sim_eval.log 2>&1 \
      && paso G4 "ok $TAG" || paso G4 "FALLO $TAG"
  done
done

# --- G5: configuracion, paso de control, BeyondMimic
paso G5 config
bash $CR/g5_config.sh >> $LOGS/g5.log 2>&1 && paso G5 "ok config" || paso G5 "FALLO config"

paso FIN lote
