#!/usr/bin/env bash
# Espera a que termine la cola de entrenamiento (train_cola.sh) y evalua las 3 politicas sin supervision:
# simulador de entrenamiento (final y 3000 iter), curvas de TensorBoard y banco del binario (3 repeticiones).
# Todo queda en ~/g1_ws/cr/out/int_* y el estado en ~/g1_ws/cr/logs/post_cola.txt. Un fallo no detiene el resto.
WS=$HOME/g1_ws; CR=$WS/cr; OUT=$CR/out; LOGS=$CR/logs; ST=$LOGS/post_cola.txt
L=$WS/unitree_rl_mjlab/logs/rsl_rl/g1_tracking
LC=/workspace/unitree_rl_mjlab/logs/rsl_rl/g1_tracking; MC=/workspace/unitree_rl_mjlab/src/assets/motions/g1
MJ="docker run --rm --gpus all -v $WS:/workspace -e PYTHONPATH=/workspace/unitree_rl_mjlab -e PYTHONUNBUFFERED=1 -w /workspace/unitree_rl_mjlab mjlab:latest"
SIM="docker run --rm -v $WS:/workspace g1-mujoco:latest"
paso() { echo "$(date -u +%FT%TZ) $*" >> $ST; }

paso ESPERANDO cola
until grep -q COLA_COMPLETA $LOGS/cola.log 2>/dev/null; do sleep 120; done
paso COLA_TERMINADA "$(grep -c TRAIN_DONE $LOGS/cola.log) corridas"

RUNS=""
for d in $(ls $L | awk '$0 >= "2026-10-02"'); do
  n=$(grep -m1 -o 'motions/g1/[A-Za-z0-9_]*\.npz' $L/$d/params/env.yaml | sed 's#motions/g1/##; s#\.npz##')
  [ -z "$n" ] && { paso SIN_MOTION $d; continue; }
  RUNS="$RUNS $d"
  paso CORRIDA $d $n "ultimo=$(ls $L/$d | grep -o 'model_[0-9]*' | sed 's/model_//' | sort -n | tail -1)"
  for it in "" 3000; do
    T=int_sim_${n}_${it:-final}
    $MJ bash -c "RUN=$LC/$d MOTION=$MC/$n.npz CKPT_ITER=$it TAG=$T python /workspace/cr/eval_sim_cuerpos.py" > $LOGS/$T.log 2>&1 \
      && paso ok $T || paso FALLO $T
  done
  # banco del binario: se reusa la ranura Mimic_Tinkus (RB+X) del sandbox con la politica y la referencia nuevas
  if [ -f $L/$d/policy.onnx ]; then
    bash $CR/sandbox.sh > $LOGS/int_sandbox_$n.log 2>&1
    T=$CR/sim2sim/cfg/config/policy/mimic/tinkus
    cp $L/$d/policy.onnx $T/exported/policy.onnx && cp $WS/unitree_rl_mjlab/src/assets/motions/g1/$n.npz $T/params/bailetinkus30.npz
    for i in 1 2 3; do
      TAG=int_${n}_r$i
      timeout 240 bash $CR/sim2sim_run.sh X unitree_rl_mjlab/src/assets/motions/g1/$n.npz $TAG >> $LOGS/int_sim2sim_runs.log 2>&1
      $SIM python /workspace/cr/sim2sim_eval.py /workspace/cr/out/sim2sim_$TAG.npz /workspace/cr/logs/g1ctrl_$TAG.log Mimic_Tinkus /workspace/cr/out/sim2sim_$TAG.json >> $LOGS/int_sim2sim_eval.log 2>&1 \
        && paso ok banco $TAG || paso FALLO banco $TAG
    done
  else
    paso SIN_ONNX $d
  fi
done
bash $CR/sandbox.sh > $LOGS/int_sandbox_restaurado.log 2>&1 && paso sandbox restaurado
$MJ python /workspace/cr/tb_curvas.py /workspace/cr/out/int_tb_curvas.json $RUNS > $LOGS/int_tb.log 2>&1 && paso ok tb || paso FALLO tb
paso FIN
