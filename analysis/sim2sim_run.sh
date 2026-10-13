#!/usr/bin/env bash
# Uso: sim2sim_run.sh <boton> <motion.npz relativo a /workspace> <tag>
# Lanza g1_ctrl (g1-deploy) sobre el sandbox y el puente MuJoCo (g1-mujoco), ambos en loopback.
B=$1; MOT=$2; TAG=$3
WS=$HOME/g1_ws; OUT=$WS/cr/out; LOGS=$WS/cr/logs
COMMON="--rm --network host -v $WS:/workspace -e CYCLONEDDS_URI=file:///workspace/cyclonedds_sim.xml"
docker rm -f s2s_ctrl s2s_sim >/dev/null 2>&1
# g1_ctrl lee el config de una ruta absoluta compilada: se monta el sandbox ENCIMA, solo dentro del contenedor
docker run -d --name s2s_ctrl $COMMON -v $WS/cr/sim2sim/cfg/config:/workspace/unitree_rl_mjlab/deploy/robots/g1/config:ro \
  -w /workspace/cr/sim2sim/cfg g1-deploy:latest \
  bash -c "/workspace/unitree_rl_mjlab/deploy/robots/g1/build/g1_ctrl --network=lo > /workspace/cr/logs/g1ctrl_$TAG.log 2>&1" >/dev/null
sleep 3
docker run --name s2s_sim $COMMON g1-mujoco:latest \
  python /workspace/cr/sim2sim_bridge.py $B /workspace/$MOT /workspace/cr/out/sim2sim_$TAG.npz > $LOGS/bridge_$TAG.log 2>&1
RC=$?
docker rm -f s2s_ctrl s2s_sim >/dev/null 2>&1
echo "sim2sim $TAG exit=$RC"
