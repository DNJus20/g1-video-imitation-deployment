#!/usr/bin/env bash
# Uso: sim2sim_tilt_run.sh <eje pitch+|pitch-|roll+|roll-> <tag>
EJE=$1; TAG=$2; EXTRA="${3:-} ${4:-} ${5:-}"
WS=$HOME/g1_ws; OUT=$WS/cr/out; LOGS=$WS/cr/logs
COMMON="--rm --network host -v $WS:/workspace -e CYCLONEDDS_URI=file:///workspace/cyclonedds_sim.xml"
docker rm -f s2s_ctrl s2s_sim >/dev/null 2>&1
docker run -d --name s2s_ctrl $COMMON -v $WS/cr/sim2sim/cfg/config:/workspace/unitree_rl_mjlab/deploy/robots/g1/config:ro \
  -w /workspace/cr/sim2sim/cfg g1-deploy:latest \
  bash -c "/workspace/unitree_rl_mjlab/deploy/robots/g1/build/g1_ctrl --network=lo > /workspace/cr/logs/g1ctrl_tilt_$TAG.log 2>&1" >/dev/null
sleep 3
docker run --name s2s_sim $COMMON g1-mujoco:latest \
  python /workspace/cr/sim2sim_tilt.py $EJE /workspace/cr/out/sim2sim_tilt_$TAG.npz $EXTRA > $LOGS/bridge_tilt_$TAG.log 2>&1
RC=$?
docker rm -f s2s_ctrl s2s_sim >/dev/null 2>&1
echo "tilt $TAG exit=$RC"; tail -2 $LOGS/bridge_tilt_$TAG.log; grep -a FSM $LOGS/g1ctrl_tilt_$TAG.log | tail -3
