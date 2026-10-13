#!/usr/bin/env bash
# Cola del camera-ready (2026-10-05): segunda intervencion, altura de la raiz corregida al suelo.
# Pre-registro en BRIDGE_code.md [2026-10-05 12:00]. bailemix57g = intervencion, baileaxe57g = control.
# Misma receta que la politica desplegada (run_train_axe.sh): tarea No-State-Estimation,
# config del repo @1425b15, 8192 envs, 15000 iteraciones. Un fallo NO detiene la cola.
export PYTHONUNBUFFERED=1 WANDB_MODE=offline WANDB_SILENT=true PYTHONPATH=/workspace/unitree_rl_mjlab
mkdir -p /workspace/cr/logs
cd /workspace/unitree_rl_mjlab
for n in bailemix57g baileaxe57g; do
  L=/workspace/cr/logs/train_${n}_cr.log
  echo "=== TRAIN $n task=Unitree-G1-Tracking-No-State-Estimation envs=8192 iter=15000 START $(date -u +%F_%H:%M:%S) ===" | tee -a $L /workspace/cr/logs/cola_g.log
  python scripts/train.py Unitree-G1-Tracking-No-State-Estimation \
    --motion_file=src/assets/motions/g1/${n}.npz \
    --env.scene.num-envs=8192 --agent.max-iterations=15000 >> $L 2>&1
  echo "=== TRAIN_DONE $n EXIT=$? $(date -u +%F_%H:%M:%S) ===" | tee -a $L /workspace/cr/logs/cola_g.log
done
echo "=== COLA_COMPLETA $(date -u +%F_%H:%M:%S) ===" >> /workspace/cr/logs/cola_g.log
