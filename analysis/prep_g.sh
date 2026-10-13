#!/usr/bin/env bash
# Genera las referencias con altura corregida al suelo (sufijo g) y las verifica. Corre en la PC GPU.
# bailemix57g = bailemix57 SIN FILTRAR + correccion de altura (intervencion).
# baileaxe57g = baileaxe57 SIN FILTRAR + la misma correccion (control).
set -e
WS=$HOME/g1_ws; M=unitree_rl_mjlab/src/assets/motions/g1
for n in bailemix57 baileaxe57; do
  docker run --rm -u $(id -u):$(id -g) -v $WS:/d gmr:latest python3 /d/cr/suelo_csv.py \
    /d/motion/${n}_gmr_g1.csv /d/$M/$n.npz /d/motion/${n}g_gmr_g1.csv 2>&1 | grep '^{' | tee $WS/cr/out/suelo_${n}g.json
done
docker run --rm --gpus all -v $WS:/workspace -e PYTHONPATH=/workspace/unitree_rl_mjlab -w /workspace/unitree_rl_mjlab mjlab:latest bash -c '
for n in bailemix57g baileaxe57g; do
  python scripts/csv_to_npz.py --input-file /workspace/motion/${n}_gmr_g1.csv --output-name ${n}.npz --input-fps 30 --output-fps 50 --robot g1 2>&1 | tail -2
done'
ls -la --time-style=long-iso $WS/$M/*57g.npz
docker run --rm -u $(id -u):$(id -g) -v $WS:/d gmr:latest python3 /d/cr/verifica_g.py 2>&1 | grep -v -iE 'cuda|nvidia|license|container|^$'
