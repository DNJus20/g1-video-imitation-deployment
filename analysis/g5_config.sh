#!/usr/bin/env bash
# G5: paso de control, diferencias contra la configuracion por defecto, primer intento, origen BeyondMimic.
# Salida: ~/g1_ws/cr/out/g5_config.txt (texto plano, evidencia cruda para el informe)
WS=$HOME/g1_ws; R=$WS/unitree_rl_mjlab; L=$R/logs/rsl_rl/g1_tracking
{
echo "== 1. Repositorio unitree_rl_mjlab"
cd $R; git remote get-url origin; git log -1 --format="HEAD %H  %ci  %s"
echo "-- archivos de src/ modificados respecto a HEAD (vacio = config por defecto):"
git status --short src/ | grep -v "__pycache__\|\.npz$"
git diff --stat src/
echo "-- respaldos de config que existen en disco:"
ls -la src/tasks/tracking/*.bak* src/tasks/tracking/*sim2real* 2>/dev/null

echo; echo "== 2. Paso de control y simulacion (env.yaml de la politica desplegada 2026-07-21_19-47-02)"
grep -n -E "^\s*(decimation|timestep|episode_length_s|num_envs):" $L/2026-07-21_19-47-02/params/env.yaml | head
echo "-- deploy.yaml (binario): step_dt"
grep -n "step_dt" $R/deploy/robots/g1/config/policy/mimic/baileaxe/params/deploy.yaml
echo "-- lazo de bajo nivel del binario (CtrlFSM / dt):"
grep -rn -E "dt *= *0\.0|RecurrentThread|1000" $R/deploy/include/FSM/CtrlFSM.h 2>/dev/null | head -5

echo; echo "== 3. Configuracion: politica desplegada vs segundo movimiento vs repo"
echo "-- diff env.yaml 07-21 vs 09-08 (sin motion_file):"
diff <(grep -v motion_file $L/2026-07-21_19-47-02/params/env.yaml) <(grep -v motion_file $L/2026-09-08_17-04-34/params/env.yaml) && echo "(identicos)"
echo "-- diff agent.yaml 07-21 vs 09-08:"
diff $L/2026-07-21_19-47-02/params/agent.yaml $L/2026-09-08_17-04-34/params/agent.yaml && echo "(identicos)"
echo "-- max_iterations / num_envs en agent.yaml/env.yaml de la desplegada:"
grep -n -E "max_iterations|num_steps_per_env|save_interval|seed" $L/2026-07-21_19-47-02/params/agent.yaml | head
grep -n -E "^\s*num_envs" $L/2026-07-21_19-47-02/params/env.yaml | head -2
echo "-- valores por defecto en el codigo del repo:"
grep -rn -E "max_iterations|num_envs *=|num_envs:" src/tasks/tracking/ 2>/dev/null | grep -v pycache | head
echo "-- comando de entrenamiento usado (scripts en ~/g1_ws):"
grep -h -A3 "scripts/train.py" $WS/run_train_axe*.sh $WS/run_train_tinkus.sh 2>/dev/null | head -12

echo; echo "== 4. Primer intento sin filtrar / no convergido (corridas en logs)"
for d in $L/*/; do n=$(basename $d); m=$(grep -m1 -o "motions/g1/[^ ]*" $d/params/env.yaml); k=$(ls $d | grep -c "model_"); last=$(ls $d | grep model_ | sed 's/model_//;s/.pt//' | sort -n | tail -1); echo "$n  $m  checkpoints=$k  ultimo=$last"; done

echo; echo "== 5. Origen de la tarea de tracking (atribucion BeyondMimic)"
grep -rn -i -E "beyondmimic|whole_body_tracking|HybridRobotics|beyond mimic" $R --include=*.py --include=*.md --include=*.toml --include=*.txt --include=*.cfg --include=LICENSE* 2>/dev/null | grep -v "/logs/" | head -20
echo "-- mjlab instalado (paquete del que hereda la tarea):"
docker run --rm mjlab:latest bash -c "pip show mjlab 2>/dev/null | head -3; python - <<'PY'
import mjlab, os
p=os.path.dirname(mjlab.__file__); print('mjlab en', p)
import subprocess
print(subprocess.run(['grep','-rn','-i','-E','beyondmimic|whole_body_tracking|HybridRobotics', p],capture_output=True,text=True).stdout[:3000])
PY
cd \$(python -c 'import mjlab,os;print(os.path.dirname(mjlab.__file__))')/.. 2>/dev/null && git log -1 --format='mjlab HEAD %H %ci' 2>/dev/null; git remote get-url origin 2>/dev/null" 2>&1 | grep -v "^==\|NVIDIA\|CUDA\|^$" | head -40
} > $WS/cr/out/g5_config.txt 2>&1
echo "g5 listo: $(wc -l < $WS/cr/out/g5_config.txt) lineas"
