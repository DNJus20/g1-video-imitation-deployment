# Arranque en frio x5 — 2026-10-12 (hora Jetson 2026-10-13 CST), robot 29 DOF colgado, PC1 nueva, deploy restaurado
Fuentes por ciclo: /proc/uptime del Jetson, g1launcher.log, g1ctrl.log (timestamps), lab_1012/frio/cicloN_pubs.log (lowcmd_pubs: publicadores emparejados/activos en rt/lowcmd).

## Ciclo 1 — EXITO
- Boot Jetson (uptime -s): 06:20:48. WiFi de vuelta ~28 s. Launcher: NRestarts=1 (eth0 aun no lista), luego "esperando rt/lowstate".
- Fabrica empieza a publicar rt/lowcmd: up=70 s (1000 msg/s, 1 activo, 2 emparejados).
- Launcher "listo": up=76 s  -> (d) tiempo hasta listo automatico = 76 s desde el arranque.
- L2+R2 (debug): rt/lowcmd pasa a 0 activos (silencio) -> (b) fabrica desactivada. Silencio hasta up=115.6 s.
- L1+R1+A: launcher "COMBO LANZAR (debug mode OK) -> start g1ctrl". g1_ctrl: Waiting 06:22:44.526, Connected .726, 3 politicas cargadas, "FSM: Start Passive" 06:22:45.015 (= up ~117 s; 0.5 s desde el arranque del proceso).
- (c) rt/lowcmd tras lanzar: 1 publicador activo (handle nuevo 1286...), 830-880 msg/s; el escritor de fabrica sigue emparejado pero callado.
- Nota: un segundo L1+R1+A fue "ignorado" por la compuerta (lowcmd ya en uso por g1_ctrl): la compuerta evita doble arranque; el texto del mensaje dice "FABRICA" aunque el publicador era g1_ctrl.

## Ciclo 2 — EXITO
- Cierre ciclo 1 confirmado en g1ctrl.log: Passive->FixStand 06:24:52, FixStand->Passive 06:25:04, luego stop por launcher.
- Boot Jetson: 06:26:43. WiFi ~22 s. Launcher NRestarts=1, "listo" up=57.8 s. Fabrica publica rt/lowcmd up=70.7 s (1000 msg/s, 1 activo, 2 emparejados).
- L2+R2: silencio en rt/lowcmd (0 activos) hasta up=155 s. L1+R1+A una vez: "COMBO LANZAR (debug mode OK)".
- g1_ctrl: Waiting 06:29:17.736, Connected .936, Start Passive 06:29:18.219 (0.5 s). (c) 1 activo ~830 msg/s, handle nuevo; fabrica emparejada callada.

## Ciclo 3 — EXITO
- Cierre ciclo 2 confirmado: Passive->FixStand 06:30:06, FixStand->Passive 06:30:11.
- Boot Jetson: 06:31:19. WiFi ~32 s. Launcher NRestarts=1, "listo" up=59.0 s. Fabrica publica rt/lowcmd up=70.9 s.
- L2+R2: silencio hasta up=104 s. L1+R1+A una vez: "debug mode OK". g1_ctrl: Waiting 06:33:02.347, Connected .547, Start Passive .836 (0.5 s). (c) 1 activo ~880 msg/s, handle nuevo.
- Cierre ciclo 3: FixStand ejecutado (confirmado por el operador); NO quedo en g1ctrl.log porque el apagado corto la escritura (linea con bytes nulos).

## Ciclo 4 — EXITO
- Boot Jetson: 06:36:53. Launcher NRestarts=1, "listo" up=59.3 s. Fabrica publica rt/lowcmd up=70.2 s.
- L2+R2: silencio hasta up=177.7 s. L1+R1+A una vez: "debug mode OK". g1_ctrl: Waiting 06:39:50.601, Connected .802, Start Passive 06:39:51.094 (0.5 s). (c) 1 activo ~820 msg/s, handle nuevo.
- Cierre ciclo 4: Passive->FixStand 06:40:49.607 confirmado en log (sync forzado); el L2+B/stop posterior no quedo escrito (apagado).

## Ciclo 5 — EXITO
- Boot Jetson: 06:52:39. WiFi ~30 s. Launcher NRestarts=1, "listo" up=58.2 s. Fabrica publica rt/lowcmd up=71.1 s.
- L2+R2: silencio hasta up=94.2 s. L1+R1+A una vez: "debug mode OK". g1_ctrl: Waiting 06:54:13.236, Connected .437, Start Passive .719 (0.5 s). (c) 1 activo ~880 msg/s, handle nuevo.
