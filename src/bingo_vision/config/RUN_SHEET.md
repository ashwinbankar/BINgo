# BINgo run sheet (OBS split feed, --cam 1 = OBS Virtual Camera)

Put every file of this folder in ONE folder, and run from there. Your YOLO weights stay at
yolov8_model/weights/best.pt (or pass --model). Install once:  pip install ultralytics matplotlib scipy

Rebuild the setup first (final phone positions, focus 0.74, exposure locked, tape marks on the floor).

| # | Command | Pass looks like | Note down |
|---|---------|-----------------|-----------|
| 1 | python3 check_split_camera.py --cam 1 | both halves green when static; safe zone known | any red/flicker |
| 2 | python3 capture_split_charuco.py --cam 1 --out calib_v2 | ~60-80 pairs, all "need:" items ticked | pair count |cl
| 3 | python3 calibrate_split_charuco.py --dir calib_v2 --out stereo_charuco_calibration_split.npz | held-out error < ~1 px | the RESULT block |
| 4 | python3 define_world_frame_split.py --cam 1 | phone disagreement < 30 mm / 3 deg; lens heights match tape within 3-5 cm | disagreement, heights vs tape |
| 5 | python3 check_known_points_split.py --cam 1 --points "x,y; x,y; ..." (6-10 tape-measured points, cm) | RMS <= 5 cm, no SCALE/ROTATION/TILT/SIGN note | the RESULT block + notes |
| 6a | python3 check_timing_split.py flash --cam 1 | offset constant (not "drifting") | offset ms, std, verdict, duplicate-frame % |
| 6b | python3 check_timing_split.py speed --cam 1 --device mps | >= 25 FPS | processed FPS, latency |
| 7 | python3 live_tracker_split.py --cam 1 --device mps  (throw ~25 times; mark a few landing spots with tape) | green circle follows the ball in both views; top view shows the arc | throws detected, anything odd |
| 8 | python3 evaluate_throws.py track_log.csv --sweep-drag   (optional: --landings landings.csv with columns throw_id,x_cm,y_cm) | median landing error <= 10-15 cm at 0.3 s lead | the RESULT block, drag k |

If a step fails its "pass looks like", stop there and report that step: later steps depend on it.
Do not move the phones between steps 2 and 8. If you move them, redo 2-4 (or use
calibrate_split_charuco.py --reuse-intrinsics OLD.npz with ~25 new pairs if focus did not change).
Re-run step 7 with  --drag-k <value>  from step 8 afterwards.
