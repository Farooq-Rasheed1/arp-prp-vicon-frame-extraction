# ARP/PRP — Vicon-visible frame extraction

Synchronize Vicon-derived poses with OAK-D depth-image timestamps, project the other robot into each camera, and extract every geometrically visible frame with a projected bounding box. Uses Python and ROS 2 SQLite bag files; a running ROS installation is not required.

## Quick start

Python 3.11 or newer is recommended. From this repository directory:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python run_notebook.py
```

This executes the sample workflow, checks projection/synchronization/annotation behavior, and writes an executed notebook under `outputs/`. Full bag extraction is disabled by default. The included tables contain **594 TB1→TB2** and **1,511 TB2→TB1** visible timestamps, **2,105 total**.

To edit interactively:

```bash
jupyter lab notebooks/01_vicon_visible_frame_extraction.ipynb
```

For Google Colab, upload the complete repository directory to `MyDrive/arp-prp-vicon-frame-extraction`, open the notebook in Colab, and run all cells. The setup cell mounts Drive and installs dependencies. Opening only the notebook from GitHub does not download its supporting files; the complete repository must also be available in the runtime. Change the setup path if the folder is named differently.

## Full bag extraction

Place these original SQLite bag files in the ignored `bags/` directory, or edit their paths in the notebook:

| File | Required topics |
| --- | --- |
| `fusion_bag_0.db3` | `/robot_a/ground_truth`, `/robot_b/ground_truth` (`nav_msgs/Odometry`, Vicon-derived) |
| `tb1_exp_2026_04_24-12_13_33_0.db3` | `/tb1/oakd/stereo/image_raw/compressedDepth` |
| `tb2_exp_2026_04_24-12_13_28_0.db3` | `/tb2/oakd/stereo/image_raw/compressedDepth` |

Set `RUN_FULL_EXTRACTION = True` and run the notebook. It writes visibility tables, raw PNGs, annotated previews, and `extracted_frames.csv` under `outputs/visible_frames/` for both observers. The raw bags are external inputs and are not included in Git.

The command-line tools also work independently:

```bash
python src/camera_visibility_scan.py \
  --fusion-db bags/fusion_bag_0.db3 \
  --tb1-db bags/tb1_exp_2026_04_24-12_13_33_0.db3 \
  --tb2-db bags/tb2_exp_2026_04_24-12_13_28_0.db3 \
  --output-dir outputs/visibility_csv

python src/extract_visibility_frames.py \
  --db bags/tb1_exp_2026_04_24-12_13_33_0.db3 \
  --topic /tb1/oakd/stereo/image_raw/compressedDepth \
  --csv outputs/visibility_csv/tb1_camera_sees_tb2.csv \
  --label tb1_sees_tb2 --output-dir outputs/tb1_sees_tb2 \
  --center-v-fraction 0.5131673177083333 --all
```

For TB2, use its bag/topic and visibility table, with `--center-v-fraction 0.5040386623806424`. The notebook already runs both jobs.

## Geometry and interpretation

- Match pose **header timestamps** to image **bag-record timestamps**, with at most 20 ms error for each robot.
- Default range limit: 6 m; robot width/height: 0.36/0.35 m.
- Camera frame uses forward `x`, left `y`; horizontal projection is `u = cx - fx*y/x`.
- Intrinsics are scaled from 1280×720 to the decoded image dimensions, normally 640×400.
- Box size is range-scaled. Vertical center is assumed to be at the camera principal point, **not computed from the target's 3D height**. Default planar camera offsets are zero.

These are geometric candidate-visibility labels and approximate projected boxes. They do not test occlusion or guarantee tight object boundaries. Setting the vertical center to `cy` does not by itself establish accurate target localization. For calibrated 3D boxes, camera-to-Vicon extrinsics and target geometry are required.

Depth visualization uses a fixed 0.35–6 m Turbo scale (warm near, cool far; invalid depth black). Raw depth values are unchanged. Corner coordinates in `extracted_frames.csv` are clipped to the image and expressed in pixels.

## Files and provenance

- `notebooks/`: edited explanatory notebook, adapted to this repository's paths.
- `src/camera_visibility_scan.py`: original visibility equations and CDR/SQLite readers; input bag paths are explicit command-line arguments.
- `src/extract_visibility_frames.py`: extraction and updated metric-depth annotation.
- `snippets/phase1_depth/`: 12 raw/annotated examples, a sample manifest, and both complete archived visibility tables.
- `run_notebook.py`: reproducible execution and smoke checks; generated output is ignored by Git.

Data originates from the ARP/PRP recording of 24 April 2026. The complete tables were copied from `camera_visibility_results`; sample images came from `vicon_gt_frames_tb1_sees_tb2_all` and `vicon_gt_frames_tb2_sees_tb1_all`, with updated previews from the notebook kit. The `visible == 1` tables are archived references, not newly inferred from these 12 samples. The original report and full bags are separate project artifacts.

For another recording, update bag/topic names, intrinsics, camera offsets, clock alignment, and robot dimensions before interpreting visibility or box coordinates.
