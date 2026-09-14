# Local validation

Validated on 14 September 2026 on Linux, in a fresh virtual environment installed from `requirements.txt`.

| Component | Tested version |
| --- | --- |
| Python | 3.14.4 |
| NumPy | 2.5.3 |
| pandas | 3.0.5 |
| matplotlib | 3.11.2 |
| Pillow | 12.3.0 |
| opencv-python-headless | 5.0.0.93 |
| nbformat / nbclient | 5.11.1 / 0.11.0 |
| ipykernel / JupyterLab | 7.3.0 / 4.6.3 |

Command: `python run_notebook.py`

Passed:

- Notebook schema validation and sample cells executed from the repository root.
- Archived table counts: 594 and 1,511 visible frames.
- Rotated observer-frame projection and expected projected width.
- Synchronization accepted at 20 ms and rejected immediately beyond it.
- Image-edge clipping of projected box coordinates.
- All 12 sample image previews rendered.

The full bag-extraction branch is disabled for this smoke run. This check does not claim that geometric visibility resolves occlusion or that the projected boxes are pixel-tight annotations. It also does not rerun the original full ROS bag experiment.

Source notebooks are stored without outputs for readable Git changes. Running the command produces a separate executed copy in `outputs/` without modifying the source notebook.
