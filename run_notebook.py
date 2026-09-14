"""Execute the sample notebook and check geometry; save results under outputs/."""
from pathlib import Path

import nbformat
from nbclient import NotebookClient

ROOT = Path(__file__).resolve().parent
NOTEBOOK = ROOT / 'notebooks/01_vicon_visible_frame_extraction.ipynb'

CHECKS = '''
import math
from camera_visibility_scan import CameraModel, CameraExtrinsics2D, Pose2D, scan_camera
from extract_visibility_frames import bbox_coordinates

camera = CameraModel(640, 400, 500., 500., 320., 200.)
stamp = 1_000_000_000
observer = Pose2D(stamp, 0., 0., math.pi / 2)
target = Pose2D(stamp, -1., 2., 0.)
projected = scan_camera([stamp], [observer], [target], camera,
                        CameraExtrinsics2D(), .36, .35, 6., 20.)[0]
assert projected.visible and abs(projected.u_px - 70.) < 1e-8
assert abs(projected.bbox_w_px - 90.) < 1e-8
assert len(scan_camera([stamp + 20_000_000], [observer], [target], camera,
                       CameraExtrinsics2D(), .36, .35, 6., 20.)) == 1
assert not scan_camera([stamp + 20_000_001], [observer], [target], camera,
                       CameraExtrinsics2D(), .36, .35, 6., 20.)
box = bbox_coordinates(Image.new('RGB', (640, 400)),
                       {'u_px': 10., 'bbox_w_px': 40., 'bbox_h_px': 100.}, .5)
assert box == (0., 150., 30., 250.)
assert summary.visible_frames.tolist() == [594, 1511]
print('PASS: sample tables, rotated projection, 20 ms boundary, clipped box corners')
'''


if __name__ == '__main__':
    notebook = nbformat.read(NOTEBOOK, as_version=4)
    nbformat.validate(notebook)
    notebook.cells.append(nbformat.v4.new_code_cell(CHECKS))
    NotebookClient(notebook, timeout=600, kernel_name='python3',
                   resources={'metadata': {'path': str(ROOT)}}).execute()
    output = ROOT / 'outputs' / NOTEBOOK.name
    output.parent.mkdir(exist_ok=True)
    nbformat.write(notebook, output)
    for cell in notebook.cells:
        for result in cell.get('outputs', []):
            if result.output_type == 'stream':
                print(result.text, end='')
    print(f'Executed notebook: {output}')
