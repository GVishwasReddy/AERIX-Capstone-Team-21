import cv2
import numpy as np
from drone_stack.gcs.hailo_infer import _Model
model = _Model('models/visdronebest.hef', 'test')
dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
res = model._run(dummy_img)
for k, v in res.items():
    print(f"Output: {k}, shape: {v.shape}")
