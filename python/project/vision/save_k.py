import json

W, H = 1280, 960
fx, fy = 1430.0, 1430.0
cx, cy = 480.0, 620.0

data = {
  "image_width": W,
  "image_height": H,
  "camera_matrix": {
    "fx": fx, "fy": fy, "cx": cx, "cy": cy, "skew": 0.0,
    "K": [[fx, 0.0, cx],
          [0.0, fy, cy],
          [0.0, 0.0, 1.0]]
  },
  "distortion": {
    "model": "none",
    "coeffs": [0.0, 0.0, 0.0, 0.0, 0.0]
  },
  "transforms_applied": {   # record what the driver does; prefer all False/0
    "rotate_deg": 0,        # set 90/180/270 if rotation is applied upstream
    "hflip": True,          # your setting says Yes
    "vflip": False
  }
}

with open("camera_intrinsics.json", "w") as f:
    json.dump(data, f, indent=2)
print("Wrote camera_intrinsics.json")