# Pe CameraInfo Publisher

### How to run

`python pe_calibration_publisher.py /PATH/TO/ros.yaml NAMESPACE IMAGE_TOPIC`

i.e.

`python pe_calibration_publisher.py /home/data/calibration/camera1.yaml camera1 image_raw`


### ROS YAML format

For details check ROS docs

```yaml
image_width: WIDTH
image_height: HEIGHT
camera_name: CAMERA_NAME
camera_matrix:
  rows: 3
  cols: 3
  data: [ Fx,     0,   CX,
          0.,     Fy,  CY,
          0.,     0.,   1.    ]
camera_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [D0, D1, D2, D3, D4, D5]
rectification_matrix:
  rows: 3
  cols: 3
  data: [ 1.,  0.,  0.,
          0.,  1.,  0.,
          0.,  0.,  1.]
projection_matrix:
  rows: 3
  cols: 4
  data: [ FxO,    0.,    CxO,     0.     ,
          0.,     FyO,   CxO,     0.     ,
          0.,     0.,    1. ,     0.     ]
```