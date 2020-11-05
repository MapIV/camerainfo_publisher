# Pe CameraInfo Publisher

### How to run

`rosrun pe_camerainfo_publsher pe_calibration_publisher.py _calibration_yaml:=/PATH/TO/ros.yaml _namespace:=NAMESPACE _input_topic:=IMAGE_TOPIC`

i.e.

`rosrun pe_camerainfo_publsher pe_calibration_publisher.py _calibration_yaml:=/home/data/calibration/camera1.yaml _namespace:=camera1 _input_topic:=image_raw`


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