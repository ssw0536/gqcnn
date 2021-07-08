# RISE GQCNN Test Repository
## Why we forked UC Berkely GQCNN repository?
우리의 시스템은 `ros-melodic`이며 `python2`에서 작업을 하고 있다. 하지만 최신 GQCNN 패키지는 `python2`를 지원을 `drop` 했다.. 따라서 우리는 하위 버전인 `GQCNN 1.1.0`을 사용해야한다. 하지만 일부 UC Berkeley Package들 버전이 맞지 않아 코드가 정상적으로 실행되지 않는 문제를 겪었다. 따라서 `python2`환경에서 `GQCNN 1.1.0`을 사용하기 위한 설치 방법을 제시한다.

## Test System
우리는 다음과 같은 환경에서 코드가 정상적으로 구동되는 것을 확인했다.

* `ubuntu 18.04`
* `ros-melodic`
* `python 2.7`

## Install Guide
1. [virtualevn](https://virtualenv.pypa.io/en/latest/index.html)를 사용하는 것을 권장한다.

    가상환경 생성
    ```bash
    $ roscd gqcnn_grasp_planner
    $ virtualenv -p python2 --system-site-packages venv
    ```
    
    가상환경 활성화
    ```
    $ source venv/bin/activate
    ```
    
2. tensorflow-gpu==1.15 설치한다. 참고로 [Tensorflow Homepage](https://www.tensorflow.org/install/source)에서 해당 버전에 맞는 `CUDA`와 `cuDNN`의 버전을 확인할 수 있다.
    ```bash
    $ pip2 install tensorflow-gpu==1.15
    ```

3. 다음 패키지들 수동으로 설치한다. 아직 확실하지 않기 때문에 개별로 하나씩 설치하는 것을 추천한다.

    추가로 설치한 패키지들
    ```
    autolab-core==0.0.14
    autolab-perception==0.0.8
    imageio==2.6.1
    pyglet==1.4.10
    visualization==0.1.1
    psutil==5.4.2
    gputil==1.4.0(unknown)
    ```
    
    아래는 내 컴퓨터에 기본적으로 깔려 있던 패키지
    ```
    scipy==1.2.2
    numpy==1.16.6
    opencv-python==4.2.0.32
    scikit-image==0.14.5
    scikit-learn==0.20.4
    scikit-video==1.1.11
    ```

3. `.` 폴더(`setup.py`가 있는 폴더)에서 다음 명령 실행으로 설치
    ```
    $ pip install .
    ```

## Usage
### 1. Download pre-trained gqcnn models
[여기서](https://berkeleyautomation.github.io/gqcnn/tutorials/tutorial.html#pre-trained-models) 확인하고 다운받도록 하자.

### 2. ROS node
ROS node는 `cobot_grasp_planner_node.py` 파일에 작성되어 있다.

**ROS Params**

* `model`: GQCNN model dir. For example : `"$(find gqcnn_grasp_planner)/models/FC-GQCNN-4.0-PJ"`
* `config`: GQCNN configuration file path. For example : `"$(find gqcnn_grasp_planner)/cfg/examples/fc_gqcnn_pj.yaml"`

**Service(`GQCNNGraspPlannerSegmask`)**
* Request
    * sensor_msgs/Image color_image
    * sensor_msgs/Image depth_image
    * sensor_msgs/CameraInfo camera_info
    * sensor_msgs/Image segmask
* Response
    * GQCNNGrasp grasp

### 3. ROS launch
```
$ roslaunch gqcnn_grasp_planner cobot_grasp_planning_service.launch
```

# Berkeley AUTOLAB's GQCNN Package
<p>
    <a><img alt="Build Status" src="https://travis-ci.org/BerkeleyAutomation/gqcnn.svg?branch=master"></a>
    <a href="https://github.com/BerkeleyAutomation/gqcnn/releases/latest"><img alt="Release" src="https://img.shields.io/github/release/BerkeleyAutomation/gqcnn.svg?style=flat"></a>
    <a href="https://github.com/BerkeleyAutomation/gqcnn/blob/master/LICENSE"><img alt="Software License" src="https://img.shields.io/badge/license-REGENTS-brightgreen.svg"></a>
    <a><img alt="Python 2 Version" src="https://img.shields.io/badge/python-2.7-yellow.svg"></a>
    <a><img alt="Python 2 Version" src="https://img.shields.io/badge/python-3.5%20%7C%203.6%20%7C%203.7-yellowgreen.svg"></a>
</p>

## Package Overview
The gqcnn Python package is for training and analysis of Grasp Quality Convolutional Neural Networks (GQ-CNNs). It is part of the ongoing [Dexterity-Network (Dex-Net)](https://berkeleyautomation.github.io/dex-net/) project created and maintained by the [AUTOLAB](https://autolab.berkeley.edu) at UC Berkeley.

## Installation and Usage
Please see the [docs](https://berkeleyautomation.github.io/gqcnn/) for installation and usage instructions.

## Citation
If you use any part of this code in a publication, please cite [the appropriate Dex-Net publication](https://berkeleyautomation.github.io/gqcnn/index.html#academic-use).

