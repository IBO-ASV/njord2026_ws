#!/usr/bin/env python3
"""Export a YOLO-World v2 ``boat`` detector for the ZED TensorRT pipeline.

The output is deliberately limited to one text prompt (``boat``).  It uses
the raw ``[1, 5, N]`` output supported by zed2i_driver, which performs the
small NMS step after TensorRT inference.  Run ``trtexec`` on the target
Jetson afterwards, so the serialized engine matches its TensorRT version.
"""

import argparse
import os
import sys


def main():
    parser = argparse.ArgumentParser(description='Export YOLO-World v2 boat ONNX model.')
    parser.add_argument(
        '--weights', default='yolov8s-worldv2.pt',
        help='YOLO-World v2 .pt weights (download it with Ultralytics if absent)')
    parser.add_argument(
        '--output', default='src/robot/config/yolo_model/yolov8s-world-boat.onnx',
        help='Output ONNX path')
    parser.add_argument('--imgsz', type=int, default=640)
    args = parser.parse_args()

    try:
        from ultralytics import YOLOWorld
    except ImportError as error:
        parser.error(f'install Ultralytics first: {error}')

    model = YOLOWorld(args.weights)
    model.set_classes(['boat'])
    output = os.path.abspath(args.output)
    exported = model.export(
        format='onnx', imgsz=args.imgsz, device='cpu', nms=False,
        dynamic=False, simplify=False)
    exported = os.path.abspath(str(exported))
    if exported != output:
        os.makedirs(os.path.dirname(output), exist_ok=True)
        os.replace(exported, output)
    print(output)
    print('Build on Jetson: trtexec --onnx=' + output + ' --saveEngine=' + output[:-5] + '.engine --fp16')
    print('Launch with: enable_vessel_perception:=true vessel_engine_path:=' + output[:-5] + '.engine')


if __name__ == '__main__':
    sys.exit(main())
