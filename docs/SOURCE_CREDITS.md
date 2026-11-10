# Original sources & project contributions

[← Project home](../README.md)

## Pretrained detection foundation — Ultralytics

This project is built on **Ultralytics YOLOE**, rather than a model trained from scratch for this project.

- **Original source repository:** https://github.com/ultralytics/ultralytics
- **Official YOLOE documentation:** https://docs.ultralytics.com/models/yoloe/
- **Original pretrained checkpoint:** `yoloe-26n-seg.pt`
- **Official checkpoint download:** https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-26n-seg.pt
- **Inference library used:** `ultralytics`, version 8.4.188
- **Upstream licensing information:** https://www.ultralytics.com/license

`tools/prepare_model.py` prepares the nano checkpoint as `models/outdoor_detector.pt`. With `--size small`, it uses **`yoloe-26s-seg.pt`**, [official checkpoint](https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-26s-seg.pt), and saves `models/outdoor_detector_small.pt`. The selected model is the small version, exported to `models/outdoor_detector_small_openvino_model/` for laptop CPU inference.

The current library has 48 canonical object categories and 57 visual prompts. Alternative descriptions in `config/detection_prompts.json` include color/material descriptions of boxes and related descriptions of people. `prompt_library.py` maps these back to stable labels, and duplicate boxes are merged. This configures pretrained models; it is not new training or fine-tuning.

### Additional people-focused model

The optional `models/human_specialist_small.pt` comes from the larger Ultralytics **`yoloe-26s-seg.pt`** checkpoint, [official download](https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-26s-seg.pt). `tools/prepare_human_model.py` configures five related prompts: person, man, woman, human and pedestrian. All map to the existing `person` category. This also uses pretrained weights, not new training. It retains the same upstream AGPL licensing requirements.

This optional second model was used during experiments; it is disabled in the final selected single-model configuration.

`perception_fusion.py` merges duplicate people and suppresses weak, nearly identical conflicting names beneath a stronger person proposal. This is a heuristic, not calibrated probability fusion. Both models see only image pixels.

Exploratory comparison also used the pretrained [Ultralytics YOLO11](https://docs.ultralytics.com/models/yolo11/) nano/small COCO detectors and a nano-outdoor/small-people combination. They are credited as evaluated candidates; the selected application uses one YOLOE small outdoor model.

## Added for this project

The surrounding application code was assembled for this laptop prototype:

- Webcam capture and frame freshness handling.
- Size-based and ground-plane distance estimates, with calibration tools.
- A local dashboard, camera overlays, detection metadata and a read-only API.
- Configuration, project structure, documentation and tests.

These are integrations and additions around the upstream model. They do not claim authorship of YOLOE, its pretrained weights, PyTorch or OpenCV. The distance formulas use standard pinhole-camera geometry, not a newly trained depth model.

## Supporting libraries

- **OpenVINO 2026.4.1:** optimized laptop CPU inference, [source and Apache-2.0 license](https://github.com/openvinotoolkit/openvino). The graph export is handled by the Ultralytics exporter. `tools/optimize_for_laptop.py` recreates these exports. **ONNX 1.23.2** was installed for export compatibility, [source/license](https://github.com/onnx/onnx).
- **Raspberry Pi Camera Module 3 Wide specifications:** [official product brief](https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf). The moving scene uses nominal 102-degree horizontal field of view; it is not a measured lens calibration.

The optional Webots test uses Cyberbotics Webots R2025a and externally referenced Pedestrian, ToyotaPrius and SimpleTree assets. Their separate upstream asset licenses and links are documented in the [simulation credits](../simulation/README.md#credits-and-licenses). These assets are not covered by the project's AGPL license.

- **OpenCV:** camera capture and image annotation — https://github.com/opencv/opencv
- **PyTorch:** model execution — https://github.com/pytorch/pytorch
- **NumPy:** image-array utilities — https://github.com/numpy/numpy

Each dependency retains its own license and notices. The Ultralytics foundation and this project's added source are supplied under AGPL-3.0; see the project's [LICENSE](../LICENSE) and the upstream license information above.


## Model preparation and validation assets

- Text tokenization helper: [Ultralytics CLIP](https://github.com/ultralytics/CLIP), installed for one-time prompt preparation.
- Text encoder checkpoint: [Ultralytics-hosted mobileclip2_b.ts](https://github.com/ultralytics/assets/releases/download/v8.4.0/mobileclip2_b.ts), downloaded by the upstream preparation code. Normal detector inference uses the prepared checkpoint without running this text encoder.
- Inference sanity-check image: [Ultralytics bus photograph](https://ultralytics.com/images/bus.jpg). Used to verify that real model inference returns people and a bus; it does not validate all outdoor categories and is not a webcam recording.
- Python standard library: [CPython](https://github.com/python/cpython), for configuration, threading, timestamps and the local HTTP server.

## Development and installed dependency credits

The project also used [Black](https://github.com/psf/black), [js-beautify](https://github.com/beautifier/js-beautify) (Python jsbeautifier/cssbeautifier packages), and [Beautiful Soup](https://www.crummy.com/software/BeautifulSoup/) to format source files. These are development tools rather than the detection model.

[Full installed software inventory](THIRD_PARTY_SOFTWARE.md) lists package versions, declared license information and original-source links for direct dependencies, transitive dependencies and installed development tools. It can be regenerated with `tools/update_credits.py`.


The mobility catalog, validation loader and future motion/navigation output contract are additional project integration code. No new third-party tracking library is used. The stationary-camera tracker uses project-authored spatial association, least-squares velocity fitting and constant-velocity point/rectangle prediction. Speed and arrival outputs are estimates with the documented scale and tracking limitations.


The blocked/dark-camera heuristic and alert overlay are project-authored code using the already-credited OpenCV and NumPy libraries. No limb/pose model was added.

## Raspberry Pi camera input adapter

The optional CSI adapter uses Raspberry Pi [Picamera2](https://github.com/raspberrypi/picamera2) (BSD-2-Clause), following its [official manual](https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf) and [capture-array colour mapping](https://github.com/raspberrypi/picamera2/blob/main/picamera2/request.py). Picamera2/libcamera is supplied by Raspberry Pi OS, not installed in this Windows environment. Adapter glue and camera selection were added for this project; no pretrained model was trained here.
