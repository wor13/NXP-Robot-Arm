# Sources and third-party components

## Robot geometry

Bundled OMX-F URDF and STL geometry originate from [ROBOTIS-GIT/open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator), commit `ee9a720bcc45acda9c0df8fe689b1182844307db`, from `open_manipulator_description`. The supplied upstream Apache 2.0 license is retained at `models/ROBOTIS_LICENSE`. Upstream filenames and source package metadata are retained. The local renderer implements the needed fixed/revolute URDF transforms without ROS.

The custom V and slot jaw viewer meshes were transformed from this project's reinforced jaw STEP designs to the local `link6` and `link7` frames. These STL files are in **meters for rendering**, unlike printable millimeter STL exports. Use the separate gripper print kits to fabricate the jaws. Changing a display mesh does not establish mounting fit, strength, tool calibration or collision clearance.

## Python dependencies

- [Viser](https://github.com/nerfstudio-project/viser): local interactive 3D scene and controls.
- [Trimesh](https://github.com/mikedh/trimesh): mesh loading.
- [NumPy](https://numpy.org/): numerical arrays.
- [SciPy](https://scipy.org/): bounded nonlinear least squares for IK.
- [DYNAMIXEL SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK): official Protocol 2.0 serial transport.
- [pySerial](https://github.com/pyserial/pyserial): serial ports.
- pytest is used for developer checks. An optional independent URDF comparison uses yourdfpy, which is deliberately not a runtime dependency.

See each upstream project for its license. Installed package distributions include their own license metadata. No SDK, Viser, SciPy or Python runtime binaries are bundled in this ZIP.

## Hardware and setup references

Reviewed October 21, 2026:

- [OpenRB-150 official manual](https://emanual.robotis.com/docs/en/parts/controller/openrb-150/): board setup, factory `usb_to_dynamixel`, power path and Arduino upload.
- [OMX hardware specifications](https://docs.robotis.com/docs/systems/omx/specifications/hardware/): OMX-F motor IDs/models and system layout.
- [XL430-W250 control table](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/).
- [XL330-M288 control table](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/).

Wolfram was used to evaluate the straight-path polynomial derivatives, peak normalized velocity/acceleration and the nominal ring/post clearance. It did not validate a physical assembly or motor communications.
