# Chinese global shutter cameras for OpenVINS at 100 m

Research date: 9 October 2026. Status: engineering shortlist, no purchase or flight validation.

## 1. Recommendation

Evaluate **two Daheng MER2-503-36U3M monochrome cameras**, matched 8 mm or
12 mm lenses, a rigid **300 to 500 mm baseline**, and a common hardware
exposure trigger. These are separate cameras assembled into a stereo rig.
The suggested baseline is a design starting point, subject to the aircraft's
width, payload and vibration constraints.

The 3 MP **MER2-302-56U3M** is a useful alternative when bandwidth and image
processing cost matter more than field coverage. Hikrobot's
**MV-CS050-10UM V5** is a second supplier option, subject to verification of
the exact revision's trigger, timestamp and output modes.

Assumptions: 100 m means height above ground, with cameras looking downward.
Global shutter only is confirmed. Aircraft speed, computer, budget, allowable
baseline, payload and required trajectory accuracy are still unspecified.

No reviewed manufacturer evidence establishes that these particular camera
assemblies deliver a specified OpenVINS trajectory accuracy at 100 m.
The recommendation follows optical geometry and documented interfaces.

## 2. Why the present camera becomes difficult

For rectified stereo with aligned principal points:

```text
disparity_px = focal_length_px * baseline_m / depth_m
```

This is the standard [OpenCV stereo relation](https://docs.opencv.org/3.4.18/dd/d53/tutorial_py_depthmap.html).
It describes instantaneous stereo geometry, not the complete VIO estimator.

Our measured tower profile has a 57.33584 mm baseline and a rectified focal
length of 372.46673 px. Its predicted disparity is:

| Ground distance | Disparity |
| --- | ---: |
| 10 m | 2.136 px |
| 30 m | 0.712 px |
| 60 m | 0.356 px |
| 100 m | 0.214 px |

Source: [working stereo calibration](calibration/20261002_tower/candidate/stereo_opencv.yaml),
using `P1[0,0]` and `-P2[0,3] / P1[0,0]`.

At 100 m, small matching or calibration errors compete with the entire
left/right displacement. There is no universal 10 m failure boundary.
OpenVINS also uses observations across time and the IMU, so weak instantaneous
stereo depth does not alone prove that VIO must fail at a particular height.
Terrain texture, translation, image sharpness and timing determine whether
useful constraints remain. See [OpenVINS feature triangulation](https://docs.openvins.com/update-featinit.html).

Global shutter removes exposure skew between rows. It does not remove motion
blur, synchronize separate cameras automatically, or improve parallax by itself.

## 3. Camera shortlist

All options below use global shutter sensors. Daheng is based in
[Beijing, China](https://en.daheng-imaging.com/list-5-1.html);
Hikrobot lists its address in
[Hangzhou, China](https://www.hikrobotics.com/en/contactus/).
Chinese camera manufacture does not imply an entirely Chinese component bill
of materials. The shortlisted Daheng cameras use Japanese Sony sensors.

| Candidate | Manufacturer specifications | Assessment |
| --- | --- | --- |
| **2 x Daheng MER2-503-36U3M** | Mono IMX264; 2448 x 2048; 36 fps; 3.45 micrometre pixels; USB 3; C/CS lens mount; opto-isolated input/output plus GPIO; 65 g per body | First prototype choice: adjustable optics and stereo spacing, with documented trigger hardware. |
| **2 x Daheng MER2-302-56U3M** | Mono IMX265; 2048 x 1536; 56 fps; 3.45 micrometre pixels; USB 3; C/CS mount; trigger I/O; 65 g per body | Fewer pixels and a smaller field of view at the same focal length. Useful if the computer cannot sustain the 5 MP pair. |
| **2 x Hikrobot MV-CS050-10UM V5** | Current catalogue lists IMX264, 2448 x 2048, USB 3 and up to 74 fps | Supplier alternative. Obtain the exact V5 datasheet before ordering; do not substitute specifications from older 60 fps versions. |

Primary specifications: [Daheng 5 MP](https://en.daheng-imaging.com/show-106-1990-1.html),
[Daheng 3 MP](https://en.daheng-imaging.com/show-106-1983-1.html),
[Hikrobot current catalogue](https://www.hikrobotics.com/en/machinevision/visionproduct/?id=134&pageNumber=2&pageSize=50&showEol=false&typeId=78).
Sony independently identifies [IMX264 as global shutter](https://developer.is.sony-semicon.com/imx264).

For Daheng, order the full trigger-capable model, **without the `-L` suffix**.
The [manufacturer's USB3 manual](https://www.dahengimaging.com/downloads/USB3-MER2-ME2P-Vision-Cameras-User-Manual-V1.0.10.pdf)
distinguishes external-trigger versions from software-trigger-only `-L` versions.
Also avoid similarly named polarization models unless deliberately required.

The 5 MP pair weighs 130 g before lenses, bracket, cables, enclosure and IMU.
Both Daheng product pages specify 0 to 45 degrees C operation. Aircraft
temperature and enclosure requirements therefore need checking.

### Lenses and initial configuration

For the 2/3-inch 5 MP sensor, Daheng lists these matching C-mount lens options:

- **HN-P-0828-6M-C2/3**, 8 mm, F2.8 to F16.
- **HN-P-1228-6M-C2/3**, 12 mm, F2.8 to F16.

Source: [Daheng 6 MP lens catalogue](https://en.daheng-imaging.com/index.php?a=prolists&c=index&catid=472&m=content).
Use two identical lenses; verify focus at ground distances, lock focus and
aperture, and calibrate the final assembly. The narrower 12 mm view offers
more angular detail but less ground coverage and overlap during attitude changes.

### Prices and procurement status

No verified public manufacturer price was found for either industrial camera
pair. Obtain a China-sourced quote covering **two camera bodies, two lenses,
locking USB cables, I/O cables, trigger hardware and the bracket**, with exact
model suffixes, lead time and SDK access. Pricing and availability remain open.

## 4. Optical design examples at 100 m

These calculations use the 5 MP sensor's 3.45 micrometre pixel pitch and its
2448 x 2048 output. They assume ideal pinhole geometry before rectification
crop or lens distortion. Final intrinsics must be measured.

```text
f_px = lens_focal_length_mm / pixel_pitch_mm
HFOV = 2 * atan(image_width_px / (2 * f_px))
ground_sample_distance_m_per_px = height_m / f_px
sigma_depth_m ~= depth_m^2 * sigma_disparity_px / (f_px * baseline_m)
```

| Lens | Baseline | Horizontal FOV | Ground detail at 100 m | Disparity at 100 m | Illustrative depth sigma at 0.2 px disparity sigma |
| --- | ---: | ---: | ---: | ---: | ---: |
| 8 mm | 0.30 m | 55.65 degrees | 4.31 cm/px | 6.96 px | 2.88 m |
| 8 mm | 0.50 m | 55.65 degrees | 4.31 cm/px | 11.59 px | 1.73 m |
| 12 mm | 0.30 m | 38.77 degrees | 2.88 cm/px | 10.43 px | 1.92 m |
| 12 mm | 0.50 m | 38.77 degrees | 2.88 cm/px | 17.39 px | 1.15 m |

At 100 m, the 8 mm configuration covers approximately 105.57 x 88.32 m;
12 mm covers 70.38 x 58.88 m. A 300 mm baseline with 8 mm optics gives about
**32.6 times** the current rig's predicted disparity at that distance.

The 0.2 px uncertainty is an illustrative assumption, not a measured camera
specification. These are single-point stereo depth uncertainties, **not VIO
trajectory errors**. More parallax does not imply the same factor of improvement
in trajectory accuracy. Vibration, calibration, blur and scene content add errors.

With the 3 MP sensor and the same lens/pixel pitch, focal length in pixels and
central ground sampling are unchanged; the image covers less ground. It has
37.25% fewer pixels per frame than the 5 MP option. That is a pixel-count
reduction, not a measured CPU saving.

## 5. Integrated stereo cameras considered

### SENSING Astra S56C

The Shenzhen manufacturer's [camera documentation](https://wiki.sensing-world.com/docs/6_1_Camera/Binocular_Camera/S56)
lists dual 2560 x 1984 global shutter sensors at 30 fps, a 60 mm baseline,
140-degree horizontal view, BMI088 IMU and GMSL2. S56C is the current model;
S56 is being phased out. The [official store](https://www.sgwrd.com/products/5mp-global-shutter-stereo-camera)
displayed a $579 listing, but the selected variant, adapter, cables, duties and
shipping need confirmation.

It improves integration, but its short baseline and very wide optics do not
establish good 100 m stereo conditioning. Its infinity focus range is not an
accuracy specification. Do not select it for this altitude from resolution alone.
The wide-angle projection also needs actual calibration; a simple pinhole FOV
calculation would be misleading.

The [official SDK repository](https://github.com/SENSING-Technology/sgMIX)
targets Jetson and states that the camera driver is separate. ISP controls
require network authorization. The exact board, JetPack, driver, timestamp
semantics and offline operating requirements must be checked. It is not a
direct USB replacement for our present x86 computer.

### Orbbec Gemini 2 XL

The [manufacturer datasheet](https://orbbec.com/wp-content/uploads/2023/12/ORBBEC_Datasheet_Gemini-2-XL.pdf)
specifies global shutter imagers, 0.4 to 20 m depth range and an optimal
0.4 to 10 m range. This provides no support for using its depth output at 100 m.
Raw-image VIO is a separate question, but it is not the first prototype choice
for the present requirement.

## 6. OpenVINS integration requirements

### Current software limits

The working desk configuration must not be treated as an aerial profile:

1. [FeatureInitializerOptions.h](../ov_core/src/feat/FeatureInitializerOptions.h)
   defaults to `fi_max_dist = 60` m. The active HW290 estimator YAML does not
   override it. [FeatureInitializer.cpp](../ov_core/src/feat/FeatureInitializer.cpp)
   rejects triangulated anchor-camera depth beyond this threshold, including
   during refinement. Ground features near 100 m camera depth would be rejected.
2. `fi_max_baseline = 40` limits feature distance divided by the effective
   transverse observation baseline. This includes movement across observations,
   not only physical stereo spacing. A point about 100 m away needs roughly
   2.5 m of effective transverse baseline to satisfy that ratio. A 300 mm stereo
   bar alone does not meet it. Review track lifetime, clone spacing and gates
   together, rather than simply disabling checks.
3. [stereo_splitter.cpp](../ov_hw290/src/stereo_splitter.cpp) accepts packed
   1280 x 480 RGB frames and produces 640 x 480 views.
   [build_rectified_profile.cpp](../ov_hw290/src/build_rectified_profile.cpp)
   explicitly requires 640 x 480 per camera. New industrial cameras need an SDK
   acquisition adapter and image-size support in the calibration tooling.

These are source inspections, not new flight results. No working estimator or
calibration settings were changed during this research.

### Acquisition and synchronization

- Acquire raw Mono8 images in a C++ ROS 2 node. OpenVINS uses sparse image
  tracks and IMU measurements; dense depth computation is unnecessary for this
  integration.
- Trigger both exposures from a common hardware source. Pair frame counters and
  detect missing frames. Verify trigger voltage levels against the exact manual.
- Establish exposure timestamp meaning and a common time relationship with the
  IMU. Hardware stereo triggering alone does not synchronize the IMU. Retaining
  the Nano/HW290 requires a verified timestamp mapping or trigger-event capture.
- Fit new camera intrinsics, stereo extrinsics, camera/IMU rotation and
  translation, and time offset. The tower calibration cannot be reused.
- Limit exposure for translation and rotational vibration. As a design example,
  at 100 m, 10 m/s and 8 mm optics, a 2 ms exposure produces about 0.46 px of
  translational blur. Rotational blur is additional and can dominate.

Daheng supplies [Linux x86 and ARM Galaxy SDKs](https://en.daheng-imaging.com/index.php?a=lists&c=index&catid=59&m=content&sylx=23&syxj=44).
The x86 package lists Ubuntu through 24.04; the ARM package's recommended
systems stop at 20.04 and its listed boards are older. Verify the exact embedded
computer before assuming current Jetson or Ubuntu ARM compatibility.
Hikrobot's catalogue lists Linux x86_64 and aarch64 MVS downloads. Neither SDK
listing proves a validated ROS 2 Humble stereo/IMU driver for our application.

### Bandwidth and compute

At 30 fps, two uncompressed Mono8 streams require, before transport overhead:

| Configuration | Image payload |
| --- | ---: |
| 2448 x 2048 stereo | 300.81 MB/s |
| 2048 x 1536 stereo | 188.74 MB/s |

At 60 fps the 5 MP pair would need 601.62 MB/s. This exceeds the 500 MB/s
ceiling after 8b/10b encoding on one shared 5 Gbit/s USB 3 Gen 1 link, even
before protocol overhead. Separate controller paths or lower frame rates would be needed.
Check the actual USB topology; separate sockets can share one upstream link.

Benchmark acquisition, rectification, tracking, estimator CPU and memory on the
flight computer. Our 640 x 480 desk measurements do not predict 5 MP cost.
Resizing a high-resolution image back to 640 pixels reduces focal length in
pixels and loses much of the far-range advantage. Cropping preserves pixel
sampling but narrows coverage; it is different from downsampling.

## 7. Evidence needed before final selection

1. Confirm downward view and 100 m above-ground target, terrain, flight speed,
   acceptable drift, computer, payload, budget and available stereo width.
2. Obtain exact quotations and written confirmation of external exposure trigger,
   raw image access, frame counters, timestamp clock and target Linux support.
3. Calibrate a prototype and replay its recordings before live aerial evaluation.
   Verify image sharpness, matching, accepted visual updates, delivery gaps and
   absence of estimator resets.
4. Compare repeated runs at increasing ground distances, up to 100 m, with
   independent ground truth such as logged RTK. Keep ground truth out of the VIO
   input when measuring standalone VIO. Report ATE, RPE, altitude error, failures,
   CPU seconds, update latency, memory, power and dropped frames.
5. Include ascent, descent, hovering and turns over representative terrain.
   Tests on textured buildings do not establish performance over water or
   uniform vegetation. Set acceptance thresholds from the mission requirement.

**Decision to discuss:** begin with the adjustable Daheng stereo prototype if
the aircraft accepts a 300 to 500 mm bar. If it cannot, reconsider optics and
temporal-parallax requirements before choosing another compact stereo module.
