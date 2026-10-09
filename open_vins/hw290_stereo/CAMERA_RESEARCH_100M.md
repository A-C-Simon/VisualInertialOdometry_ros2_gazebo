# Chinese global shutter cameras for OpenVINS at 100 m

Research date: 9 October 2026. Status: engineering shortlist, no purchase or flight validation.

## 1. Recommendation

Evaluate **two Daheng MER2-503-36U3M monochrome cameras**, matched 8 mm lenses,
a rigid **450 mm centre-to-centre baseline**, and a common hardware exposure
trigger. Keep 12 mm lenses as an alternative to evaluate for greater angular
detail. These are separate cameras assembled into a stereo rig. The baseline
is a design starting point, subject to the complete mounting envelope and payload.

The 3 MP **MER2-302-56U3M** is a useful alternative when bandwidth and image
processing cost matter more than field coverage. Hikrobot's
**MV-CS050-10UM V5** is a second supplier option, subject to verification of
the exact revision's trigger, timestamp and output modes.

Confirmed requirements: **100 m above ground**, flight speed **up to 10 m/s**,
**maximum stereo width 500 mm**, **global shutter only**, and **Jetson Orin NX
class compute**. Budget is excluded from the current selection. Downward view
remains an assumption. The exact computer, memory, carrier board, operating
system, payload and required trajectory accuracy are still unspecified.

Treat the 500 mm limit as overall assembly width until clarified. It is not
automatically a 500 mm optical baseline: camera bodies, lenses, brackets and
cable clearances occupy space beyond the lens centres. A 450 mm baseline plus
the 8 mm lens's [listed 33 mm barrel diameter](https://en.daheng-imaging.com/show-472-702-1.html)
spans roughly 483 mm before additional mounting clearance. Check drawings and
cable routing before fixing the baseline.

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

Budget is deliberately excluded from the present technical ranking.
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

The table's 500 mm baselines are geometry references, not assemblies claimed
to fit the confirmed 500 mm overall width. For the proposed **450 mm baseline**,
disparity at 100 m is **10.43 px with 8 mm lenses**, or **15.65 px with 12 mm
lenses**. Ground sampling and FOV remain as listed for each lens.

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

### Confirmed computer performance class: Jetson Orin NX

Orin NX class compute is the design reference, not confirmation of an actual
NVIDIA module or a particular carrier board. It keeps the 5 MP, 30 fps pair
worth evaluating; it does not establish that the complete pipeline sustains
30 fps. No benchmark has been run on the proposed computer.

The existing [KLT tracker](../ov_core/src/track/TrackKLT.cpp) calls
`cv::calcOpticalFlowPyrLK`, and the [splitter](../ov_hw290/src/stereo_splitter.cpp)
uses CPU `cv::remap` with `cv::Mat`. These paths do not automatically use CUDA
or Tensor cores. AI TOPS is not a prediction of estimator throughput. Measure
CPU load, frame latency, thermal throttling and delivery gaps at the aircraft's
actual power mode, including other onboard workloads.

If the computer is an actual Orin NX, a JetPack 6 based system is a relevant
integration target: [NVIDIA documents Ubuntu 22.04 for JetPack 6.2](https://developer.nvidia.com/embedded/jetpack-sdk-62).
The Daheng ARM SDK listing currently recommends only Ubuntu through 20.04,
so exact aarch64/JetPack support remains a vendor-confirmation and bench-test
requirement. Hikrobot's aarch64 package is a second option to investigate,
not proof of compatibility with the intended board and OS.

For USB acquisition, budget 300.81 MB/s of raw stereo image payload at 30 fps,
plus transport and recording overhead. Inspect the actual carrier's topology:
[NVIDIA's bring-up guide](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/HR/JetsonModuleAdaptationAndBringUp/JetsonOrinNxNanoSeries.html)
describes carrier-specific USB routing. Multiple sockets do not guarantee
independent upstream bandwidth. GMSL2 or CSI would require matched camera,
deserializer and driver support; similar compute power alone does not provide
these interfaces.

If profiling shows image preparation dominates on an actual NVIDIA platform,
[VPI Remap](https://docs.nvidia.com/vpi/group__VPI__Remap.html) offers a CUDA
backend to investigate. This is a possible software change, not a feature of
the working pipeline or a measured performance gain. Keep timestamp semantics,
rectified calibration and trajectory replay checks when evaluating it.

Keep full-resolution 5 MP capture as the first candidate. If measured latency
is excessive, compare an ROI or the 3 MP camera with the same pixel pitch and
lens before aggressively resizing. Those options retain central angular
sampling at the cost of ground coverage. Compare accuracy and compute on the
same recordings. Budget is not the reason to select the 3 MP fallback.

### Confirmed speed: up to 10 m/s

For a downward pinhole camera over flat ground, horizontal translation gives:

```text
image_speed_px_per_s = f_px * ground_speed_m_per_s / height_m
translation_per_frame_px = image_speed_px_per_s / frame_rate_hz
translation_blur_px = image_speed_px_per_s * exposure_s
```

At 100 m height, 10 m/s and 30 fps:

| Lens | Translational shift per frame | Blur at 1 ms exposure | Blur at 2 ms exposure |
| --- | ---: | ---: | ---: |
| 8 mm | 7.73 px | 0.23 px | 0.46 px |
| 12 mm | 11.59 px | 0.35 px | 0.70 px |

Start evaluation at 30 fps and around 1 ms exposure in adequate daylight.
This limits translational blur; it is not a universal exposure setting or a
guarantee of successful KLT tracking. Lighting, gain, rotation and vibration
must be measured. Rotation contributes approximately `f_px * angular_rate *
exposure_s` near the image centre. At 10 m height, the table's translational
shifts and blur are ten times larger for the same speed. Takeoff, descent,
lower-altitude travel and turns therefore need separate validation.

The 8 mm lens is the initial choice because its wider field provides more
coverage and smaller inter-frame displacement than 12 mm, while the 450 mm
baseline still provides about 10 px of stereo disparity at the target height.
This is an engineering tradeoff for testing, not an established accuracy result.

## 7. Evidence needed before final selection

1. Confirm downward view, terrain, acceptable drift, exact computer/OS and payload.
   Use the confirmed 100 m above-ground target, 10 m/s maximum speed and
   500 mm width limit when checking the assembly envelope.
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

**Decision to discuss:** begin with the adjustable Daheng stereo prototype,
8 mm lenses and a nominal 450 mm baseline within the 500 mm width limit.
Confirm the complete mechanical envelope and exact computer/driver support
before ordering. Budget is excluded from the present ranking. Review
temporal-parallax requirements and estimator gates as part of the aerial profile.
