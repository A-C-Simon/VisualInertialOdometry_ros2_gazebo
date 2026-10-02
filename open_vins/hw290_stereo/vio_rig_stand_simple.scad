// VIO stand - SIMPLE (v5)
// ELP front, lens 200mm up | Nano + HW290 stacked on mast REAR
// Nano: open-top + open-end box (drop/slide in, USB free, no label)
// HW290 21x20: lidded pocket, slides in from +X side, wire notch rear

cam_w = 80; cam_h = 16.5; cam_thick = 8; baseline = 60; lens_d = 14;
nano_l = 43.2; nano_w = 17.8; nano_thick = 5;
imu_w = 21; imu_l = 20; imu_thick = 4;

base_l = 120; base_w = 80; base_t = 6;
mast_w = 24; mast_d = 24; cam_z = 200;
ghosts = true;
$fn = 24;

// --- base ---
difference() {
    translate([-base_l/2, -base_w/2, 0]) cube([base_l, base_w, base_t]);
    for (x=[-1,1], y=[-1,1])
        translate([x*(base_l/2-10), y*(base_w/2-10), -1])
            cylinder(d=3.2, h=base_t+2, $fn=16);
}

// --- mast + gussets ---
translate([-mast_w/2, -mast_d/2, base_t-1])
    cube([mast_w, mast_d, cam_z]);
for (r=[0,90,180,270]) rotate([0,0,r])
    translate([mast_w/2-1, mast_d/2-1, base_t-1])
        linear_extrude(mast_w) polygon([[0,0],[0,30],[30,0]]);

// --- camera mount (front) ---
// AGENT: ELP 3DGS1200P01 stereo mounts HERE. Lens optical centers (rig frame,
// mm, X right / Y forward / Z up, origin base center): LEFT (-30, 15, 206),
// RIGHT (+30, 15, 206). baseline=60 (line 6), lens plane y = cam_plate_y+4,
// height z = base_t+cam_z. Plate 96 wide, lens cutouts d=lens_d+1.5, USB slot
// center, 2x M2 holes at x=+-34. See CALIB block at file end for extrinsics.
cam_plate_y = mast_d/2 - 1;
difference() {
    translate([-48, cam_plate_y, base_t+cam_z-15])
        cube([96, 4, 30]);
    for (s=[-1,1])
        translate([s*baseline/2, cam_plate_y-1, base_t+cam_z])
            rotate([90,0,0]) cylinder(d=lens_d+1.5, h=6, $fn=32);
    translate([-8, cam_plate_y-1, base_t+cam_z-9])
        cube([16, 6, 8]);
    for (s=[-1,1])
        translate([s*34, cam_plate_y-1, base_t+cam_z-7])
            rotate([90,0,0]) cylinder(d=2.2, h=6, $fn=12);
}
translate([-48, cam_plate_y, base_t+cam_z+13]) cube([96, 4, 4]);
if (ghosts)
    %color("deepskyblue", 0.45)
        translate([-cam_w/2, cam_plate_y+4, base_t+cam_z-8])
            cube([cam_w, cam_thick, cam_h]);

// --- HW290 BOX (rear, level with camera): lidded pocket, REAR entry ---
// AGENT: HW290 IMU (21 x 20 x 4mm incl. pin connector) mounts HERE. Board
// seats pushed fully forward: center (0, -16.7, 206), plane vertical facing
// rear, 21mm edge along X. Pocket = board + fit*2 per side (fit line ~54).
// Open full-width rear slot (push in from behind), 3mm lid on top, +X side
// closed, I2C leads exit the rear slot. Lever arm to lenses in CALIB block
// at file end. Rotation R_imu_cam is NOT constrained by this pocket: it
// depends on ICM-20689 die orientation on the breakout + insert pose.
// top COVERED, +X side CLOSED, full-width rear slot: push board
// straight in from behind, I2C leads exit the same rear slot
// Push-fit: 0.3mm/side clearance on 21 x 20 x 4 board (tune `fit` if needed)
fit = 0.3;
imu_slot_x = imu_w + fit*2;       // 21.6
imu_slot_y = imu_thick + fit*2;   // 4.6
imu_slot_z = imu_l + fit*2;       // 20.6
imu_floor = base_t+cam_z-10;      // 196
imu_rim = imu_floor + 24;         // 220 outer top
echo(str("HW290 pocket: ", imu_slot_x, " x ", imu_slot_y, " x ", imu_slot_z,
         " for board ", imu_w, " x ", imu_thick, " x ", imu_l));
difference() {
    // outer: slot + 2mm walls, 3mm floor/lid, fused into mast
    translate([-(imu_slot_x/2+2), -mast_d/2-7, imu_floor-3])
        cube([imu_slot_x+4, imu_slot_y+3.4, 27]);
    // interior slot, open through entire rear face, lid on top
    translate([-imu_slot_x/2, -mast_d/2-7, imu_floor])
        cube([imu_slot_x, imu_slot_y, imu_slot_z]);
}
if (ghosts)
    %color("orange", 0.55)
        translate([-imu_w/2, -mast_d/2-7+fit, imu_floor])
            cube([imu_w, imu_thick, imu_l]);

// --- NANO BOX (rear, halfway up): open roof + open +X end ---
// floor + walls only, board drops in from top or slides from end
nano_z0 = 107;   // floor bottom, walls to 120
difference() {
    // outer 48 x 22.6 x 13, back embedded 1mm into mast
    translate([-24, -mast_d/2-21.6, nano_z0]) cube([48, 22.6, 13]);
    // interior 44 x 18.6, open through top (+roof) and +X end
    translate([-22, -mast_d/2-19.6, nano_z0+3]) cube([47, 18.6, 11]);
}
if (ghosts)
    %color("limegreen", 0.5)
        translate([-nano_l/2, -mast_d/2-19.2, nano_z0+3])
            cube([nano_l, nano_w, nano_thick]);

// --- label: ELP only ---
translate([0, 15.5, 212]) rotate([90,0,0]) mirror([1,0,0])
    linear_extrude(1.5) text("ELP", size=7, halign="center", valign="center");

// === CALIB: single source of truth for camera-IMU extrinsics ===
// Rig frame: X right, Y forward (lens look dir), Z up, origin at base center.
// Optical frame (cam0/cam1): x right, y DOWN, z forward.
// p_opt = [dx_rig, -dz_rig, dy_rig]. Units below in METERS for Kalibr/OpenVINS.
// NOTE: translation below is exact from CAD. Rotation R_imu_cam is NOT in
// CAD (depends on ICM-20689 die orientation on YOUR breakout + insert pose):
// carry over the old R only if chip orientation vs camera is unchanged,
// else re-run hw290_stereo/calibrate_rotation.py. Time offset keep 0.02.
lens_y = cam_plate_y + 4;          // lens plane (plate front)
lens_z = base_t + cam_z;           // 206mm above z=0
imu_cx = 0;
imu_cy = -mast_d/2 - 7 + fit + imu_thick/2;  // board pushed fully in
imu_cz = imu_floor + imu_l/2;
cam0_p = [imu_cx - (-baseline/2), -(imu_cz - lens_z), (imu_cy - lens_y)] / 1000;
cam1_p = [imu_cx - (baseline/2),  -(imu_cz - lens_z), (imu_cy - lens_y)] / 1000;
echo(str("CALIB baseline_m: ", baseline/1000));
echo(str("CALIB p_IMU_in_cam0_m: ", cam0_p));
echo(str("CALIB p_IMU_in_cam1_m: ", cam1_p));
echo("CALIB rotation: reuse old R_imu_cam iff chip orientation unchanged, else recalibrate; timeshift keep 0.02");
