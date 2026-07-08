
/*
  Parametric two-layer enclosure for a 2-axis servo gimbal + controller board.

  Usage in OpenSCAD:
    1. Open this file.
    2. Change the parameters in the "Main dimensions" section.
    3. Set `part` to one of:
       "assembly", "bottom_shell", "mid_plate", "top_left", "top_right", "dome", "dome_alignment_check", "print_layout".
    4. Render (F6) and export STL.
*/

$fn = 96;
part = "dome_alignment_checka";

// ---------- Main dimensions, in mm ----------
box_length = 240;
box_width = 240;
wall_thickness = 3;
floor_thickness = 3;
corner_radius = 8;

// Lower layer: controller board + power bank.
// User target: 80 mm from the bottom to the top face of the separator plate.
lower_inner_height = 74;

// Removable separator plate.
mid_plate_thickness = 3;
plate_clearance = 0.45;

// Upper layer: gimbal base area up to roughly the rotating disc.
// User target: 40 mm above the separator plate, total shell height 120 mm.
upper_inner_height = 40;

// Removable split top cover.
top_cover_thickness = 3;
top_cover_clearance = 0.35;
split_gap = 0.8;

// Circular cutout around the gimbal rotating/base disc.
// Set this slightly larger than the largest moving radius you want to expose.
top_cutout_radius = 55;
reference_disc_radius = 50;

// Four copper standoff / locating post holes in the removable separator plate.
// Measure center-to-center spacing on the gimbal base.
gimbal_standoff_spacing_x = 69;
gimbal_standoff_spacing_y = 69;
gimbal_standoff_hole_d = 6.5;

// Diagonal rectangular relief window on the separator plate for the lower servo
// protrusion. The short-side midpoint nearest the reference standoff hole lies
// on the standoff square diagonal, 17 mm away from the hole center.
servo_window_enabled = true;
servo_window_w = 22;
servo_window_l = 50;
servo_window_gap_from_hole = 17;
servo_window_ref_x_side = -1;
servo_window_ref_y_side = 1;

// Transparent dome over the gimbal.
dome_radius = 100;
dome_wall_thickness = 2;
dome_flange_width = 8;
dome_flange_height = 3;
dome_mount_hole_d = 3.4;
dome_mount_radius = 98;

// Four corner screws hold the split top cover to the lower shell.
top_cover_mount_spacing_x = 208;
top_cover_mount_spacing_y = 208;
top_cover_mount_clearance_d = 3.4;
top_cover_post_pilot_d = 2.8;
top_cover_post_outer_d = 10;
top_cover_post_clearance_d = 11.4;
top_cover_post_rib_thickness = 4;
top_cover_plate_corner_clearance = 30;

// Optional controller-board mounting bosses in the lower layer.
board_bosses_enabled = false;
board_mount_spacing_x = 92;
board_mount_spacing_y = 58;
board_boss_outer_d = 8;
board_screw_hole_d = 2.8;
board_boss_height = 6;

// Cable / connector openings on the shell.
front_usb_opening_w = 20;
front_usb_opening_h = 10;
front_usb_opening_x = -42;

front_power_opening_w = 20;
front_power_opening_h = 10;
front_power_opening_x = 42;

left_debug_opening_w = 34;
left_debug_opening_h = 14;
left_debug_opening_y = 16;

right_cable_slot_w = 42;
right_cable_slot_h = 13;
right_cable_slot_y = -12;

rear_vent_slots = 5;
rear_vent_slot_w = 4;
rear_vent_slot_h = 22;

// Finger notches make plates easier to lift out.
finger_notch_radius = 12.5;

eps = 0.02;
box_height = floor_thickness + lower_inner_height + mid_plate_thickness + upper_inner_height;
inner_length = box_length - 2 * wall_thickness;
inner_width = box_width - 2 * wall_thickness;
plate_length = inner_length - 2 * plate_clearance;
plate_width = inner_width - 2 * plate_clearance;

module rounded_rect_2d(l, w, r) {
    safe_r = min(r, min(l, w) / 2 - 0.01);
    offset(r = safe_r)
        square([l - 2 * safe_r, w - 2 * safe_r], center = true);
}

module rounded_prism(l, w, h, r) {
    linear_extrude(height = h)
        rounded_rect_2d(l, w, r);
}

module screw_boss(h, outer_d, hole_d) {
    difference() {
        cylinder(h = h, d = outer_d);
        translate([0, 0, -eps])
            cylinder(h = h + 2 * eps, d = hole_d);
    }
}

module dome_mount_holes(h) {
    for (a = [45, 135, 225, 315]) {
        translate([
            dome_mount_radius * cos(a),
            dome_mount_radius * sin(a),
            -eps
        ])
            cylinder(h = h + 2 * eps, d = dome_mount_hole_d);
    }
}

module top_cover_mount_holes(h) {
    for (x = [-top_cover_mount_spacing_x / 2, top_cover_mount_spacing_x / 2])
        for (y = [-top_cover_mount_spacing_y / 2, top_cover_mount_spacing_y / 2])
            translate([x, y, -eps])
                cylinder(h = h + 2 * eps, d = top_cover_mount_clearance_d);
}

module top_cover_post_clearance_holes(h) {
    for (x = [-top_cover_mount_spacing_x / 2, top_cover_mount_spacing_x / 2])
        for (y = [-top_cover_mount_spacing_y / 2, top_cover_mount_spacing_y / 2])
            translate([x, y, -eps])
                cylinder(h = h + 2 * eps, d = top_cover_post_clearance_d);
}

module top_cover_post_rib_clearance(h) {
    for (sx = [-1, 1])
        for (sy = [-1, 1])
            translate([
                sx * (plate_length / 2 - top_cover_plate_corner_clearance / 2),
                sy * (plate_width / 2 - top_cover_plate_corner_clearance / 2),
                h / 2
            ])
                cube([
                    top_cover_plate_corner_clearance,
                    top_cover_plate_corner_clearance,
                    h + 2 * eps
                ], center = true);
}

module reinforced_top_cover_post(sx, sy) {
    x = sx * top_cover_mount_spacing_x / 2;
    y = sy * top_cover_mount_spacing_y / 2;
    h = box_height - floor_thickness;
    rib_x_len = inner_length / 2 - abs(x) + wall_thickness;
    rib_y_len = inner_width / 2 - abs(y) + wall_thickness;

    translate([x, y, floor_thickness])
        screw_boss(h, top_cover_post_outer_d, top_cover_post_pilot_d);

    translate([
        sx * (abs(x) + rib_x_len / 2),
        y,
        floor_thickness + h / 2
    ])
        cube([rib_x_len, top_cover_post_rib_thickness, h], center = true);

    translate([
        x,
        sy * (abs(y) + rib_y_len / 2),
        floor_thickness + h / 2
    ])
        cube([top_cover_post_rib_thickness, rib_y_len, h], center = true);
}

module wall_port_front(x, z, w, h) {
    translate([x, -box_width / 2, z])
        cube([w, wall_thickness * 4, h], center = true);
}

module wall_port_rear(x, z, w, h) {
    translate([x, box_width / 2, z])
        cube([w, wall_thickness * 4, h], center = true);
}

module wall_port_left(y, z, w, h) {
    translate([-box_length / 2, y, z])
        cube([wall_thickness * 4, w, h], center = true);
}

module wall_port_right(y, z, w, h) {
    translate([box_length / 2, y, z])
        cube([wall_thickness * 4, w, h], center = true);
}

module shell_lip_supports(z_center, support_h) {
    support_w = 6;
    side_len = inner_length - 2 * corner_radius;
    end_len = inner_width - 2 * corner_radius;

    translate([0, inner_width / 2 - support_w / 2, z_center])
        cube([side_len, support_w, support_h], center = true);
    translate([0, -inner_width / 2 + support_w / 2, z_center])
        cube([side_len, support_w, support_h], center = true);
    translate([inner_length / 2 - support_w / 2, 0, z_center])
        cube([support_w, end_len, support_h], center = true);
    translate([-inner_length / 2 + support_w / 2, 0, z_center])
        cube([support_w, end_len, support_h], center = true);
}

module bottom_shell() {
    mid_support_h = 2.4;
    top_support_h = 2.0;

    union() {
        difference() {
            rounded_prism(box_length, box_width, box_height, corner_radius);

            translate([0, 0, floor_thickness])
                rounded_prism(
                    inner_length,
                    inner_width,
                    box_height + eps,
                    max(corner_radius - wall_thickness, 1)
                );

            lower_port_z = 35;

            wall_port_front(front_usb_opening_x, lower_port_z, front_usb_opening_w, front_usb_opening_h);
            wall_port_front(front_power_opening_x, lower_port_z, front_power_opening_w, front_power_opening_h);

            for (i = [0 : rear_vent_slots - 1]) {
                x = (i - (rear_vent_slots - 1) / 2) * 12;
                wall_port_rear(x, lower_port_z, rear_vent_slot_w, rear_vent_slot_h);
            }
        }

        // Internal shelf for the removable separator plate.
        shell_lip_supports(
            floor_thickness + lower_inner_height - mid_support_h / 2,
            mid_support_h
        );

        // Reinforced corner posts let the split top cover screw down without a top inner lip.
        for (sx = [-1, 1])
            for (sy = [-1, 1])
                reinforced_top_cover_post(sx, sy);

        if (board_bosses_enabled) {
            for (x = [-board_mount_spacing_x / 2, board_mount_spacing_x / 2])
                for (y = [-board_mount_spacing_y / 2, board_mount_spacing_y / 2])
                    translate([x, y, floor_thickness])
                        screw_boss(board_boss_height, board_boss_outer_d, board_screw_hole_d);
        }
    }
}

module gimbal_standoff_holes(extra_h) {
    for (x = [-gimbal_standoff_spacing_x / 2, gimbal_standoff_spacing_x / 2])
        for (y = [-gimbal_standoff_spacing_y / 2, gimbal_standoff_spacing_y / 2])
            translate([x, y, -eps])
                cylinder(h = mid_plate_thickness + extra_h + 2 * eps, d = gimbal_standoff_hole_d);
}

module servo_relief_window(h) {
    ref_x = servo_window_ref_x_side * gimbal_standoff_spacing_x / 2;
    ref_y = servo_window_ref_y_side * gimbal_standoff_spacing_y / 2;
    dir_x = -servo_window_ref_x_side;
    dir_y = -servo_window_ref_y_side;
    dir_len = sqrt(dir_x * dir_x + dir_y * dir_y);
    unit_x = dir_x / dir_len;
    unit_y = dir_y / dir_len;
    center_x = ref_x + unit_x * (servo_window_gap_from_hole + servo_window_l / 2);
    center_y = ref_y + unit_y * (servo_window_gap_from_hole + servo_window_l / 2);
    angle_z = atan2(unit_y, unit_x);

    translate([center_x, center_y, h / 2])
        rotate([0, 0, angle_z])
            cube([servo_window_l, servo_window_w, h + 2 * eps], center = true);
}

module mid_plate() {
    difference() {
        rounded_prism(
            plate_length,
            plate_width,
            mid_plate_thickness,
            max(corner_radius - wall_thickness - plate_clearance, 1)
        );

        gimbal_standoff_holes(0);

        if (servo_window_enabled)
            servo_relief_window(mid_plate_thickness);

        top_cover_post_clearance_holes(mid_plate_thickness);
        top_cover_post_rib_clearance(mid_plate_thickness);

        // Finger notch at the rear edge.
        translate([0, plate_width / 2, -eps])
            cylinder(h = mid_plate_thickness + 2 * eps, r = finger_notch_radius);
    }
}

module top_cover_side(side = -1) {
    side_label_gap = split_gap / 2;
    half_center_x = side * (box_length / 4 + side_label_gap / 2);
    half_width = box_length / 2 - split_gap / 2;

    difference() {
        intersection() {
            rounded_prism(
                box_length - 2 * top_cover_clearance,
                box_width - 2 * top_cover_clearance,
                top_cover_thickness,
                max(corner_radius - top_cover_clearance, 1)
            );
            translate([half_center_x, 0, top_cover_thickness / 2])
                cube([half_width, box_width + 2, top_cover_thickness + 2], center = true);
        }

        translate([0, 0, -eps])
            cylinder(h = top_cover_thickness + 2 * eps, r = top_cutout_radius);

        dome_mount_holes(top_cover_thickness);

        top_cover_mount_holes(top_cover_thickness);

        // Half-moon finger notches on the split seam.
        translate([side * split_gap / 2, -box_width / 2 + 18, -eps])
            cylinder(h = top_cover_thickness + 2 * eps, r = 6);
        translate([side * split_gap / 2, box_width / 2 - 18, -eps])
            cylinder(h = top_cover_thickness + 2 * eps, r = 6);
    }
}

module dome_shell() {
    difference() {
        union() {
            difference() {
                intersection() {
                    sphere(r = dome_radius);
                    translate([
                        -dome_radius - dome_flange_width - 1,
                        -dome_radius - dome_flange_width - 1,
                        0
                    ])
                        cube([
                            2 * (dome_radius + dome_flange_width + 1),
                            2 * (dome_radius + dome_flange_width + 1),
                            dome_radius + 1
                        ]);
                }

                intersection() {
                    sphere(r = dome_radius - dome_wall_thickness);
                    translate([
                        -dome_radius - dome_flange_width - 1,
                        -dome_radius - dome_flange_width - 1,
                        -eps
                    ])
                        cube([
                            2 * (dome_radius + dome_flange_width + 1),
                            2 * (dome_radius + dome_flange_width + 1),
                            dome_radius + 1
                        ]);
                }
            }

            difference() {
                cylinder(h = dome_flange_height, r = dome_radius + dome_flange_width);
                translate([0, 0, -eps])
                    cylinder(
                        h = dome_flange_height + 2 * eps,
                        r = dome_radius - dome_flange_width
                    );
            }
        }

        dome_mount_holes(dome_flange_height);
    }
}

module gimbal_reference() {
    plate_z = floor_thickness + lower_inner_height + mid_plate_thickness;

    color([0, 0, 0, 0.22])
        translate([0, 0, plate_z])
            cylinder(h = upper_inner_height, r = reference_disc_radius);

    color([0.8, 0.55, 0.18, 0.65])
        for (x = [-gimbal_standoff_spacing_x / 2, gimbal_standoff_spacing_x / 2])
            for (y = [-gimbal_standoff_spacing_y / 2, gimbal_standoff_spacing_y / 2])
                translate([x, y, floor_thickness + lower_inner_height])
                    cylinder(h = upper_inner_height + mid_plate_thickness, d = gimbal_standoff_hole_d);
}

module assembly() {
    color("#f2f2f2") bottom_shell();

    color("#c9d5df")
        translate([0, 0, floor_thickness + lower_inner_height])
            mid_plate();

    color("#d8d8d8")
        translate([0, 0, box_height])
            top_cover_side(-1);
    color("#d8d8d8")
        translate([0, 0, box_height])
            top_cover_side(1);

    color([0.45, 0.8, 1.0, 0.28])
        translate([0, 0, box_height + top_cover_thickness])
            dome_shell();

    %gimbal_reference();
}

module dome_alignment_check() {
    color("#d8d8d8")
        top_cover_side(-1);
    color("#d8d8d8")
        top_cover_side(1);

    color([0.45, 0.8, 1.0, 0.32])
        translate([0, 0, top_cover_thickness])
            dome_shell();

    // Reference rods pass through the shared dome/cover hole centers.
    color("#ff3333")
        for (a = [45, 135, 225, 315]) {
            translate([
                dome_mount_radius * cos(a),
                dome_mount_radius * sin(a),
                -0.5
            ])
                cylinder(h = top_cover_thickness + dome_flange_height + 4, d = 2.2);
        }
}

module print_layout() {
    bottom_shell();

    translate([box_length + 20, 0, 0])
        mid_plate();

    translate([box_length + 20, box_width + 20, 0])
        top_cover_side(-1);

    translate([box_length + 20, -box_width - 20, 0])
        top_cover_side(1);

    translate([0, box_width + dome_radius + 40, 0])
        dome_shell();
}

if (part == "assembly") {
    assembly();
} else if (part == "bottom_shell") {
    bottom_shell();
} else if (part == "mid_plate") {
    mid_plate();
} else if (part == "top_left") {
    top_cover_side(-1);
} else if (part == "top_right") {
    top_cover_side(1);
} else if (part == "dome") {
    dome_shell();
} else if (part == "dome_alignment_check") {
    dome_alignment_check();
} else if (part == "print_layout") {
    print_layout();
}
