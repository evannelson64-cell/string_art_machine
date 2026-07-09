include <BOSL2/std.scad>
include <BOSL2/gears.scad>

// ────────────────────────────────────────────────
//    VIEW & EXPORT CONTROL
// ────────────────────────────────────────────────
Show_Pinion        = true;
Show_Rack          = true;
Show_Cradle        = true;
Show_Slider        = true;
Show_Motor_Mount   = true;
Cradle_Translucent = true;

/* [Slider & Cradle Params] */
Slider_Belly_Width    = 30;
Slider_Stem_Width     = 15;
Top_Plate_Width       = 55;
Belly_Plate_Thickness = 8;
Outer_Wall_Thickness  = 8;
Floor_Height          = 8;
Tab_Thickness         = 6;

// Parametric Straw Mounts
straw_od              = 6;
straw_id              = 4; 
straw_overhang        = 12.7; // Exactly 0.5 inches of overhang past the edge
straw_base_length     = 30;   // The length of the tube that sits over the slider body
straw_h               = straw_base_length + straw_overhang; // Total dynamic length
straw_offset_y        = 11;   // 11mm from surface to center axis of straw

/* [Mechanical Controls] */
Side_Clearance      = 0.5;
Vertical_Clearance  = 0.5;
Length              = 100;
Slide_Travel        = 0;
Bolt_Gap_Adjustment = 6;
Manual_Rack_Z_Shift = 0;

/* [Gear & Motor Hardware] */
num_teeth_pinion  = 24;
module_size       = 1.3;
gear_thick        = 10;
slot_len_val      = 6;
motor_offset_target = 24;
pilot_diam        = 24;
rim_diam          = 28;
rim_depth         = 2;
Gear_Window_Length = 25;

/* [Hidden] */
$fn = 64;
c_alpha = Cradle_Translucent ? 0.4 : 1.0;
circ_pitch = module_size * PI;
Total_Rack_Teeth = ceil(Length / circ_pitch);
motor_plate_thickness = 5.08;
bolt_head_clearance   = 5.35;
shaft_diam_val = 8.128;
shaft_len_val  = 12.7;

pinion_y_center = Floor_Height - (pilot_diam / 2) - (slot_len_val / 2);
mount_x_pos = (gear_thick/2) + motor_offset_target;

c_width = Slider_Belly_Width + (Side_Clearance * 2) + (Outer_Wall_Thickness * 2);
bridge_anchor_x = (c_width / 2) - Bolt_Gap_Adjustment;

slider_bottom_y = Floor_Height + Vertical_Clearance;
tunnel_h        = Belly_Plate_Thickness + (Vertical_Clearance * 2);
top_plate_y     = Floor_Height + tunnel_h + Tab_Thickness + Vertical_Clearance;

// ────────────────────────────────────────────────
//    ROUNDING HELPERS
// ────────────────────────────────────────────────
R = 1.5;   // global outer edge radius
R_FN = 16; // $fn for rounding arcs (low = fast render)

module rbox(w, h, d, r=R) {
    rr = min(r, w/2-0.01, h/2-0.01, d/2-0.01);
    hull() {
        for (x=[-1,1], y=[-1,1], z=[-1,1])
            translate([x*(w/2-rr), y*(h/2-rr), z*(d/2-rr)])
                sphere(r=rr, $fn=R_FN);
    }
}

// ────────────────────────────────────────────────
//    GEAR & SHAFT
// ────────────────────────────────────────────────
module solid_pinion_with_shaft() {
    translate([0, pinion_y_center + (slot_len_val/2), 0])
    rotate([0, 90, 0]) {
        color("Lime", 1.0)
        spur_gear(mod=module_size, teeth=num_teeth_pinion,
                  thickness=gear_thick, shaft_diam=0, center=true);
        color("Silver", 1.0)
        translate([0, 0, (gear_thick/2) + (shaft_len_val/2)])
            cylinder(h=shaft_len_val, d=shaft_diam_val, center=true);
    }
}

// ────────────────────────────────────────────────
//    RACK
// ────────────────────────────────────────────────
module rack_only() {
    rack_w_match = gear_thick + 0.6 + (Side_Clearance * 2);
    color("Gold", 1.0)
    translate([0, 0, Manual_Rack_Z_Shift])
    translate([-rack_w_match/2, Floor_Height + 2.3, 0])
    rotate([0, -90, 90])
    translate([-rack_w_match/2, -5.6, 0])
    rack(mod=module_size, teeth=Total_Rack_Teeth,
         thickness=rack_w_match, height=8);
}

// ────────────────────────────────────────────────
//    MOTOR MOUNT PLATE + ARMS
// ────────────────────────────────────────────────
module motor_mount_geometry() {
    mount_size = 48;
    hole_dist  = 31;
    strut_w    = 8;
    arm_len    = mount_x_pos - bridge_anchor_x;
    plate_corner_r = 4;  

    // ── Motor mount plate (face plate) ──
    color("Silver", c_alpha)
    translate([mount_x_pos + (motor_plate_thickness/2), pinion_y_center, 0])
    rotate([0, 90, 0])
    difference() {
        hull() {
            for (x=[-1,1], y=[-1,1])
                translate([x*(mount_size/2 - plate_corner_r),
                           y*(mount_size/2 - plate_corner_r), 0])
                    cylinder(r=plate_corner_r, h=motor_plate_thickness,
                             center=true, $fn=R_FN*2);
        }
        hull() {
            translate([0,  slot_len_val/2, 0])
                cylinder(h=motor_plate_thickness+2, d=pilot_diam, center=true);
            translate([0, -slot_len_val/2, 0])
                cylinder(h=motor_plate_thickness+2, d=pilot_diam, center=true);
        }
        for (x=[-1,1], y=[-1,1]) {
            hull() {
                translate([x*hole_dist/2, (y*hole_dist/2) + slot_len_val/2, 0])
                    cylinder(h=motor_plate_thickness+2, d=3.4, center=true);
                translate([x*hole_dist/2, (y*hole_dist/2) - slot_len_val/2, 0])
                    cylinder(h=motor_plate_thickness+2, d=3.4, center=true);
            }
        }
    }

    // ── Connecting arms ──
    color("DimGray", c_alpha)
    difference() {
        union() {
            for (z_side=[-1,1])
                translate([bridge_anchor_x + (arm_len/2),
                           pinion_y_center,
                           z_side * (mount_size/2 - strut_w/2)])
                    cube([arm_len, mount_size/3, strut_w], center=true);

            for (y_side=[-1,1])
                translate([bridge_anchor_x + (arm_len/2),
                           pinion_y_center + (y_side * (mount_size/2 - strut_w/2)),
                           0])
                    cube([arm_len, strut_w, mount_size/3], center=true);
        }
        for (z_side=[-1,1], y_side=[-1,1]) {
            translate([bridge_anchor_x + arm_len/2,
                       pinion_y_center + (y_side * hole_dist/2),
                       z_side * hole_dist/2])
            rotate([0, 90, 0])
            hull() {
                translate([0,  slot_len_val/2, 0])
                    cylinder(h=arm_len+5, d=bolt_head_clearance, center=true);
                translate([0, -slot_len_val/2, 0])
                    cylinder(h=arm_len+5, d=bolt_head_clearance, center=true);
            }
        }
    }
}

// ────────────────────────────────────────────────
//    CRADLE
// ────────────────────────────────────────────────
module cradle() {
    total_w   = Slider_Belly_Width + (Side_Clearance*2) + (Outer_Wall_Thickness*2);
    total_h   = Floor_Height + tunnel_h + Tab_Thickness;
    mount_size = 48;
    hole_dist  = 31;
    support_thick = 8;

    color("DimGray", c_alpha)
    union() {
        difference() {
            translate([0, total_h/2, 0])
                rbox(total_w, total_h, Length);

            translate([0, Floor_Height + (tunnel_h/2), 0])
                cube([Slider_Belly_Width + (Side_Clearance*2), tunnel_h, Length+2], center=true);

            translate([0, total_h - (Tab_Thickness/2), 0])
                cube([Slider_Stem_Width + (Side_Clearance*2), Tab_Thickness+2, Length+2], center=true);

            translate([0, Floor_Height/2, 0])
                cube([total_w+2, Floor_Height+10, Gear_Window_Length], center=true);

            if (Show_Motor_Mount) {
                for (z_side=[-1,1], y_side=[-1,1]) {
                    translate([bridge_anchor_x,
                               pinion_y_center + (y_side * hole_dist/2),
                               z_side * hole_dist/2])
                    rotate([0, 90, 0])
                    hull() {
                        translate([0,  slot_len_val/2, 0])
                            cylinder(h=20, d=bolt_head_clearance, center=true);
                        translate([0, -slot_len_val/2, 0])
                            cylinder(h=20, d=bolt_head_clearance, center=true);
                    }
                }
            }
        }

        if (Show_Motor_Mount) {
            translate([bridge_anchor_x + (support_thick/2), pinion_y_center, 0])
            difference() {
                rbox(support_thick, mount_size, mount_size);

                rotate([0, 90, 0])
                hull() {
                    translate([0,  slot_len_val/2, 0])
                        cylinder(h=support_thick+2, d=pilot_diam, center=true);
                    translate([0, -slot_len_val/2, 0])
                        cylinder(h=support_thick+2, d=pilot_diam, center=true);
                }
                for (z_side=[-1,1], y_side=[-1,1]) {
                    translate([0, y_side * hole_dist/2, z_side * hole_dist/2])
                    rotate([0, 90, 0])
                    hull() {
                        translate([0,  slot_len_val/2, 0])
                            cylinder(h=support_thick+2, d=bolt_head_clearance, center=true);
                        translate([0, -slot_len_val/2, 0])
                            cylinder(h=support_thick+2, d=bolt_head_clearance, center=true);
                    }
                }
            }
        }
    }
}

// ────────────────────────────────────────────────
//    SLIDER
// ────────────────────────────────────────────────
module slider_only() {
    rack_cutout_w = gear_thick + 0.6 + (Side_Clearance*2);
    stem_start  = slider_bottom_y + Belly_Plate_Thickness - 0.05;
    stem_h_calc = top_plate_y - stem_start + 0.05;
    
    wall_thickness = (straw_od - straw_id) / 2;
    fin_overlap = wall_thickness * 0.5; 
    fin_height_val = straw_offset_y - (straw_od / 2) + fin_overlap;

    // DYNAMIC MATH: Positions the straw center so the tip extends exactly 'straw_overhang' out.
    straw_center_z = (Length / 2) + straw_overhang - (straw_h / 2);
    
    // The support fin only needs to cover the portion of the tube actually resting on the slider body
    fin_length_z = straw_base_length;
    fin_center_z = (Length / 2) - (fin_length_z / 2);

    color("Cyan", c_alpha)
    difference() {
        union() {
            // Belly plate — flat cube.
            translate([0, slider_bottom_y + (Belly_Plate_Thickness/2), 0])
                cube([Slider_Belly_Width, Belly_Plate_Thickness, Length], center=true);

            // Stem — flat cube
            translate([0, stem_start + (stem_h_calc/2), 0])
                cube([Slider_Stem_Width, stem_h_calc, Length], center=true);

            // Top plate — plain cube, top 4 edges rounded only.
            translate([0, top_plate_y + 3, 0])
            union() {
                cube([Top_Plate_Width, 6, Length], center=true);
                intersection() {
                    rbox(Top_Plate_Width, 6, Length);
                    translate([0, 3 - R, 0])
                        cube([Top_Plate_Width + 2, R * 2, Length + 2], center=true);
                }
            }
            
            // ─── HOLLOW STRAW MOUNT ───
            // Extended length tube dynamically positioned to overhang the edge by 0.5"
            translate([0, top_plate_y + 6 + straw_offset_y, straw_center_z]) {
                difference() {
                    cylinder(h = straw_h, d = straw_od, center = true, $fn = 32);
                    cylinder(h = straw_h + 2, d = straw_id, center = true, $fn = 32);
                }
            }
            
            // ─── THIN RECTANGULAR SUPPORT FIN ───
            // Runs from the edge of the slider inward, anchoring the tube solidly without floating in mid-air
            translate([0, top_plate_y + 6 + (fin_height_val / 2), fin_center_z])
                cube([2, fin_height_val, fin_length_z], center = true);
        }

        // Rack cutout through belly (internal — stays sharp)
        translate([0, Floor_Height + 3.0, 0])
            cube([rack_cutout_w, 6, Length+2], center=true);
    }
}

// ────────────────────────────────────────────────
//    FINAL ASSEMBLY
// ────────────────────────────────────────────────
if (Show_Cradle)      cradle();
if (Show_Motor_Mount) motor_mount_geometry();

translate([0, 0, Slide_Travel]) {
    if (Show_Slider) slider_only();
    if (Show_Rack)   rack_only();
}

if (Show_Pinion) solid_pinion_with_shaft();
