// ============================================================
//  Popsicle Stick Connector Plates + Pancake Disc w/ Pillars
//  Units: mm
// ============================================================

// --- Popsicle Stick Dimensions ---
stick_length    = 152.4;   // 6 inches
stick_width     = 15.0;
stick_thickness = 4.0;
fat_stick_width = 24.0;

// --- M3 bolt hole (shaft clears, head stops) ---
bolt_hole_d = 3.4;

// --- Hole placement ---
hole_offset = stick_length / 2 - stick_width / 2;   // 68.7 mm
// Hole-to-hole distance = 2 * hole_offset = 137.4 mm

// --- Pillar Dimensions ---
pillar_diameter   = 16.0;
pillar_height     = 25.0;
pillar_radius     = hole_offset;   // 68.7 mm — keeps N↔S and E↔W = 137.4 mm

// --- M3 self-tapping bolt hole in pillar top ---
pillar_bolt_d     = 2.3;    // sweet spot: snug self-tapping fit in PLA/PETG
pillar_bolt_depth = 12.0;

// ============================================================
//  *** ADJUST PANCAKE SIZE HERE ***
//  Minimum sensible value: ~170 mm (just clears pillar outer edges)
//  Default comfortable value: 190 mm
//  Increase freely — only the outer ring gets bigger, nothing else changes.
pancake_diameter  = 200.0;   // mm — fully independent, edit this freely
// ============================================================

pancake_thickness = 4.0;
pancake_hole_r    = 1.75 * 25.4;   // 44.45 mm (1.75 inch radius inner hole)

// --- Layout spacing ---
row_gap = 5;

// ============================================================
//  Modules
// ============================================================

module stadium(len, w, t) {
    r = w / 2;
    hull() {
        translate([ len/2 - r, 0, 0])
            cylinder(h = t, r = r, center = true, $fn = 72);
        translate([-len/2 + r, 0, 0])
            cylinder(h = t, r = r, center = true, $fn = 72);
    }
}

module popsicle_stick(len, w, t, hd, hx) {
    difference() {
        stadium(len, w, t);
        for (sx = [1, -1])
            translate([sx * hx, 0, 0])
                cylinder(h = t + 1, r = hd / 2, center = true, $fn = 36);
    }
}

module pancake_with_pillars() {
    t  = pancake_thickness;
    od = pancake_diameter;
    hr = pancake_hole_r;
    pillar_top_z = t / 2 + pillar_height;

    difference() {
        union() {
            // ── Ring disc ──
            cylinder(h = t, d = od, center = true, $fn = 120);

            // ── 4 pillars at N / E / S / W ──
            for (angle = [0, 90, 180, 270]) {
                rotate([0, 0, angle])
                translate([pillar_radius, 0, t / 2])
                    cylinder(h = pillar_height, d = pillar_diameter,
                             center = false, $fn = 48);
            }
        }

        // ── Centre hole through ring ──
        cylinder(h = t + 1, r = hr, center = true, $fn = 120);

        // ── Self-tapping M3 blind holes from top of each pillar ──
        for (angle = [0, 90, 180, 270]) {
            rotate([0, 0, angle])
            translate([pillar_radius, 0, pillar_top_z - pillar_bolt_depth])
                cylinder(h = pillar_bolt_depth + 0.1,
                         d = pillar_bolt_d,
                         center = false, $fn = 24);
        }
    }
}

// ============================================================
//  Regular sticks  ×8
// ============================================================
for (i = [0 : 7]) {
    translate([0, i * (stick_width + row_gap), 0])
        popsicle_stick(stick_length, stick_width, stick_thickness,
                       bolt_hole_d, hole_offset);
}

// ============================================================
//  Fat oval sticks  ×2
// ============================================================
fat_y_start = 8 * (stick_width + row_gap) + fat_stick_width;

for (i = [0 : 1]) {
    translate([0, fat_y_start + i * (fat_stick_width + row_gap), 0])
        popsicle_stick(stick_length, fat_stick_width, stick_thickness,
                       bolt_hole_d, hole_offset);
}

// ============================================================
//  Pancake disc with pillars  ×1
// ============================================================
pancake_y = fat_y_start
          + 2 * (fat_stick_width + row_gap)
          + pancake_diameter / 2
          + row_gap;

translate([0, pancake_y, 0])
    pancake_with_pillars();