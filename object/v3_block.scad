/**
 * Robotics Payload Cube
 */

// --- Parameters ---
section = false;
techdraw = false;
size = 60;
wall_thickness = 0.8;
edge_radius = 2.5;

// Bulkhead Rib parameters
n_offset = size * 0.075;          
min_rib_depth = 1;        
rib_width = 3;        

// Perforation parameters
perf_hole_dia = 6.0;
perf_spacing = perf_hole_dia + 3; 
hole_area_radius = size*0.5;

$fn = 64;

if (techdraw) {
    projection(cut = false) section_wrapper();
} else {
    section_wrapper();
}

module section_wrapper() {
    if (section) {
        section_view(0.3 * size);
    } else {
        block();
    }
}

/**
 * Creates a sectional cut for internal inspection.
 * @param height_offset Z-offset for the cut plane.
 */
module section_view(height_offset) {
    difference() {
        block();
        translate([0, 0, height_offset + size/2])
            cube([size * 2, size * 2, size], center = true);
    }
}

/**
 * Main assembly logic.
 */
module block() {
    // The dimension of the internal void
    inner_dim = size - (wall_thickness * 2);
    // The radius of the internal corners
    inner_r = max(0.1, edge_radius - wall_thickness);

    difference() {
        union() {
            // Main shell
            difference() {
                rounded_cube(size, edge_radius);
                rounded_cube(inner_dim, inner_r);
            }
            
            // Annular Bulkheads
            // We use the exact inner_dim so they sit flush against the flat faces
            bulkhead_system(inner_dim, inner_r, n_offset, rib_width, min_rib_depth);
        }

        // Subtractions
        union() {
            perforated_divet_grid(size, hole_area_radius, perf_hole_dia, perf_spacing);
        }
    }
}

/**
 * Places vertical ribs
 * These rings wrap around the internal walls, top, and bottom.
 * * @param dim Internal dimension of the cube.
 * @param r Internal corner radius.
 * @param n Offset from the centre for the ring pairs.
 * @param w Width of the rib (thickness of the plate).
 * @param d Depth of the rib (controlled here by the sphere subtraction).
 */
module bulkhead_system(dim, r, n, w, d) {
    for (ax = [0, 1]) {
        rotate([0, 0, ax * 90]) {
            for (side = [-1, 1, -3, 3, -5, 5]) {
                translate([side * n, 0, 0])
                    vertical_annular_ring_sphere_diff(dim, r, w, d);
            }
        }
    }
}

/**
 * Creates a vertical ring using a sphere subtraction.
 * * @param dim Outer width/height of the ring.
 * @param r Corner radius matching the internal shell.
 * @param w Width of the rib plate.
 */
module vertical_annular_ring_sphere_diff(dim, r, w, d) {
    // Rotate the plate to stand vertically parallel to the Z-axis
    rotate([0, 90, 0]) {
        difference() {
            // The outer profile matching the internal shell
            rounded_cube_flat(dim, w, r);
            
            // Re-introducing the sphere subtraction as requested
            // The diameter is dim to ensure it touches the flat faces
            a = (dim * 0.5) - d;
            sphere(r = a);
        }
    }
}

/**
 * Helper: A rounded cube that is flat on the Z axis (a rounded plate).
 */
module rounded_cube_flat(dim, height, r) {
    side = dim - 2 * r;
    hull() {
        for (x = [-1, 1], y = [-1, 1]) {
            translate([x * side/2, y * side/2, 0])
                cylinder(h = height, r = r, center = true);
        }
    }
}

/**
 * Standard modules for the cube geometry and perforations.
 */
module perforated_divet_grid(s, limit_r, hole_d, pitch) {
    for (ax = [0:2]) {
        for (side = [-1, 1]) {
            rotate_v = (ax == 0) ? [0, 90, 0] : (ax == 1 ? [90, 0, 0] : [0, 0, 0]);
            rotate(rotate_v)
                translate([0, 0, side * (s / 2)])
                    grid_logic(limit_r, hole_d, pitch, s);
        }
    }
}

module grid_logic(limit_r, hole_d, pitch, s) {
    count = floor((limit_r * 2) / pitch);
    for (i = [-count:count]) {
        for (j = [-count:count]) {
            x = i * pitch; y = j * pitch;
            if (sqrt(x*x + y*y) < (limit_r - hole_d/2)) {
                translate([x, y, 0])
                    cylinder(h = s/2, d = hole_d, center = true);
            }
        }
    }
}

module rounded_cube(s, r) {
    side = s - 2 * r;
    hull() {
        for (x = [-1, 1], y = [-1, 1], z = [-1, 1]) {
            translate([x * side/2, y * side/2, z * side/2])
                sphere(r = r);
        }
    }
}


