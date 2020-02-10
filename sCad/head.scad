include <design.scad>;
use <wigleUnit.scad>;
use <utils.scad>;
use <elMount.scad>;
use <covers.scad>;
use <rims.scad>;
use <neck.scad>;


module head_top_cover(move){
    topCover(wx_head, wy_head, r_corner_head, sc_head, sc2_head, mid_head, A_out, w_rim_head, ni_head, t, heights_head, thickness_bottom, bottom_rim_w, wall_t, move, a);
}

neck_pivot_spacing = .25;
w_rim_low = (wx_head - neck_t)/2;

module head_bottom_cover(){
    
    pivot_center = neck_axle_d/2 -thickness_bottom + axle_d_from_wall;
    difference() {
        bottomCover(wx_head, wy_head, r_corner_head, sc_head, sc2_head, mid_head, A_out, w_rim_low, ni_head, t, heights_head, thickness_bottom, bottom_rim_w, wall_t, 1, a);
        // FOR PIVOTING
        // OPENING
        translate([0,0,pivot_center]) rotate([0,90,0]) cylinder(r = r_pivot_circle + neck_pivot_spacing, h = neck_t + 2*neck_pivot_spacing, $fn = 12*a, center = true);
        // PIVOT AXLE
        translate([0,0, pivot_center]) rotate([0,90,0]) cylinder(r = neck_axle_d/2 + .1, h = wx_head - 2, $fn = 6*a, center = true);
        //CABLE HOLE
        translate([0, r_pivot_circle, 0]) hull(){
            translate([neck_t/2 - 5 + neck_pivot_spacing, 0, -t]) cylinder(r=5, h = 3*t, $fn = 5*a);
            translate([-neck_t/2 + 5 - neck_pivot_spacing, 0, -t]) cylinder(r=5, h = 3*t, $fn = 5*a);
            }
            //cube([neck_t + neck_pivot_spacing, 5, 20], center = true);
        //cylinder(r = neck_t/2  + .5, h = 3*thickness_bottom, $fn = 6*a, center = true);
        
    }
    
    //translate([0,0,pivot_center]) rotate([0,90,0]) cylinder(r = r_pivot_circle, h = neck_t, $fn = 3*a, center = true);
}

module head(){
    
    pivot_center = neck_axle_d/2 -thickness_bottom + axle_d_from_wall;
    H_head = ni_head*t;
    
    module moved_rpi_mount(mode){
        translate([wx_head/2 + A_out, 0, t + 6]) rotate([90,0,-90]) rpi_mount(el_mount_h, bolt_sink, mode, 3*a);
    }
    
    module moved_servo_mount_(){
        translate([-wx_head/2 - A_out, wy_head/2 - 2, H_head - t - 24]) rotate([0,90,0]) rotate([0,0,90]) servo_mount();

        }
    
    module head_shell_(){
        difference(){
        union() {
            shell(wall_t, 1, 1, .5, t, wx_head, wy_head, r_corner_head, ni_head, A_in, A_out, a);
        intersection(){
            moved_rpi_mount(0);
            shell(el_mount_h, 1, 1, .5, t, wx_head, wy_head, r_corner_head, ni_head, A_in, A_out, a);
        }
        // servo mount:
        intersection(){
            moved_servo_mount_();
            shell(10, 1, 1, .5, t, wx_head, wy_head, r_corner_head, ni_head, A_in, A_out, a);
        }
        }
        moved_rpi_mount(2);
        }
        
        
        upRim(wx_head, wy_head, r_corner_head, sc_head, sc2_head, mid_head, w_rim_head, ni_head, A_out, t, heights_head, 0, a);
        lowerRim(wx_head, wy_head, r_corner_head, sc_head, sc2_head, mid_head, w_rim_low, ni_head, A_out, t , 2, 0, a);
        
    }
    
    // HEAD SHELL
    difference(){
        head_shell_();
        translate([0,0, pivot_center]) rotate([0,90,0]) cylinder(r = neck_axle_d/2 + .1, h = wx_head - 2, $fn = 6*a, center = true);
        translate([0,0, pivot_center]) rotate([0,90,0]) cylinder(r = r_pivot_circle + .3, h = neck_t + 1, $fn = 12*a, center = true);
        cube([neck_t + 1, r_pivot_circle*2 + 10, 3*t], center = true);
    }
    
    //moved_servo_mount_();
    
}

module show_head(angle, mode){
    translate([0,0,neck_h]) 
    rotate([90,0,0])
    union() {
    translate([-thickness_bottom + neck_axle_d/2 + axle_d_from_wall,0,0]) rotate([0,-90,-angle]) 
        union() {
            head();
            // TOP COVER
            head_top_cover(1);

            // BOTTOM COVER
            head_bottom_cover();
        }
        
    neck_pivot(neck_h, axle_d_from_wall, neck_pivot_angle_min, neck_pivot_angle_max, 0, [0,0,0], mode);
    
    }
    
}


show_head(15, 0);
a = 10;
//head();
//head_bottom_cover();
//head_top_cover(0);

/*
head();
top = 0;


intersection() {
    translate([0,0,500.001])cube(1000, center = true);
    if (top == 0) head_bottom_cover();
    else if (top == 1) head_top_cover(0);
}


intersection() {
    translate([0,0,-500 - .001])cube(1000, center = true);
    if (top == 0) head_bottom_cover();
    else if (top == 1) head_top_cover(0);
}
*/


//head_bottom_cover();