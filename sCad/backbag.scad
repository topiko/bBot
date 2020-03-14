include <design.scad>;
use <wigleUnit.scad>;
use <utils.scad>;
use <elMount.scad>;
use <covers.scad>;
use <rims.scad>;
use <neck.scad>;


module backbag_top_cover(move){
    
    //_backbag_bottom,
    w_rim_top = wall_t;
    r_cables = 1;
    width_cable_slot = 20;
    difference(){
    topCover(wx_backbag, wy_backbag, r_corner_backbag, sc_backbag, sc2_backbag, mid_backbag, A_out, w_rim_top,  ni_backbag, t, heights_backbag, thickness_bottom, bottom_rim_w, wall_t, move, a);
        translate([wx_backbag/2 - r_cables  - bottom_rim_w, -bottom_rim_w, 0]){
            hull(){
            translate([0, wy_backbag/2 - r_cables,0]) cylinder(r = r_cables, h = 100000, $fn = 4*a);
            translate([0, wy_backbag/2 - r_cables - width_cable_slot,0]) cylinder(r = r_cables, h = 100000, $fn = 4*a);
            }
        }
    }

}

//w_rim_low = 2;
backbag_bottom_rim_w = 1;
w_rim_backbag_bottom = 1;

module backbag_bottom_cover(){
    
    echo(backbag_bottom_rim_w);
    bottomCover(wx_backbag, wy_backbag, r_corner_backbag, sc_backbag, sc2_backbag, mid_backbag, A_out, w_rim_backbag_bottom, ni_backbag, t, heights_backbag, thickness_bottom, bottom_rim_w, wall_t, 1, a);
    
}

module backbag_old(){
    
    H_backbag = ni_backbag*t;
    
    module moved_converter_mount(mode){
        translate([-wx_backbag/2 - A_out, 54/2 - wy_backbag/2 + 3, t]) rotate([90, 0,90]) converter_mount(el_mount_h, bolt_sink, mode, 3*a);
    }
    
    switch_move_y = 2*switch_d;
    module moved_switch_mount(mode){
        translate([-wx_backbag/2 - A_out, wy_backbag/2 - switch_move_y, switch_d/2 + t/2]) rotate([90, 0,90]) hole_mount(mode, switch_d/2, r_out = 5);
    }
    
    module moved_led_mount(mode){
        translate([-wx_backbag/2 - A_out, wy_backbag/2 - switch_move_y, switch_d/2 + 2.5*t-1]) rotate([90, 0,90]) hole_mount(mode, 1.5, r_out = 3);
    }
    
    h_bat_sup = ni_backbag*t*.6;
    bat_sup_x = -wx_backbag/2 + (wx_backbag - bat_z) - 3;
    module backbag_shell_(){
        difference(){
        union() {
            shell(wall_t, 1, 1, .5, t, wx_backbag, wy_backbag, r_corner_backbag, ni_backbag, A_in, A_out, a);
            upRim(wx_backbag, wy_backbag, r_corner_backbag, sc_backbag, sc2_backbag, mid_backbag, wall_t, ni_backbag, A_out, t, heights_backbag, 0, a);
            lowerRim(wx_backbag, wy_backbag, r_corner_backbag, sc_backbag, sc2_backbag, mid_backbag, wall_t, ni_backbag, A_out, t, 2, 0, a);
            
        // Converter mount
        intersection(){
            moved_converter_mount(0);
            shell(el_mount_h, 1, 1, .5, t, wx_backbag, wy_backbag, r_corner_backbag, ni_backbag, A_in, A_out, a);
        }
        // battery support
        intersection(){
            translate([bat_sup_x,0,h_bat_sup/2]) cube([.33*4, wy_backbag, h_bat_sup], center = true);
            shell(10, 1, 1, .5, t, wx_backbag, wy_backbag, r_corner_backbag, ni_backbag, A_in, A_out, a);
        }
        translate([bat_sup_x,0,h_bat_sup/2]) cube([.33*4, wy_backbag, h_bat_sup], center = true);
        // switch mount
        moved_switch_mount(0);
        
        // led mount
        moved_led_mount(0);
        }
        moved_converter_mount(2);
        moved_switch_mount(2);
        moved_led_mount(2);
    }
        
        
        
        
        
    }
    
    // backbag SHELL
    //rotate([0,-90,0]){
    color("DarkSlateGray")backbag_shell_();
    translate([0,0,.01])backbag_bottom_cover();    
    //}
    
    
}

module backbag(){
    
    H_backbag = ni_backbag*t;
    
    module moved_converter_mount(mode){
        translate([-wx_backbag/2 - A_out, 54/2 - wy_backbag/2 + 3, t]) rotate([90, 0,90]) converter_mount(el_mount_h, bolt_sink, mode, 3*a);
    }
    
    
    module backbag_shell_(){
    
        union() {
            shell(wall_t, 1, 1, .5, t, wx_backbag, wy_backbag, r_corner_backbag, ni_backbag, A_in, A_out, a);
            
            
        
        }
        //moved_converter_mount(2);
        //moved_switch_mount(2);
        //moved_led_mount(2);
        R = 8;
        dx = 1.6;
        intersection(){
            shell(dx + wall_t, 1, 1, .5, t, wx_backbag + 2*dx, wy_backbag + 2*dx, r_corner_backbag, 4, A_in, A_out, a);
            hull(){
                rotate([0,50,0]){
                    
                translate([0, -2*R, 0]) cylinder(r = R, h = wy_backbag, $fn = 5*a);
                translate([0, 2*R, 0])cylinder(r = R, h = wy_backbag, $fn = 5*a);
                }
            }
        }
        
        upRim(wx_backbag, wy_backbag, r_corner_backbag, sc_backbag, sc2_backbag, mid_backbag, wall_t, ni_backbag, A_out, t, heights_backbag, 0, a);
        lowerRim(wx_backbag, wy_backbag, r_corner_backbag, sc_backbag, sc2_backbag, mid_backbag, wall_t, ni_backbag, A_out, t, 2, 0, a);
        
        
    }
    
    // backbag SHELL
    //rotate([0,-90,0]){
    color("DarkSlateGray")backbag_shell_();
    translate([0,0,.01])backbag_bottom_cover();    
    //}
    
    
}


module display_backbag(dx, H){
    
    /*
    translate([-wx/2 - thickness_bottom - A_out - dx, 0, H])
    rotate([0,-90,0])rotate([0,0,90]){
        backbag();
        backbag_top_cover(1);
    }*/
    // translate([+wx/2 + thickness_bottom + A_out + dx + wx_backbag/2, 0, H])
    translate([-wx/2 - thickness_bottom - A_out - dx - wx_backbag/2, 0, H])
    {
        backbag();
        backbag_top_cover(1);
    }
}


//display_backbag(1, 120);
a = 10;
backbag();
//backbag_bottom_cover();
//backbag_top_cover(1);

/*
backbag();
top = 0;


intersection() {
    translate([0,0,500.001])cube(1000, center = true);
    if (top == 0) backbag_bottom_cover();
    else if (top == 1) backbag_top_cover(0);
}


intersection() {
    translate([0,0,-500 - .001])cube(1000, center = true);
    if (top == 0) backbag_bottom_cover();
    else if (top == 1) backbag_top_cover(0);
}
*/


//backbag_bottom_cover();