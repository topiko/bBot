include <design.scad>;
    

build = 0;

module el_mount(hole_coord, bolt_coord, h, bolt_sink, mode, a){
    r_up = due_d_bolt;
    if (mode==0){
        difference(){
            union(){
                for (point=hole_coord)
                    color("DarkGrey")
                    translate(point) difference() {
                        cylinder(h = h, r1 = h*sqrt(2) + r_up, r2 = r_up, $fn = a);
                        cylinder(h = h+1, r = due_d_bolt/2, $fn = a);
                    }
                }
            for (point=bolt_coord){
                color("Silver") translate(point) mirror([0,0,1])translate([0,0, -h-due_d_bolt/sqrt(3) - bolt_sink]) bolt(h, due_d_bolt/2, a);
            }
        }
    }
    else if (mode == 1)
        color("Silver") translate([-due_width/2, -18, h]) cube([due_width, due_length, 15]);
    else if (mode == 2)
        for (point=bolt_coord){
            color("Silver") translate(point) mirror([0,0,1])translate([0,0, -h-due_d_bolt/sqrt(3)- bolt_sink]) bolt(h, due_d_bolt/2, a);
        }

}

module servo_mount(){
    
    servo_h = 25;
    servo_w = 10;
    servo_L = 24;
    t_flange = 5.5;
    z_flange = 19;
    h_flange = 2.4;
    add_bottom = 5;
    module side_(){
        difference() {
            translate([t_flange, 0,0]) linear_extrude(height = servo_h + 4*add_bottom) polygon(points = [[0,0], [servo_w, 0], [0,servo_w], [-t_flange,servo_w], [-t_flange, 0]]);
            translate([0, 0, z_flange + add_bottom]) cube([t_flange, servo_w, h_flange]);
        }
    }
    
    //translate([servo_h/2, 0, 0]) 
    color("Silver") translate([add_bottom, 0 ,0]) rotate([90,0,-90]) side_();
    //translate([servo_h/2, servo_L, 0]) mirror([0,1,0]) rotate([90,0,-90]) side_();
    
    }


module hole_mount(mode, r, r_out){
    
    if (mode == 0){
        difference(){
            cylinder(r = r_out, h = wall_t + A_out, $fn = 5*a);
            cylinder(r = r + .1, h = wall_t + A_out, $fn = 5*a);
        }
    }
    else if (mode == 2){
        cylinder(r = r+.1, h = 3*wall_t + A_out, $fn = 5*a, center = true);
    }
    
}


module height_bar(){
    h = 16;
    r = 3.5;
    
    difference() {
        cylinder(h = h, r = r, $fa = 1, $fs = .2);
        cylinder(h = h, r = d3bolt_t/2, $fa = 1, $fs = .2);
    }
    
}

module rpi_mount(h, bolt_sink, mode, a){
    el_mount(hole_coord_rpi, bolt_coord_rpi, h, bolt_sink, mode, a);
}
module due_mount(h, bolt_sink, mode, a){
    el_mount(hole_coord_due, bolt_coord_due, h, bolt_sink, mode, a);
}
module stepper_mount(h, bolt_sink, mode, a){
    el_mount(hole_coord_step, bolt_coord_step, h, bolt_sink, mode, a);
}
module battery_mount(h, bolt_sink, mode, a){
    el_mount(hole_coord_battery, bolt_coord_battery, h, bolt_sink, mode, a);
}
module mpu_mount(h, bolt_sink, mode, a){
    el_mount(hole_coord_mpu, bolt_coord_mpu, h, bolt_sink, mode, a);
}
module dcdc_mount(h, bolt_sink, mode, a){
    el_mount(hole_coord_dcdc, bolt_coord_dcdc, h, bolt_sink, mode, a);
}



if (false){
    rpi_mount(4, 4, 2, 40);
    due_mount(4, 4, 2, 40);
}

//dcdc_mount(1, 4, 0, 40);
//stepper_mount(4, bolt_sink, 2, a);
height_bar();
//due_mount(1, 4, 2, 40);
//servo_mount();
//switch_mount(2);