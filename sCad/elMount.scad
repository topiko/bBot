include <design.scad>;
    

build = 0;

module el_mount(hole_coord, bolt_coord, h, bolt_sink, mode, a){
    r_up = due_d_bolt/2 + due_d_bolt/2;
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



if (false){
    rpi_mount(4, 4, 2, 40);
    due_mount(4, 4, 2, 40);
}
//stepper_mount(4, bolt_sink, 2, a);
due_mount(1, 4, 0, 40);