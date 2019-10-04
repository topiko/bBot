include <design.scad>;
use <nema14.scad>;
use <wigleUnit.scad>;
use <wheel.scad>;


module noLeg(wx, wy, r_corner, sc, sc2, mid, A_out, ni, t, t_motor_mount, wall_t, axle_x, bolt_sink, motors, a){

    
    axle_h = t + nema14_x/2;
    
    // LEG:    
    module motor_and_wheel_(){
        // MOTOR MOUNTS
        translate([axle_x, -nema14_z + wy/2 - t_motor_mount + A_out, axle_h])   rotate([-90,0,0])
        union(){
        if (motors==1){
            union(){
                nema14(3*a);
                nema_bolts(t_motor_mount - bolt_sink, nema_bolt_d/2, 3*a);
                translate([0,0,nema14_z + 6]) wheel();
            }
        }
        // motor mount:
        else if (motors==2) {
             nema14_mount(t_motor_mount, 3*a);
        }
        
        // bolts
        else if (motors==3) {
            echo(a);
             nema_bolts(t_motor_mount - bolt_sink, nema_bolt_d/2, 3*a);
        }
        
        }
    }
    
    
    
    motor_and_wheel_();
        
}



//noLeg(wx, wy, r_corner, sc, sc2, mid, A_out, ni, t, t_motor_mount, wall_t, axle_x, t_motor_mount - 2, 1, a);
//noLeg(wx, wy, r_corner, sc, sc2, mid, A_out, ni, t, t_motor_mount, wall_t, axle_x, t_motor_mount - bolt_wall_t, 1, a);
noLeg(wx, wy, r_corner, sc, sc2, mid, A_out, ni, t, t_motor_mount, wall_t, axle_x, t_motor_mount - bolt_wall_t, 1, a);
