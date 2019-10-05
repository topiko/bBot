include <design.scad>;
use <rims.scad>;
use <elMount.scad>;
use <hand.scad>;
use <leg.scad>;


module shifted_due_mount(mode){
        translate([-wx/2 - A_out, 0, t]) rotate([90,0,90]) due_mount(el_mount_h, bolt_sink, mode, a*3);
}


module shifted_stepper_mount(mode){
        translate([wx/2 + A_out, 0, 0]) rotate([90,0,-90]) stepper_mount(el_mount_h, bolt_sink, mode, a*3);
}

module shifted_battery_mount(mode){
        translate([wx/2 + A_out, 0, 0]) rotate([90,0,-90]) battery_mount(el_mount_h, bolt_sink, mode, a*3);
}


module rein_up(mode){
    motor_axle_reinforcement(hand_axle_h,0,3*nema14_x/2 + 5, mode);
}

module rein_low(mode){
    motor_axle_reinforcement(nema14_x/2 + t,atan(w_rim/(nema14_x/2 + t)),3*nema14_x, mode);
}


module motor_axle_reinforcement(z,alpha,L, mode){
    
    w = 16;
    t = 3;
    lip = .5;
    
    module rein(){
        translate([axle_x, wy/2+A_out,z]) rotate([0, alpha,0])mirror([0,0,1]) rotate([0,0,90])linear_extrude(height = L) polygon(points = [[0, -w/2], [0, w/2], [-t, w/2 - t], [-t, -w/2 + t]]);
    }
    
    module cutout(){
        translate([axle_x, wy/2+A_out,z]) rotate([0, alpha,0]) mirror([0,0,1]) rotate([0, 0,90])linear_extrude(height = L) polygon(points = [[-t - w_rim, -nema_r_axle], [-t-w_rim, nema_r_axle], [-t/2, nema_r_axle], [-t/2, nema_r_axle + lip], [0, nema_r_axle + lip], [0, -nema_r_axle - lip], [-t/2, -nema_r_axle - lip], [-t/2, -nema_r_axle]]);
        }
    
    if (mode == 0) rein();
    else if (mode == 1) cutout();
    
    }
    
module body(show_shell, disp){
    
    
    module solid_(){
        // SHELL
        if (show_shell==2)
            difference(){
                shell(wall_t, sc, sc2, mid, t, wx, wy, r_corner, ni, A_in, A_out, a);
                translate([0,-500,0]) cube(1000);
            }
        else{
            shell(wall_t, sc, sc2, mid, t, wx, wy, r_corner, ni, A_in, A_out, a);
        }
        
        // SUPPORT RIMS
        upRim(wx, wy, r_corner, sc, sc2, mid, w_rim, ni, A_out, t, heights_, 0, a);
        lowerRim(wx, wy, r_corner, sc, sc2, mid, w_rim, ni, A_out, t, 2, 0, a);
        
        // ARDUINO MOUNT:
        module arduino_mount(){
            intersection(){
                shell(20, sc, sc2, mid, t, wx, wy, r_corner, ni, A_in, A_out, a);
                shifted_due_mount(0);
            }
        }
        
        // STEPPER DRIVER MOUNTS:
        module stepper_mount(){
            intersection(){
                shell(20, sc, sc2, mid, t, wx, wy, r_corner, ni, A_in, A_out, a);
                shifted_stepper_mount(0);
            }
        }
        
        module battery_mount(){
            intersection(){
                shell(20, sc, sc2, mid, t, wx, wy, r_corner, ni, A_in, A_out, a);
                shifted_battery_mount(0);
            }
        }
        
        // MOTOR MOUNTS:
        module hand_nema_mount() {
            intersection(){
                shell(20, sc, sc2, mid, t, wx, wy, r_corner, ni, A_in, A_out, a);    
                hand(wx_hand, wy_hand, r_corner_hand, sc_hand, sc2_hand, mid_hand, A_in, A_out, ni_hand, t, max(wx_hand, wy_hand), rot_hand, axle_x, hand_y, hand_z, t_motor_mount, hand_shift, t_motor_mount - bolt_wall_t, 1, 2, a);
            }
        }
        
        // MOTRO AXLE REINFORCEMENT:
        
        module rein_up_(){
            intersection(){
                shell(20, sc, sc2, mid, t, wx, wy, r_corner, ni, A_in, A_out, a);
                rein_up(0);
            }
        }
        
        module wheel_nema_mount_(){
            intersection(){
                shell(20, sc, sc2, mid, t, wx, wy, r_corner, ni, A_in, A_out, a);    
                noLeg(wx, wy, r_corner, sc, sc2, mid, A_out, ni, t, t_motor_mount, wall_t, axle_x, t_motor_mount - bolt_wall_t, 2, a);        
            }
        }
        
        if (disp==0){
            hand_nema_mount();
            mirror([0,1,0])hand_nema_mount();
            wheel_nema_mount_();
            mirror([0,1,0])wheel_nema_mount_();
            arduino_mount();
            //stepper_mount();
            battery_mount();
            rein_up_();
        }
    }
    
    // CUTOOUTS
    module body_(){
        
        difference(){
            color("DarkSlateGray") solid_();
            
            // HAND MOTORS
            hand(wx_hand, wy_hand, r_corner_hand, sc_hand, sc2_hand, mid_hand, A_in, A_out, ni_hand, t, max(wx_hand, wy_hand), rot_hand,axle_x, hand_y, hand_z, t_motor_mount, hand_shift, t_motor_mount - bolt_wall_t, 1, 1, a);
            mirror([0,1,0]) hand(wx_hand, wy_hand, r_corner_hand, sc_hand, sc2_hand, mid_hand, A_in, A_out, ni_hand, t, max(wx_hand, wy_hand), rot_hand, axle_x, hand_y, hand_z, t_motor_mount, hand_shift, t_motor_mount - bolt_wall_t, 1, 1, a);
            
            // RIM BOLTS
            upRim(wx, wy, r_corner, sc, sc2, mid, w_rim, ni, t, heights_, 2, a);
            lowerRim(wx, wy, r_corner, sc, sc2, mid, w_rim, ni, t, 1, 2, a);
            
            // LEGS
            noLeg(wx, wy, r_corner, sc, sc2, mid, A_out, ni, t, t_motor_mount, wall_t, axle_x, t_motor_mount - bolt_wall_t, 1, a);       
            mirror([0,1,0]) noLeg(wx, wy, r_corner, sc, sc2, mid, A_out, ni, t, t_motor_mount, wall_t, axle_x, t_motor_mount - bolt_wall_t, 1, a);
            
            // DUE MOUNT
            shifted_due_mount(2);
            
            // STEPPER DRIVER MOUNT
            shifted_stepper_mount(2);
            
            // BAttery MOUNT
            shifted_battery_mount(2);
            
            // Opening for motor AXle
            rein_up(1);
            rein_low(1);
        }
        
    }
    
    body_();
}

//a = 15;
body(0, 0);
//rein_up(0);
//rein_low(0);

