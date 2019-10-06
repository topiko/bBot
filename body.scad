include <design.scad>;
use <rims.scad>;
use <elMount.scad>;
use <hand.scad>;
use <leg.scad>;


module shifted_due_mount(mode){
        translate([-wx/2 - A_out, 0, t]) rotate([90,0,90]) due_mount(el_mount_h, bolt_sink, mode, a*3);
}

module rein_up(mode){
    translate([axle_x, 0,0])motor_axle_reinforcement(hand_axle_h,-atan(w_rim/(nema14_x/2)),3*nema14_x, 12, 2, 0, mode);
}

module rein_low(mode){
    translate([axle_x, 0,0])motor_axle_reinforcement(nema14_x/2 + t,atan(w_rim/(nema14_x/2)),3*nema14_x, 12, 2, 1, mode);
}

module stepper_slide(mode){
    
    h = 22; 
    echo("Battery space = ", (hand_axle_h - nema14_x/2) - nema14_x - t - h);
    t_slid = 1.5;
    shift_x = (wx/2 - axle_x) - w_rim - 5 - t_slid/2;
    t_support = 2.75;
    
    module sups(x, z, H){
        //translate([(wx/2 - axle_x) - x - t_slid/2, 0 ,h])
       translate([(wx/2) - x - t_slid/2, 0 ,h]) {
            motor_axle_reinforcement(z,0,H, 3*t_slid + 2*t_support, t_support, mode, 0);
            mirror([0,1,0])motor_axle_reinforcement(z,0,H, 3*t_slid + 2*t_support, t_support, mode,0);
        }
    }
    module slid(add_t, h){
        n_h = 8;
        module holes(r_h, d_h){
            from_bottom = (h - d_h*(n_h-1))/2;
            for (i=[0:1:n_h-1]){
                translate([0,0,-h/2 + from_bottom + i*d_h]) rotate([0,90,0]) cylinder(r=r_h, h = 2*t_slid, $fn = 20, center = true);
                }
            translate([-t_slid/2, 0,0]) cube([t_slid, 2.5, (n_h-1)*d_h + 2], center = true);
                
         }
         
         w_controller = 13;
         hole_coord = [-29,-16,-14,-1,1,14,16,29];
            
         difference(){   
            cube([t_slid + 2*add_t, wy + 2*A_out - t_support, h], center = true);
            for (y=hole_coord) translate([0,y,0]) holes(.3, 2.54);   
            
        }
        
        
    }
    
    //x_ = [6,32,wx/2 - axle_x + nema14_x/2 + 1];
    x_ = [6,30,wx/2 - axle_x + nema14_x/2 + .2];
    if (mode == 0){ 
        sups(x_[0], nema14_x + t, h);
        sups(x_[1], nema14_x + t, h);
        sups(x_[2], H-h, H);

    }
    else if (mode == 1){ 
        translate([wx/2-x_[0]-t_slid/2, 0, nema14_x + t + h/2]) 
        slid(.1, h);
        translate([wx/2-x_[1]-t_slid/2, 0, nema14_x + t + h/2]) 
        slid(.1, h);
        translate([wx/2-x_[2]-t_slid/2, 0, H/2]) 
        slid(.1, H-2*t);
    }
    else if (mode == 2) slid(.1);
}


module motor_axle_reinforcement(z,alpha,L, w,t, up,mode){
    
    //w = 16;
    //t = 3;
    lip = .5;
    axle_x = 0;
    //up = 1;
    module rein(){
        module rein_(){
            rotate([0,0,90])linear_extrude(height = L) polygon(points = [[0, -w/2], [0, w/2], [-t, w/2 - t], [-t, -w/2 + t]]);
        }
        
        translate([axle_x, wy/2+A_out,z]) rotate([0, alpha,0])      if (up == 1) rein_();
        else if (up == 0) mirror([0,0,1]) rein_();
        
    }
    
    module cutout(){
        
        module cutout_(){
        rotate([0, 0,90])linear_extrude(height = L) polygon(points = [[-t - w_rim, -nema_r_axle], [-t-w_rim, nema_r_axle], [-t/2, nema_r_axle], [-t/2, nema_r_axle + lip], [0, nema_r_axle + lip], [0, -nema_r_axle - lip], [-t/2, -nema_r_axle - lip], [-t/2, -nema_r_axle]]);
        }
        translate([axle_x, wy/2+A_out,z]) rotate([0, alpha,0]) mirror([0,0,1]) 
          if (up == 1) cutout_();
        else if (up == 0) mirror([0,0,1]) cutout_();
       
        
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
        
        // BATTERY MOUNT
        /*
        module battery_mount_(){
            intersection(){
                shell(20, sc, sc2, mid, t, wx, wy, r_corner, ni, A_in, A_out, a);
                shifted_battery_mount(0);
            }
        }
        */
        // MOTOR MOUNTS:
        module hand_nema_mount() {
            intersection(){
                shell(20, sc, sc2, mid, t, wx, wy, r_corner, ni, A_in, A_out, a);    
                hand(wx_hand, wy_hand, r_corner_hand, sc_hand, sc2_hand, mid_hand, A_in, A_out, ni_hand, t, max(wx_hand, wy_hand), rot_hand, axle_x, hand_y, hand_z, t_motor_mount, hand_shift, t_motor_mount - bolt_wall_t, 1, 2, a);
            }
        }
        
        // MOTRO AXLE REINFORCEMENT:
        
        module stepper_slides_(){
            intersection(){
                shell(20, sc, sc2, mid, t, wx, wy, r_corner, ni, A_in, A_out, a);
                stepper_slide(0);
            }
        }
        
        // STEPPER SLIDE:
        module stepper_slide_(){
            intersection(){
                shell(20, sc, sc2, mid, t, wx, wy, r_corner, ni, A_in, A_out, a);
                stepper_slide(0);
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
            
            stepper_slide_();
            //battery_mount_();
            
            
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
            
            // Opening for motor AXle
            rein_up(1);
            rein_low(1);
            
            // STEPPER PLATE SLIDE
            stepper_slide(1);
        }
        
    }
    
    body_();
}

//a = 15;
body(0, 0);
//rein_up(0);
//rein_low(0);

//stepper_slide(1);
