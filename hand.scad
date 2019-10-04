use <wigleUnit.scad>;
use <utils.scad>;
use <nema14.scad>;
include <design.scad>;

module hand(wx_hand, wy_hand, r_corner_hand, sc_hand, sc2_hand, mid_hand, A_in, A_out, ni_hand, t, wall_t, rot_hand, axle_x, hand_y, hand_z, t_motor_mount, shift_w, bolt_sink, move, motors, a){
    
    heights_ = heights(sc_hand, sc2_hand, mid_hand, t, wx_hand, ni_hand);
    ss = width_scale(sc_hand,sc2_hand, (ni_hand+1)/ni_hand, mid_hand);
    motor_shift = t;
    axle_h = add(heights_) - motor_shift;
    

    module hand(){
        
        if (motors == 1){
        // hand shell
        translate([0, shift_w, -axle_h]){
        shell(wall_t, sc_hand, sc2_hand, mid_hand, t, wx_hand, wy_hand, r_corner_hand, ni_hand, A_in, A_out, a);
        // hand bottom
        mirror([0,0,1])wigle_square(wx_hand, wy_hand, r_corner_hand, -theta(sc_hand, sc2_hand, 1, ni_hand, mid_hand, t, wx_hand), A_in, A_out, t/3, wall_t, 1, a);
        // hand top
        translate([0,0,add(select(heights_, [0:1:ni_hand]))])wigle_square(ss*wx_hand, ss*wy_hand, ss*r_corner_hand, theta(sc_hand, sc2_hand, ni_hand, ni_hand, mid_hand, t, wx_hand), A_in, A_out, t/3, wall_t, 1, a);
        // shoulder
        //translate([0, 0, axle_h]) joint_();
        }
        }   
        translate([0,-nema14_z - wy_hand + t_motor_mount + A_out, 0])
        if (motors==1){
             rotate([-90,0,0]) union(){
                nema14(3*a);
                nema_bolts(t_motor_mount - bolt_sink, nema_bolt_d/2, 3*a);
                }
        }
        // motor mount:
        else if (motors==2) {
             rotate([-90,0,0]) nema14_mount(t_motor_mount, 3*a);
        }
        
        // bolts
        else if (motors==3) {
            rotate([-90,0,0]) nema_bolts(t_motor_mount - bolt_sink, nema_bolt_d/2, 3*a);
        }
        
    }
    
    
    color("DarkGrey")
    if (move==1)
        translate([axle_x, hand_y, hand_z]) rotate([rot_hand, 0, 0]) hand();   
    else 
        hand(); 
       
    
    //mirror([0,1,0]) hand(); 
}


hand(wx_hand, wy_hand, r_corner_hand, sc_hand, sc2_hand, mid_hand, A_in, A_out, ni_hand, t, max(wx_hand, wy_hand), rot_hand, axle_x, hand_y, hand_z, t_motor_mount, hand_shift, bolt_sink, 1, 3, a);