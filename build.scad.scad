include <design.scad>;

// BUILD:
if (build==1){

    // BODY:
    //body(0, 1);

    // LEGS
    noLeg(wx, wy, r_corner, sc, sc2, mid, A_out, ni, t, t_motor_mount, wall_t, axle_x, t_motor_mount - 2, 1, a);
    mirror([0,1,0])noLeg(wx, wy, r_corner, sc, sc2, mid, A_out, ni, t, t_motor_mount, wall_t, axle_x, t_motor_mount - 2, 1, a);

    // HANDS:
    hand(wx_hand, wy_hand, r_corner_hand, sc_hand, sc2_hand, mid_hand, A_in, A_out, ni_hand, t, max(wx_hand, wy_hand), rot_hand, axle_x, hand_y, hand_z, t_motor_mount, hand_shift, t_motor_mount - bolt_wall_t, 1,1, a);
    mirror([0,1,0]) hand(wx_hand, wy_hand, r_corner_hand, sc_hand, sc2_hand, mid_hand, A_in, A_out, ni_hand, t, max(wx_hand, wy_hand), rot_hand, axle_x, hand_y, hand_z, t_motor_mount, hand_shift, t_motor_mount - bolt_wall_t, 1,1, a);

    // TOP COVER
    top_cover_w_neck(wx, wy, r_corner, sc, sc2, mid, lid_spacing_scale, A_out, w_rim, ni, t, heights_, thickness_bottom, bottom_rim_w, wall_t, 1, a);

    // BOTTOM COVER
    bottomCover(wx, wy, r_corner, sc, sc2, mid, lid_spacing_scale, A_out, w_rim, ni, t, heights_, thickness_bottom, bottom_rim_w, wall_t, 1, a);

    //translate([head_x,0,thickness_bottom + add(heights_)]) show_head(25, 0);
    
    shifted_due_mount(2);
    shifted_stepper_mount(2);

}
