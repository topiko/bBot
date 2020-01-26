
include <design.scad>;



function get_fn(d, t) = floor(d/t);

module pivot(){
    
    difference() {
        cylinder(r = r_pivot_circle, h = neck_t, $fn = floor(2*Pi*r_pivot_circle/dt));
        cylinder(r = neck_axle_d/2, h = neck_t, $fn = 4*a);
    }
} 


module neck(neck_h, pivot_point, pivot_angle_min, pivot_angle_max, axle_d_from_wall, mode){
    
    // This amount of angle needs to be added to the contact angle due to the 
    // bottom thickness.
    add_angle = asin(axle_d_from_wall/r_pivot_circle);
    phi_up = pivot_angle_max + add_angle;
    phi_low = pivot_angle_min + add_angle;
    alpha_up = 90 - phi_up;
    alpha_low = 90 - phi_low;
    neck_contact_point_up = pivot_point + [r_pivot_circle*cos(alpha_up), r_pivot_circle*sin(alpha_up), 0];
    neck_contact_point_low = pivot_point + [r_pivot_circle*cos(-alpha_low), r_pivot_circle*sin(-alpha_low), 0];
    
    x1l = neck_contact_point_low[0];
    y1l = neck_contact_point_low[1];
    
        
    x1 = neck_contact_point_up[0];
    y1 = neck_contact_point_up[1];
    
    x0 = neck_width;
    a = (x0 - x1 + y1*tan(-alpha_up))/pow(y1,2);
    b = (-2*x0 + 2*x1 - y1*tan(-alpha_up))/y1;
    c = x0;
    
    function y2_(y) = [a*pow(y,2) + b*y + c, y];
    
    points = [for (y=[y1:-dt:0]) y2_(y), [x0, 0], [x1l,0], [x1l, y1l]];
    
    module bolts_(){
        for (x=[x1l+3, x0-8]){
            translate([x, -1, neck_t/2]) rotate([90,0,0])bolt_new(neck_h/2, 1.4, 20);
        }
    }
    
    difference(){
        linear_extrude(height = neck_t) polygon(points = points);
        bolts_();
    }
    
    if (mode == 1) bolts_();
}

module neck_pivot(neck_h, axle_d_from_wall, pivot_angle_min, pivot_angle_max, move, move_xyz, mode){    
    
    pivot_point = [-axle_d_from_wall, neck_h, 0];
    
    module neck_(){
        rotate([90,0,0])
        translate([0,0, -neck_t/2]) 
        union() {
            translate(pivot_point) pivot();
            neck(neck_h, pivot_point, pivot_angle_min, pivot_angle_max, axle_d_from_wall, mode);
        }
    }
    
    
    color("DarkGrey")   
    if (move==0){
        translate(-pivot_point) rotate([-90,0,0]) neck_();
    }
    else if (move == 0){
        translate(move_xyz) neck_();
    }
    
}


//neck_h = 20;
//axle_d_from_wall = axle_d/2 + 1;

neck_pivot(neck_h, axle_d_from_wall, neck_pivot_angle_min, neck_pivot_angle_max, 0, [0,0,0], 0);

