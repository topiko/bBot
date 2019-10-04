use <wigleUnit.scad>;
include <design.scad>;





module bolts_(wx, wy, r_corner, w_rim, t, threads, a){
//    
    bolt_x_ = wx/2 - w_rim/2;
    bolt_y_ = wy/2 - w_rim/2;
    
    // Choose whwther to use 4 or 8 bolts for attachemnt of top cover
    nbolts = wx > 40 ? 8 : 4;
    
    r_ = r_corner - (r_corner - w_rim/2)/sqrt(2) - w_rim/2;

    middle_points = [for (xy=[[1,0], [0,1], [-1,0], [0,-1]]) [xy[0]*bolt_x_, xy[1]*bolt_y_]];
    corner_points = [for (i=[-1,1]) for (j=[-1,1]) [i*(bolt_x_ - r_), j*(bolt_y_ - r_)]];
    
    bolts8locs = concat(corner_points, middle_points);
    bolts4locs = corner_points; 
    
    bolt_points = nbolts == 8 ? bolts8locs : nbolts == 4 ? bolts4locs : 0;
    
    bolt_r = threads == 1 ? d3bolt_t/2 : d3bolt/2;
    
    for (point=bolt_points){
        translate(point) translate([0,0,-2*t])bolt(2*t, bolt_r, 3*a);
        }
    
}


module upRim(wx, wy, r_corner, sc, sc2, mid, w_rim, ni, A_out, t, heights_, mode, a){
    
    
    
    // top most rim: 
    module top_rim(add_, w){
        wigle_square(wx*sc2 + 2*add_, wy*sc2 + 2*add_, r_corner*sc2 + add_, theta(sc, sc2, ni,ni,mid, t, wx), 0, A_out, t, w, 0, a);
    }

    // second from top rim:
    module top_rim2(){
        sc_ = width_scale(sc,sc2, (ni-1)/ni, mid);
        wigle_square(wx*sc_, wy*sc_, r_corner*sc_, theta(sc, sc2, ni-1,ni,mid, t, wx), 0, A_out, t, w_rim, 0,a);
    }

    //upper rim 
    
    module full_upper_rim(){
    // for convenience:        

            // add top rim:
            top_rim(0, w_rim);
            // make slope so can be printed:
            difference(){
                // cut the slope from second from top rim:
                translate([0,0,-heights_[ni - 1]]) top_rim2();
                hull() {
                    // make smaller top rim -> mirror to get the bottom footprint of the actual rim:
                    mirror([0,0,1]) top_rim(-w_rim, 5);
                    // This is the second rim from top, again mirror to get the bottom footprint. Hull these tow and cut it from the second from top rim to get sloping rim.
                    translate([0,0,-heights_[ni - 1]]) mirror([0,0,1]) top_rim2();
                }
            }
                
    }
    
    
    h = add(select(heights_, [0:1:ni-1]));
    // move to top:
    
    translate([0,0,h])
    if (mode == 0){
        difference(){
            color("DarkSlateGray") full_upper_rim();    
            translate([0,0,t]) bolts_(wx, wy, r_corner, w_rim, t, 1, a);
        }
    }
    else if (mode==1){
        color("DarkSlateGray") full_upper_rim();    
        bolts_(wx, wy, r_corner, w_rim, t, 1, a);
    }
    else if (mode==2){
        bolts_(wx, wy, r_corner, w_rim, t, 1, a);
    }
}

// lower rim
module lowerRim(wx, wy, r_corner, sc, sc2, mid, w_rim, ni, A_out, t, bottom_thickness, mode, a){
    
    
    //bottom_thickness = 2;
    bolts_shift = bottom_thickness;
    color("DarkSlateGray")
    if (mode == 0){
        difference() {
            wigle_square(wx, wy, r_corner, theta(sc, sc2, 0,ni,mid, t, wx), 0, A_out, t, w_rim, 0, a);
            mirror([0,0,1]) translate([0,0,bolts_shift]) bolts_(wx, wy, r_corner, w_rim, t, 1, a);
        }
    }
    
    else if (mode == 1){
         wigle_square(wx, wy, r_corner, theta(sc, sc2, 0,ni,mid, t, wx), 0, 0, t, w_rim, 0, a);
        mirror([0,0,1]) translate([0,0,bolts_shift]) bolts_(wx, wy, r_corner, w_rim, t, 1, a);
    }
    else if (mode == 2){
        mirror([0,0,1]) translate([0,0,bolts_shift]) bolts_(wx, wy, r_corner, w_rim, t, 1, a);
    }
}




//wx, wy, r_corner, sc, sc2, mid, w_rim, ni, A_out, t, heights_, mode, a
//upRim(wx/2, wy, r_corner, sc, sc2, mid, w_rim, ni, A_out, t, heights_, 0, a);
//lowerRim(wx, wy, r_corner, sc, sc2, mid, w_rim, ni, A_out, t, 2, 0, a);

upRim(wx_head, wy_head, r_corner_head, sc_head, sc2_head, mid_head, w_rim_head, ni_head, A_out, t, heights_head, 0, a);

//shell(wall_t, sc, sc2, mid, t, wx, wy, r_corner, ni, A_in, A_out, a);