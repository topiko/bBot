include <design.scad>;
use <rims.scad>;
use <elMount.scad>;
use <head.scad>;
use <wigleUnit.scad>;

build = 0;

// TOP COVER
module topCover(wx, wy, r_corner, sc, sc2, mid, A_out, w_rim, ni, t, heights_, height, bottom_rim_w, wall_t, move, a){
    
    sc_ = width_scale(sc, sc2, (ni+1)/ni, mid); // + lid_spacing_scale;
    theta_ = theta(sc, sc2, ni+1,ni,mid, height, wx);

    module top_plate(){
        
        wx = wx*sc_ + 2*lid_spacing;
        wy = wy*sc_ + 2*lid_spacing;
        r_corner = r_corner*sc_ + lid_spacing;
        
        /*wigle_square(wx*sc_, wy*sc_, r_corner*sc_, theta_, 0, 0, height, wx*10, 0,a);
        wigle_square(wx*sc_ + 2*bottom_rim_w, wy*sc_ + 2*bottom_rim_w, r_corner*sc_ + bottom_rim_w, theta_, 0, 0, height+bottom_rim_w, bottom_rim_w, 2,a);
        mirror([0,0,1]) wigle_square(wx*sc_ + 2*bottom_rim_w, wy*sc_ + 2*bottom_rim_w, r_corner*sc_ + bottom_rim_w, -theta_, 0, 0, bottom_rim_w, bottom_rim_w, 2,a);*/
        wigle_square(wx, wy, r_corner, theta_, 0, 0, height, wx*10, 0,a);
        module tmprim(h, theta_){
            wigle_square(wx + 2*bottom_rim_w, wy + 2*bottom_rim_w, r_corner + bottom_rim_w, theta_, 0, 0, h+bottom_rim_w, bottom_rim_w, 2,a);
        }
        tmprim(height, theta_);
        mirror([0,0,1]) tmprim(0, -theta_);;
    }
    
    shift = move == 1 ? add(heights_) : 0;
    color("DarkGrey")
    difference(){
        translate([0,0, shift]) top_plate();
        upRim(wx, wy, r_corner, sc, sc2, mid, w_rim, ni, t, heights_, 2, a);
        translate([0,0, shift + height - get_bolt_sink(3, .2)]) bolts_(wx, wy, r_corner, w_rim, t, 0, a);
    }
}


module top_cover_w_neck(wx, wy, r_corner, sc, sc2, mid, A_out, w_rim, ni, t, heights_, thickness_bottom, bottom_rim_w, wall_t, mode, a){
    difference(){
        topCover(wx, wy, r_corner, sc, sc2, mid, A_out, w_rim, ni, t, heights_, thickness_bottom, bottom_rim_w, wall_t, 1, a);
        // Necj bolts:
        translate([head_x,0, thickness_bottom + add(heights_)]) show_head(25, 1);
        // cable hole:
        translate([3 + neck_width + head_x, 0, thickness_bottom + add(heights_)])
        intersection() {
           cylinder(r = neck_t/2, h = 20, center = true, $fn = 40);
           translate([10,0,0]) cube(20, center = true);
        }
        // switch hole
        translate([wx/2 - switch_d/2 - w_rim - .5, wy/2  - switch_d/2 - w_rim - 5,add(heights_) - 1]) cylinder(r = switch_d/2, h = 2*thickness_bottom, $fn = 40);
    }
    if (mode==2) translate([head_x,0, thickness_bottom + add(heights_)]) show_head(60, mode);


    
}

module bottom_cover_w_mpu(wx, wy, r_corner, sc, sc2, mid, A_out, w_rim, ni, t, heights_, height, bottom_rim_w, wall_t, move, a){
    
    shift = move == 1 ? -height : 0;
    mpu_mount_t = 1;
    echo(height);
    difference() {
        bottomCover(wx, wy, r_corner, sc, sc2, mid, A_out, w_rim, ni, t, heights_, height, bottom_rim_w, wall_t, move, a);
        translate([axle_x,0,shift + height + .6]) cube([16,21,height*2], center = true);
        //translate([0,0,shift]) mpu_mount(mpu_mount_t, .2, 2, 40);
    }
    
    //translate([0,0,shift]) mpu_mount(el_mount_h, .5, 0, 40);
    //translate([0,0,shift]) mpu_mount(el_mount_h, 1, 2, 40);
    //translate([0,0, shift]) mpu_mount(el_mount_h, bolt_sink, 0, 30);
    //translate([0,0, shift]) mpu_mount(el_mount_h, bolt_sink, 2, 30);
    
}

module bottomCover(wx, wy, r_corner, sc, sc2, mid, A_out, w_rim, ni, t, heights_, height, bottom_rim_w, wall_t, move, a){
    
    sc_ = width_scale(sc, sc2, 0/ni, mid); // + lid_spacing_scale;
    theta_ = theta(sc, sc2, 0,ni,mid, height, wx);
        
    module bottom_plate(){
        
        wx = wx*sc_ + 2*lid_spacing;
        wy = wy*sc_ + 2*lid_spacing;
        r_corner = r_corner*sc_ + lid_spacing;
        
        /*wigle_square(wx*sc_, wy*sc_, r_corner*sc_, theta_, 0, 0, height, wx*10, 0,a);
        wigle_square(wx*sc_ + 2*bottom_rim_w, wy*sc_ + 2*bottom_rim_w, r_corner*sc_ + bottom_rim_w, theta_, 0, 0, height+bottom_rim_w, bottom_rim_w, 2,a);
        mirror([0,0,1])wigle_square(wx*sc_ + 2*bottom_rim_w, wy*sc_ + 2*bottom_rim_w, r_corner*sc_ + bottom_rim_w, -theta_, 0, 0, bottom_rim_w, bottom_rim_w, 2,a);*/
        wigle_square(wx, wy, r_corner, theta_, 0, 0, height, wx*10, 0,a);
        module tmprim(h, theta_){
            wigle_square(wx + 2*bottom_rim_w, wy + 2*bottom_rim_w, r_corner + bottom_rim_w, theta_, 0, 0, h+bottom_rim_w, bottom_rim_w, 2,a);
        }
        tmprim(height, theta_);
        mirror([0,0,1]) tmprim(0, -theta_);
        
    }
    
    
    shift = move == 1 ? -height*cos(theta_) : 0;
    color("DarkGrey")
    difference(){
        translate([0,0, shift]) bottom_plate();
        lowerRim(wx, wy, r_corner, sc, sc2, mid, w_rim, ni, t, height/2, 2, a);
        translate([0,0, shift + get_bolt_sink(3, .2)]) mirror([0,0,1]) bolts_(wx, wy, r_corner, w_rim, t, 0, a);
    }
    
}


a = 20;
dt = .02;
//top_cover_w_neck(wx, wy, r_corner, sc, sc2, mid, A_out, w_rim, ni, t, heights_, thickness_bottom, bottom_rim_w, wall_t, 0, a);

bottom_cover_w_mpu(wx, wy, r_corner, sc, sc2, mid, A_out, w_rim, ni, t, heights_, thickness_bottom, bottom_rim_w, wall_t, 1, a);

//bottomCover(wx, wy, r_corner, sc, sc2, mid, lid_spacing_scale, A_out, w_rim, ni, t, heights_, thickness_bottom, bottom_rim_w, wall_t, 1, a);