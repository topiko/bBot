use <wigleUnit.scad>;
//include <design.scad>;


dt_utils = .3;
dTHETA_utils = 10;

function get_bolt_sink(boltD, add) = boltD/2 + add;

function add(v, i = 0, r = 0) = i < len(v) ? add(v, i + 1, r + v[i]) : r;

function select(vector,indices) = [ for (index = indices) vector[index] ];


function width_scale(sc,sc2, x, mid) = 1 + x*(pow(mid,3)*sc2 - pow(mid,3) + 3*mid - sc*(3*mid - 2)-2)/(mid*(pow(mid,2) - 2*mid + 1)) + pow(x,3)*(pow(mid,2)*sc2 - pow(mid,2) + 2*mid - sc*(2*mid-1) - 1)/(pow(mid,2)*(pow(mid,2) - 2*mid + 1)) + pow(x,2)*(-2*pow(mid,3)*sc2 + 2*pow(mid,3) -3*pow(mid,2) + sc*(3*pow(mid,2) - 1) + 1)/(pow(mid,2)*(pow(mid,2) - 2*mid + 1));


function theta(sc, sc2, i, ni, mid, t, wx) = asin((width_scale(sc,sc2, (i+1)/ni, mid) - width_scale(sc,sc2, i/ni, mid))*wx/(2*t));


function heights(sc, sc2, mid, t, w, ni) = [for (i=[0:1:ni]) t*cos(theta(sc, sc2, i, ni, mid, t, w))];


// shell:
module shell(wall_t, sc, sc2, mid, t, wx, wy, r_corner, ni, A_in, A_out, a){
    heights_ = heights(sc, sc2, mid, t, wx, ni);
    color("DarkSlateGray") 
    union(){
    for (i=[0:1:ni]){
        wx_ = width_scale(sc,sc2, i/ni, mid)*wx;
        wy_ = width_scale(sc,sc2, i/ni, mid)*wy;
        r_corner_ = width_scale(sc,sc2, i/ni, mid)*r_corner;
        
        theta_ = theta(sc, sc2, i, ni, mid, t, wx);
        h = add(select(heights_, [0:1:i-1]));
        translate([0,0,h]) wigle_square(wx_, wy, r_corner, theta_, A_in, A_out, t, wall_t, 0, a);
    }
    }
}

module bolt(L, r, a){
    h_ = 2*r/sqrt(3);
    r2_ = 2*r;
    a = 40;
    cylinder(r = r, h = L, $fn = a);
    translate([0,0,L])cylinder(r1 = r, r2 = r2_, h = h_, $fs = dt_utils/2, $fa = dTHETA_utils);
    translate([0,0,L+h_])cylinder(r1 = r2_, r2 = r2_, h = r2_, $fs = dt_utils/2, $fa = dTHETA_utils);
    }

module bolt_new(L, r, a){
    h_ = 2*r/sqrt(3);
    r2_ = 2*r;
    h_base = 2*r2_;
    
    translate([0,0,-L])cylinder(r = r, h = L, $fs = dt_utils, $fa = dTHETA_utils);
    cylinder(r1 = r, r2 = r2_, h = h_, $fs = dt_utils, $fa = dTHETA_utils);
    translate([0,0,h_])cylinder(r1 = r2_, r2 = r2_, h = h_base, $fs = dt_utils, $fa = dTHETA_utils);
}
    
    
bolt_new(20, 1.5, 20);