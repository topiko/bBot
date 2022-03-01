use <utils.scad>;

// NEMA
nema_spacing = .3;
nema14_x = 35.2 + nema_spacing;
nema14_y = 35.2 + nema_spacing;
nema14_z = 23.0 + nema_spacing;
corner_cut = 2  + nema_spacing;
nema_L_axle = 17;
nema_r_axle = 5/2  + nema_spacing;
nema_bolt_d = d3bolt;
nema_L_bolt = 5;

middle_cyl_r = 11  + nema_spacing;
middle_cyl_h = 2  + nema_spacing;


build = 0;

module nema14(){
    module corner(){
        linear_extrude(height = nema14_z) polygon(points = [[0,0],[nema14_x/2, 0], [nema14_x/2, nema14_y/2 - corner_cut], [nema14_x/2-corner_cut, nema14_y/2], [0, nema14_y/2]]);
    }
    
    color("grey"){
    union() {
        corner();
        mirror([1,0,0]) corner();
        mirror([1,1,0]) corner();
        mirror([0,1,0]) corner();
    }
    
    // TOP RING
    cylinder(r = middle_cyl_r, h = nema14_z +  middle_cyl_h);
    // AXLE
    cylinder(r = nema_r_axle, h = nema14_z +  nema_L_axle);
    }
}


bolt_dist = 26/2;
module nema_bolts(L, r, a){
    
    for (i=[-1,1]){
        for (j=[-1,1]){
            translate([i*bolt_dist, j*bolt_dist, 0]) bolt(L + nema14_z, r, a);
        }
    }
}

module nema14_mount(t, a){
    
    a = 40;    
    module base_plate(){
        s_ = 2*t/nema14_x + 1;
        translate([0,0,t])
        intersection(){
        translate([0,0, -1000/2])cube(1000, center = true);
        hull(){
            translate([0,0,-t])
            intersection(){
                translate([0,0, t-1000/2])cube(1000, center = true);
                nema14(a);
            }
            
            scale([s_,s_,1])intersection(){
                translate([0,0, t-1000/2])cube(1000, center = true);
                nema14(a);
            }
        }
        }
    }
    
    translate([0,0,nema14_z])
    difference(){
        base_plate();
        translate([0,0,-1]) nema_bolts(nema_L_bolt+1, d3bolt/2, 2*a);
        translate([0,0,-1]) cylinder(r = middle_cyl_r, h = middle_cyl_h+1, $fn = 2*a);
        translate([0,0,-1]) cylinder(r = nema_r_axle, h = nema14_z +  nema_L_axle, $fn = a);
    }
    
    nema_bolts(nema_L_bolt+1, d3bolt/2, 2*a);
    
}

//nema14();
//L_bolt = 5;
//r_bolt = 3/2;
//nema_bolts(L_bolt, r_bolt, 10);
nema14();
