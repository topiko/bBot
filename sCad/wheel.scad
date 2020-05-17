include <design.scad>;

Pi = 3.141592653589793;

r_tube = 2;
t_rim = 3.5;
D_wheel = 91;
dist_tube = sqrt(pow(r_tube,2) - pow(t_rim/2,2));
echo(D_wheel);
echo(dist_tube);
R = D_wheel/2 - dist_tube - r_tube;
wRim = 20;
maxPhi = 270;
wall_t = 10;
t_hub = 14;

a = 50;
nema_axle_h = 14;
nema_axle_r = 2.54;
r0 = nema_axle_r - .5;

D = (R - wRim + wall_t)/r0;

n_spokes = 8;

function spiral(r0, D, x, phi) = (r0*exp(ln(D)/maxPhi*phi)-x)*[cos(phi),sin(phi)];
function reverse(list) = [for (i=[len(list)-1:-1:0]) list[i]];
function spiral_points(r0, D, maxPhi, x, a) = [for (i=[0:1:a]) spiral(r0, D, x, i/a*maxPhi)]; 

function spiral_polygon_points(r0, D, maxPhi, wall_t, a) = concat(spiral_points(r0, D, maxPhi, 0, a), reverse(spiral_points(r0, D, maxPhi, wall_t, a)));

function poly2(a,b,c, x) = [x, a*pow(x, 2) + b*x + c];


module spokes(){
    for (i=[0:1:n_spokes]){
        rotate([0,0,360/n_spokes*i]) polygon(points = spiral_polygon_points(r0, D, maxPhi, wall_t, a));    
    }
}

module nema_axle_(){
    difference(){
            cylinder(r = nema_axle_r, h = nema_axle_h, $fn = a);
            translate([-50, nema_axle_r - .5,0])cube(100);
    }
}

module hub(){
    circle(r=r0, $fn = a);
}

module rim(){
    difference() {
        circle(r=R, $fn = a);
        circle(r=R-wRim, $fn = 2*a);
    }
}


module wheel(){
    
    /*
    module spokes_rims_(){
        linear_extrude(height = max(t_hub, t_rim)){
            hub();
            rim();
            spokes();
        }
    }
    */
    
    module side_profile_(){
        denom = pow(R, 2) - 2*R*r0 + pow(r0, 2);
        a_ = (t_hub - t_rim)/denom;
        b_ = 2*R*(t_rim - t_hub)/denom;
        c_ = (pow(R, 2)*t_hub - r0*t_rim*(2*R - r0))/denom;
        p = concat([[r0,t_hub+1]], [for (x=[r0:R/a:R]) poly2(a_,b_,c_, x)], [[R+1, t_rim], [R+1, t_hub+1]]);
        rotate_extrude($fn = 2*a) polygon(points = p);
    }
    
    color("DarkGray")
    
    difference(){
        //spokes_rims_();
        cylinder(r = R, h = t_hub, $fs = dt/2, $fa = dTHETA);
        side_profile_();
        // Tyre
        rotate_extrude($fs = dt/2, $fa = dTHETA) translate([R + dist_tube,t_rim/2]) circle(r=r_tube, $fs = dt/2, $fa = dTHETA);
        // motor axle
        nema_axle_();
        // Fixing bolt
        fix_bolt();
    }
    
    //cylinder(r = 7, h = t_hub, $fa = dTHETA);
    module fix_bolt(){
        bolt_l = 6;
        H = t_hub/3*2; 
        translate([0, nema_axle_r + bolt_l - .5, H]) rotate([90,0,180]){
        bolt_new(bolt_l, 1.4, 20);
        translate([0,0,1.5]) cylinder(r=1.6*2, h = 20, $fs = dt/2, $fa=dTHETA);
            }
    }
    
    
    //color("Black") rotate_extrude($fs = dt/2, $fa = dTHETA) translate([R + dist_tube,t_rim/2]) circle(r=r_tube, $fs = dt/2, $fa = dTHETA);
    
    
    
}

/*
intersection(){
    wheel();
    cylinder(r=7, h = t_hub);
}*/
    
wheel();