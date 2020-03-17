a = 50;
r = 2.8;
t = 1.2;
wl = 4*r - t*2;

A = 12;
H = 4;

module spring_unit(r,t,A, wl, H, n){
    
    module quarter_wedge_(r,t){
        difference(){
            circle(r = r, $fn = a);
            circle(r = r-t, $fn = a);
            polygon(points = [[0,0], [2*r, 0], [2*r, 2*r], [-2*r, 2*r], [-2*r, -2*r], [0,-2*r]]);
        }
    }

    
    module unit_(){
        translate([0, r - A/2, 0]) linear_extrude(height = H) {
            quarter_wedge_(r,t);
            translate([wl,0,0 ]) mirror([1,0,0])quarter_wedge_(r,t);
            translate([wl/2,A-2*r,0 ]) mirror([-1,1,0])quarter_wedge_(r,t);
            translate([wl/2,A-2*r,0 ]) mirror([0,1,0])quarter_wedge_(r,t);
            translate([wl/4 - t/2, 0, 0]) square([t, A - 2*r]);
            translate([wl/4*3 - t/2, 0, 0]) square([t, A - 2*r]);
        }
    }
    
    module full_(){
        for (i = [0:1:n-1]) translate([wl*i,0,0]) unit_();
    }
        
    if ((n!=-.5) && (n!=.5)) full_();
    else if (n == -.5) {
        intersection() {
            translate([wl*n, 0 , 0]) unit_();
            translate([0, -A, 0]) cube([wl, 2*A, 2*H]);
        }
    }
    else if (n == .5) {
        intersection() {
            translate([0, 0 , 0]) unit_();
            translate([0, -A, 0]) cube([wl*n, 2*A, 2*H]);
        }
    }
}


module screw_sup(r_out, h){
    difference(){
        cylinder(r = r_out, h = h);
        cylinder(r = r_screw, h = h);
    }
}

L = 34;
n = floor(L/wl);
A_servo = A/2;
A_horn = A/5*4;

module spring_full(){
    translate([0,A/2, 0]) spring_unit(r,t,A, wl, H, n);
    translate([-wl/2,A_horn/2, 0]) spring_unit(r,t,A_horn, wl, H, -.5);
    translate([n*wl,A_servo/2, 0]) spring_unit(r,t,A_servo, wl, H, .5);
}


r_screw = 1.3/2;
$fs = .2;
$fa = 6;
echo(n);
translate([(n+.5)*wl+r_screw, A_servo - t/2,0]) screw_sup(r_screw + t, H);

difference(){
    union(){
        spring_full();
        translate([-wl/2,A_horn,H/2]) rotate([90,0,0]) screw_sup(H/2, 2*t);
    }
    translate([-wl/2,A_horn,H/2]) rotate([90,0,0]) cylinder(r=r_screw+.1, h = 10, center = true);
}
