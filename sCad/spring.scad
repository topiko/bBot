a = 200;
r = 2;
t = .9;
wl = 4*r - t*2;

A = 10;
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


L = 30;
n = floor(L/wl);

translate([0,A/2, 0]) spring_unit(r,t,A, wl, H, n);
translate([-wl/2,A/4, 0]) spring_unit(r,t,A/2, wl, H, -.5);
translate([n*wl,A/4, 0]) spring_unit(r,t,A/2, wl, H, .5);

