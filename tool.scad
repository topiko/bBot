

spacing = .1;
d = 4 + spacing;
h = 8;
L = 1.2*h;

difference(){
    union(){
        cylinder(r = d, h = h, $fn = 6);
        cylinder(r = d, h = h/4, $fn = 100);
    }
    translate([0,0,1]) cylinder(r = d/sqrt(3), h = h, $fn = 6);
}

echo(d/sqrt(3));
translate([d/sqrt(3), -d/4, 0]) cube([L, d/2, h/2]);
