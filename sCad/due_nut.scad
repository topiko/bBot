a = 100;

r_bolt = 2.8/2;
h_bolt = 2.5;
h_plate = 2.25*h_bolt;
t_plate = r_bolt*1.5;
difference() {
    translate([0,0, h_plate/2]) cube([5*r_bolt, t_plate, h_plate], center = true);
    cylinder(r = r_bolt, h = h_bolt, $fn = a);
}