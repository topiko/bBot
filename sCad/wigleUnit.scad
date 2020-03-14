
include <utils.scad>;
include <design.scad>;

build = 0;


module wigle_cyl(theta, A_in, A_out, t, wall_t, R, phi0, phi1, cyl, a){
    

    rotate_extrude(angle = phi1 - phi0, $fa = dTHETA, $fs = dt) wigle(theta, A_in, A_out, t, wall_t, R, 0, cyl, a);
    }

module wigle_bar(theta, A_in, A_out, t, wall_t, L, solid, cyl, a){
    translate([0, -9*wall_t, 0]) rotate([90,0,90]) linear_extrude(height = L, $fn = 2)
        
    wigle(theta, A_in, A_out, t, wall_t, 10*wall_t, solid, cyl, a);
}

module wigle(theta, A_in, A_out, t, wall_t, R, solid, cyl, a){
    
    
    // Wigle
    function fx(i,a,A,t, phase) = A*pow(sin(phase + 180*i/a), 2); 
    function fy(i,a,t) = t*i/a;
    
    
    theta_in = solid == 1 ? 0 : theta;
    A_in_ = solid == 1 ? 0 : A_in;
    t_in = solid == 1 ? t*cos(theta) : t;
    
    // SINE CORNER:
    points_in = [for (i=[0:1:a]) [max(0, R + cos(theta_in)*fx(i,a,A_in_,t_in,theta_in/2) + sin(theta_in)*fy(a-i,a,t_in) - wall_t), -sin(theta_in)*fx(i,a,A_in_,t_in,theta_in/2) + cos(theta_in)*fy(a-i,a,t_in)]];
    points_out = [for (i=[0:1:a]) [max(0, R + cos(theta)*fx(i,a,A_out,t, 0) + sin(theta)*fy(i,a,t)), -sin(theta)*fx(i,a,A_out,t,0) + cos(theta)*fy(i,a,t)]];
    points_wigle = concat(points_in, points_out);
    
    
    // SPHERE CORNER:
    function fx_sphere(i,a,t) = t*cos(theta + (90-theta)*i/a) - t*cos(theta); 
    function fy_sphere(i,a,t) = t*sin(theta + (90-theta)*i/a) - t*sin(theta);
    function fy_straight(i,a,t) = i/a*t*(1-sin(theta)); 
    
    points_in_sphere = [for (i=[0:1:a]) [max(0, R - max(wall_t, t*cos(theta))), max(0, fy_straight(i,a,t))]];
    points_out_sphere = [for (i=[a:-1:0]) [max(0, R + fx_sphere(i,a,t)),max(0, fy_sphere(i,a,t))]];
    points_sphere = concat(points_in_sphere, points_out_sphere);
    

    // FULL SPHERE CORNER:
    r = wall_t*cos(theta)/2;
    hy = t - r;
    
    function fx_full_sphere(i,a, t, hy, wall_t, theta) = r*cos(theta + 180*i/a) - hy*sin(theta) + r*cos(-theta); 
    function fy_full_sphere(i,a, t, hy, wall_t, theta) = r*sin(theta + 180*i/a) + hy*cos(-theta);
    
    
    points_full_sphere = [for (i=[0:1:a]) [max(0, R - fx_full_sphere(i,a, t, hy, wall_t, theta)), fy_full_sphere(i,a, t, hy, wall_t, theta)], [R, 0], [R - wall_t, 0]];
    

    // SELECT:    
    points = cyl==0 ? points_wigle : cyl==1 ? points_sphere : cyl==2 ? points_full_sphere : 1;
    
    polygon(points);
    
}


module wigle_wigle_square(wx, wy, r, theta, A_in, A_out, t, wall_t, cyl, a){
      
    
    a1 = r*360/(50*Pi)/t*a;
    echo(a1);
    intersection(){
        wigle_square(wx, wy, r, theta, 0, 0, t, max(wx, wy), 0, a);
        union(){
            for (i=[0:1:max(wx,wy)/t+1]){
                rotate_extrude($fn = a1, $fs = .01) translate([(1+i)*t,0,0]) rotate([0,0,90]) wigle(0, 0, A_out, t, wall_t, wall_t, 0, cyl, a);
            }
        }
    }
}


module wigle_square(wx, wy, r, theta, A_in, A_out, t, wall_t, cyl, a){
    
    module corner(){
        // corner:
        translate([wx/2 - r, wy/2 - r, 0]) wigle_cyl(theta, A_in, A_out, t, wall_t, r, 0, 90, cyl, a);
        
        x_ = wy/2 - wx/2;
        points = [[0, 8*wy], [wy, x_ + wy], [0, wy/2 - wx/2]];
        //sides
        
        wall_t_y = min(wall_t, wy/2);
        solid = wy/2 < wall_t ? 1 : 0;
        intersection(){
            translate([0, wy/2 - wall_t_y, 0]) wigle_bar(theta, A_in, A_out, t, wall_t_y, wx/2 - r, solid, cyl, a);
            linear_extrude(height = t+1) polygon(points);
        }
        
        wall_t_x = min(wall_t, wx/2);
        solid = wx/2 < wall_t ? 1 : 0;
        difference(){
            translate([wx/2 - wall_t_x, 0, 0]) mirror([0,1,0])rotate([0,0,-90]) wigle_bar(theta, A_in, A_out, t, wall_t_x, wy/2 - r, solid, cyl, a);
            linear_extrude(height = t+1) polygon(points);
        }
    }
    
    
    union() {
        corner();
        mirror([1,0,0])corner();
        mirror([0,1,0])corner();
        mirror([1,0,0]) mirror([0,1,0])corner();
    }
}

