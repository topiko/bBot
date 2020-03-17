include <design.scad>;



a = 5;
fs = .5;
nCells = 6;
rCell = 18.2/2;
wallThickness = 1.15;
spacing = .11;
lipThickness = wallThickness/2;
hLip = 1;
thicknessCap = .5;
hBat = 65;
wLip = wallThickness/2;

rCell_spacing = rCell + spacing;
r1 = rCell_spacing;
r2 = 5;


function get_y2(r1,r2,x, t) = sqrt(pow(r2 + t + r1, 2) - pow(x, 2)); 

module 2d_layout(r1, r2, t, end, solid){
    
    module ring_(r1, r2, t, phi0, phi1){
        
        dphi = dTHETA; //(phi1 - phi0)/(5*a);

        points = [for (phi=[phi1:-dphi:phi0]) r1*[cos(phi), sin(phi)]];
        points2 = [for (phi=[phi0 + 180: dphi: 270])  (r2+t)*[cos(phi), sin(phi)] + [x, y2]];
        points3 = [for (phi=[270: -dphi: phi0 + 180]) r2*[cos(phi), sin(phi)] + [x, y2]];
        points4 = [for (phi=[phi0:dphi:phi1]) (r1+t)*[cos(phi), sin(phi)]];
        
        points_mod = solid == 1 ? [[0,0]] : points;
        points2_mod = solid == 1 ? [[x,0]] : concat(points2, [[x, y2 - r2 - t]]);
        points4_mod = end == 0 ? concat(points4, [[0, r1 + t]]) : concat(points4, [[-r1 - t, 0]]);
        
        polygon(points = concat(points_mod, points2_mod, points3, points4_mod));
    }

    
    module upside_(phi1){
        ring_(r1, r2, t, phi0, phi1);
    }

    
    x = rCell_spacing;
    y2 = get_y2(r1,r2,x, t); //sqrt(pow(r2 + t + r1, 2) - pow(x, 2)); 
    phi0 = floor(acos(x/(r1 + r2 + abs(t))));    
       
    if (end == 0){
        phi1 = 90;
        upside_(phi1);
        mirror([0,1,0]) upside_(phi1);
        mirror([1,0,0]) upside_(phi1);
        mirror([1,0,0]) mirror([0,1,0])upside_(phi1);
    }
    else if (end == 1){
        phi1 = 180;
        upside_(phi1);
        mirror([0,1,0])upside_(phi1);
    }
    
}

//2d_layout(r1, r2, 2, 0, 1);

module 2D_full(r1, r2, t, solid){

        2d_layout(r1, r2, t, 1, solid);
        for (i=[1:1:nCells-2]){
            translate([(rCell_spacing)*2*i, 0, 0]) 2d_layout(r1, r2, t, 0, solid);
        }
        translate([(rCell_spacing)*2*(nCells-1), 0, 0]) mirror([1,0,0])2d_layout(r1, r2, t, 1, solid);
        
}

module container_shell(H,solid){
    
    color("DarkSlateGray") 
    if (solid==1){
        linear_extrude(height = H) 2D_full(r1, r2, wallThickness, solid);
        
    }
    else if (solid == 0){
        
        lip = wLip + spacing;
        linear_extrude(height = H-hLip) 2D_full(r1, r2, wallThickness, solid);
        linear_extrude(height = H) 2D_full(r1, r2, wallThickness - lip, solid);
        cap(H, 0);
    
        //cap(wLip + spacing, 0);
    }
}



module cap(H, topBot){
    
    
    T = rCell*1.2;
    solid = 1;
    module cap_(){
        color("DarkGray") {
            translate([0,0,-thicknessCap]) linear_extrude(height = thicknessCap) 2D_full(r1 + wallThickness - T, r2, T, 1);
            linear_extrude(height = hLip) 2D_full(r1 + wallThickness - wLip, r2, wLip, 0);
        }
        
    }
    
    if (topBot == 0) cap_();
    else if (topBot == 1) translate([0,0,H]) mirror([0,0,1]) cap_(); 
    
}


holeZ = [3*r1, 9*r1];
holeY = 20;

module container(spacingBackCont, shiftZ, mode){
    
    module shifted_(solid){
    
        rotate([90,0,0]) rotate([0,0,90])translate([0,0,-H/2])container_shell(H,solid);
        

    }
    
    module attach_(){
        r_top = 4;
        hole_y = 20;
        hole_coord = [[0, -hole_y], [0, hole_y]];
        shift_y = get_y2(r1,r2,rCell_spacing, wallThickness);
        
        h_attach_cyl = holeY*2 + 10;
        color("DarkSlateGray")
        
        
        for (z=holeZ){
            translate([shift_y, 0, z]) rotate([90,0,0])
            
            cylinder(r = r2+wallThickness, h = h_attach_cyl, center = true, $fs = .1, $fa = dTHETA);
                
        }
    
            
        
        
    }
    
    
    h_mount = el_mount_h;
    r_bolt = 1.5;
    l_bolt = 4;
    r_nut = 5.8/2/sqrt(3)*2;
    
    shift_attach = h_mount + wallThickness; 
    shift_bolt = l_bolt - shift_attach;
    module attach_bolts_(mode){
        
        for (z=holeZ){
            for (y=[-holeY, holeY]){
                translate([xshift, y, z + shiftZ]) {
                    if (mode == 1){
                        rotate([0,-90,0]) translate([0,0,shift_bolt]) bolt_new(l_bolt, r_bolt, 20);
                        rotate([0,-90,0]) translate([0,0,shift_bolt + 2*r_bolt]) cylinder(r = 2*r_bolt, h = 30, $fs = .3, $fa = dTHETA);
                        rotate([0,-90,0]) translate([0,0,shift_bolt-l_bolt]) cylinder(h = 2.3, r = r_nut, $fa = 60);
                    }
                    else if (mode == 0){
                        translate([shift_attach, 0, 0]) rotate([0,-90,0]) attach_to_body_mount_();
                    }
                }
            }
        }
    }
    
    module attach_to_body_mount_(mode){
        
        
        difference(){
            cylinder(h = h_mount, r1 = r_nut + 1, r2 = r_bolt*5, $fs = .2);            //
            cylinder(h = 2.3, r = r_nut, $fa = 60);
        }
        
    }
    
    module cable_cut_(w, r){
        translate([-30,0,0])
        rotate([0,90,0]) 
        
        hull(){
                translate([0,w/2-r, 0]) cylinder(h = 20, r = r, $fs = .1, $fa = dTHETA);
                translate([0,-w/2+r, 0]) cylinder(h = 20, r = r, $fs = .1, $fa = dTHETA);
            }
        }
    
    module container_(){
        
        difference(){
            translate([-wx/2 - r1 - A_out - wallThickness - spacingBackCont, 0, shiftZ]) 
            
            union(){
                mirror([0,1,0])
                shifted_(0);
                attach_();
            }
            intersection(){
                shell(3*r2, sc, sc2, mid, t, wx, 2*wy, r_corner, ni, 0, A_out, t/dt);
                translate([xshift, -wy/2, 0]) cube([wx/2, wy, (ni+1)*t]);
            }
            // bolts:
            attach_bolts_(1);   
            // hole for cables
            translate([xshift,H/2-10/2,shiftZ + rCell_spacing]) cable_cut_(12, 1.5);
            translate([xshift,H/2-3,shiftZ + rCell_spacing*9]) cable_cut_(10, 2);
            
        }
        
    }
    
    
    if (mode == 0){
        attach_bolts_(0);
    }
    else if (mode == 1){
        attach_bolts_(1);
    }
    else {
        container_();
    }
    
    xshift = -wx/2- A_out - wallThickness;
    H = hBat + 5;
    
    //attach_bolts_(0);
    
}

dTHETA = 3;
//container(backBattSpace, hBattBag, 0);
//container(backBattSpace, hBattBag, 2);
//container(backBattSpace, hBattBag, 2);

cap(hBat, 1);


