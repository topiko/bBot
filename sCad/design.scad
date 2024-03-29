use <utils.scad>;

Pi = 3.141592653589793;
dt = .25;
dTHETA = 3;

A = 1;
A_in = A; //A;
A_out = A; //A;
wall_t = .66;
t = 4;

wx = 57.5; //echo(15 + 35 + 1.5 + 2 + 4);
wy = 69; //battery width 65mm + spacings
//wx = 58; //echo(15 + 35 + 1.5 + 2 + 4);
//wy = 77; //battery width 65mm + spacings

r_corner = wx/10;
H_ = 132; //124; //124; 150; //
ni = floor(H_/ t);
H = t*(ni+1);
sc = 1.;
sc2 = 1.;
mid = .65;
w_rim = 4;     
a = floor(t/dt);
echo("a = ", a);
echo("Height = ", H);


    
// BOTS:
d3bolt = 3;
d3bolt_t = 2.8;

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

// DUE dimensions:
due_width = 53.21;
due_length = 104.47;
due_hole_w = 48.26;
due_d_bolt = d3bolt;


move_up = due_length - 15.24 - 6.68 + 6;
x_hole_due  = due_hole_w/2;
hole_coord_due = [[-x_hole_due, move_up, 0], 
                  [x_hole_due, move_up-1.27, 0],
                  [x_hole_due, move_up-76.43, 0],  
                  [-x_hole_due + 5 + 27.94, move_up-76.43 + 24.13, 0],  
                  [-x_hole_due, move_up-82.55, 0],
                  [-x_hole_due + 5, move_up-82.55 + 30.48, 0]
                  ];
                  
bolt_coord_due = [[-x_hole_due, move_up, 0], 
                  //[x_hole_due, move_up-1.27, 0],
                  //[x_hole_due, move_up-76.43, 0],  
                  [-x_hole_due + 5 + 27.94, move_up-76.43 + 24.13, 0],  
                  [-x_hole_due, move_up-82.55, 0],
                  [-x_hole_due + 5, move_up-82.55 + 30.48, 0]];

// rpi dimensions:
rpi_width = 31;
rpi_length = 69;
rpi_hole_w = 23;
rpi_hole_l = 58;
rpi_d_bolt = 2.8;

x_hole_rpi  = rpi_hole_w/2;
hole_coord_rpi = [[-x_hole_rpi, 0, 0], [x_hole_rpi, 0, 0], [-x_hole_rpi, rpi_hole_l, 0],  [x_hole_rpi, rpi_hole_l, 0]];
bolt_coord_rpi = hole_coord_rpi;


// converter params:
hole_coord_dcdc = [[-35/2, 2.5, 0], [35/2, 19 + 2.5, 0]];
bolt_coord_dcdc = hole_coord_dcdc;
h_dcdc = 25;

// HAND PARAMS:
sc_hand = 1.;
sc2_hand = 1;
wx_hand = 14;
wy_hand = wx_hand;
r_corner_hand = wx_hand/8;
L_hand = 72;
ni_hand = ceil(L_hand/t);
mid_hand = .8;
rot_hand = 0;
//hand_axle_h = H - nema14_x/2 - 26;
//hand_axle_h = t + 3*nema14_x/2 + 27;
hand_axle_h = H - nema14_x/2 - 2*t; 
hand_z = hand_axle_h; // -ni_hand*t;
t_motor_mount = 4;
hand_shift = 6;
hand_y = wy/2 + wy_hand - 2*t_motor_mount;
axle_x = wx/2 - nema14_x/2 - 1.5;
bolt_wall_t = .7;


// Battery Params
bat_x = 55;
bat_y = 27;
bat_z = 30;
backBattSpace = .5;
hBattBag = 10*t;


echo("Top h = ", H - hand_axle_h - nema14_x/2);

// STEPPER CONTROLLERS
x_hole_stepper = wx/2 - r_corner;
stepper_z0 = t + nema14_x + t;
stepper_z1 = hand_axle_h - nema14_x/2 - t;
stepper_h = stepper_z1 - stepper_z0;


//hole_coord_step = [for (i=[-1,1]) for (j=[0,1]) [i*x_hole_stepper, stepper_z0 + j*stepper_h]];
//bolt_coord_step = hole_coord_step;
hole_coord_step = [];
bolt_coord_step = hole_coord_step;

// BATTERY MOUNT
x_hole_battery = 0;
battery_z0 = t + nema14_x - t;
battery_z1 = hand_axle_h - nema14_x/2 + t;
battery_h = battery_z1 - battery_z0;

hole_coord_battery = [for (j=[0,1]) [x_hole_battery, battery_z0 + j*battery_h]];
bolt_coord_battery = hole_coord_battery;



// MPU6050 GYRO
hole_x_mpu = 0;
hole_y_mpu = 8;
hole_coord_mpu = [for (j=[-1,1])[hole_x_mpu, j*hole_y_mpu]];
bolt_coord_mpu = hole_coord_mpu;

// LEG PARAMS
sc_leg = 1;
sc2_leg = 1.;
mid_leg = .6;
wx_leg = 22;
wy_leg = wx_leg;
r_corner_leg = 2;
ni_leg = 6;
wheel_angle = 0;
//leg_y = wy/2*sc + wy_leg - t_motor_mount - t_motor_mount;

// HEAD PARAMS:
spacing = 4; // distance between rpi and walls
wx_head = rpi_width + wall_t*2 + spacing;
wy_head = wx_head;
r_corner_head = wx_head/10;
ni_head = ceil(75/t);
head_x = -14; //-7

sc_head = 1;
sc2_head = 1;
mid_head = .5;
w_rim_head = w_rim;
heights_head = heights(sc_head, sc2_head, mid_head, t, wx_head, ni_head);

/*
// BACKBAG PARAMS:
wx_backbag = bat_y + 4*wall_t; // + w_rim*2; //wx; //rpi_width + wall_t*2 + spacing;
wy_backbag = bat_x + 4*wall_t; // + w_rim*2; // 54 + 25; //wy*1.1;
r_corner_backbag = wx_backbag/10;

ni_backbag = ceil((2*bat_z)/t);
backbag_x = -7;

sc_backbag = 1;
sc2_backbag = 1;
mid_backbag = .5;
w_rim_backbag = w_rim;
heights_backbag = heights(sc_backbag, sc2_backbag, mid_backbag, t, wx_backbag, ni_backbag);
backbag_H = H - ni_backbag*t - 2*t;
*/

// NECK PARAMS
neck_t = 18;
axle_wall_t = 8;
neck_axle_d = 3.15;
r_pivot_circle = neck_axle_d/2 + axle_wall_t;

neck_h = wx_head/2 + A_out + 4;
axle_d_from_wall = 1;
neck_width = 23.5;

neck_pivot_angle_min = -5;
neck_pivot_angle_max = 60;

// COVER PARAMS
thickness_bottom = 2;
bottom_rim_w = 1;
//lid_spacing_scale = .0;
lid_spacing = .3;
switch_d = 5;


// MISC PARAMS
heights_ = heights(sc, sc2, mid, t, wx, ni);
el_mount_h = 3.5;
bolt_sink = 1.2;
