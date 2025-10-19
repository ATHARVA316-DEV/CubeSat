// VERA CubeSat Subsystem Parameters
housing_width = 100;
housing_length = 100;
housing_height = 40;
wall_thickness = 2;
exploded = true; // Set to true for exploded view

// Component dimensions
flywheel_diameter = 70;
flywheel_thickness = 10;
motor_diameter = 42;
motor_height = 20;
supercap_diameter = 16;
supercap_height = 35;
arduino_width = 18;
arduino_length = 45;
arduino_thickness = 1.6;

// Assembly
if (exploded) {
    exploded_assembly();
} else {
    assembled_system();
}

module assembled_system() {
    // Main housing
    main_housing();
    
    // Internal components
    translate([0, 0, wall_thickness]) {
        // Motor and flywheel at center
        translate([housing_width/2, housing_length/2, 0])
        motor_flywheel_assembly();
        
        // Supercapacitor bank
        translate([15, housing_length/2, 5])
        supercapacitor_bank();
        
        // Arduino board
        translate([housing_width-25, 15, 2])
        arduino_board();
        
        // IMU sensor
        translate([housing_width/2-10, housing_length/2-25, 2])
        imu_sensor();
        
        // Current sensor
        translate([housing_width/2+25, housing_length/2-10, 2])
        current_sensor();
        
        // Power conditioning PCB
        translate([20, 15, 2])
        power_pcb();
    }
}

module exploded_assembly() {
    // Housing
    main_housing();
    
    // Components with offset positions
    translate([0, 0, wall_thickness + 60]) {
        translate([housing_width/2, housing_length/2, 0])
        motor_flywheel_assembly();
    }
    
    translate([0, 0, wall_thickness + 20]) {
        translate([15, housing_length/2, 5])
        supercapacitor_bank();
        
        translate([housing_width-25, 15, 2])
        arduino_board();
        
        translate([housing_width/2-10, housing_length/2-25, 2])
        imu_sensor();
        
        translate([housing_width/2+25, housing_length/2-10, 2])
        current_sensor();
        
        translate([20, 15, 2])
        power_pcb();
    }
}

module main_housing() {
    difference() {
        // Main body
        cube([housing_width, housing_length, housing_height]);
        
        // Internal cavity
        translate([wall_thickness, wall_thickness, wall_thickness])
        cube([housing_width-2*wall_thickness, housing_length-2*wall_thickness, housing_height]);
        
        // Ventilation slots on sides
        for (i = [0:5]) {
            translate([-1, 20+i*10, housing_height/2])
            cube([wall_thickness+2, 6, 3]);
            
            translate([housing_width-wall_thickness-1, 20+i*10, housing_height/2])
            cube([wall_thickness+2, 6, 3]);
        }
        
        // M3 mounting holes at corners
        translate([5, 5, -1])
        cylinder(h=wall_thickness+2, r=1.5);
        
        translate([housing_width-5, 5, -1])
        cylinder(h=wall_thickness+2, r=1.5);
        
        translate([5, housing_length-5, -1])
        cylinder(h=wall_thickness+2, r=1.5);
        
        translate([housing_width-5, housing_length-5, -1])
        cylinder(h=wall_thickness+2, r=1.5);
        
        // USB port slot for Arduino
        translate([housing_width-1, 10, 8])
        cube([wall_thickness+2, 12, 6]);
    }
    
    // VERA label (raised text)
    translate([housing_width/2-15, housing_length-8, housing_height-0.5])
    linear_extrude(height=1)
    text("VERA", size=6, halign="center");
}

module motor_flywheel_assembly() {
    // BLDC Motor
    cylinder(h=motor_height, r=motor_diameter/2);
    
    // Motor shaft
    translate([0, 0, motor_height])
    cylinder(h=5, r=2);
    
    // Flywheel
    translate([0, 0, motor_height+3])
    cylinder(h=flywheel_thickness, r=flywheel_diameter/2);
    
    // Motor mounting screws
    for (angle = [0:90:270]) {
        rotate([0, 0, angle])
        translate([motor_diameter/2-3, 0, -2])
        cylinder(h=4, r=1.5);
    }
}

module supercapacitor_bank() {
    for (i = [0:2]) {
        translate([0, i*20, 0])
        cylinder(h=supercap_height, r=supercap_diameter/2);
    }
    
    // Connection PCB
    translate([-5, -5, supercap_height])
    cube([supercap_diameter+10, 50, 2]);
}

module arduino_board() {
    // PCB
    cube([arduino_width, arduino_length, arduino_thickness]);
    
    // Components on top
    translate([2, 2, arduino_thickness])
    cube([14, 8, 3]);
    
    translate([2, 15, arduino_thickness])
    cube([6, 12, 4]);
    
    // USB connector
    translate([arduino_width, 30, arduino_thickness])
    cube([4, 8, 3]);
    
    // Standoffs
    for (pos = [[2, 2], [2, arduino_length-2], [arduino_width-2, 2], [arduino_width-2, arduino_length-2]]) {
        translate([pos[0], pos[1], -3])
        cylinder(h=3, r=1);
    }
}

module imu_sensor() {
    // MPU6050 breakout board
    cube([20, 15, 1.6]);
    
    // Sensor IC
    translate([7.5, 5, 1.6])
    cube([5, 5, 1]);
    
    // Pin headers
    translate([0, 12, 1.6])
    cube([20, 2, 3]);
}

module current_sensor() {
    // ACS712 breakout board
    cube([25, 15, 1.6]);
    
    // Sensor IC
    translate([10, 7, 1.6])
    cube([8, 6, 2]);
    
    // Terminal blocks
    translate([2, 2, 1.6])
    cube([4, 4, 5]);
    
    translate([19, 2, 1.6])
    cube([4, 4, 5]);
}

module power_pcb() {
    // Power conditioning PCB
    cube([30, 20, 1.6]);
    
    // Components
    translate([5, 5, 1.6])
    cube([6, 4, 2]); // Buck-boost IC
    
    translate([15, 10, 1.6])
    cylinder(h=4, r=2); // Inductor
    
    translate([25, 5, 1.6])
    cube([3, 8, 3]); // Capacitors
    
    // Connectors
    translate([0, 15, 1.6])
    cube([8, 4, 4]);
}