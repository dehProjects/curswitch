/* File: platform.scad
 * Platform to hold f103Ard board with two fethe subboards, and maybe more
 * Author: deh
 * Latest edit: 20190828
 */
 $fn = 30;
 
 thick = 4;
 bwid  = 63;
 
 bpbase = [138.2, bwid, thick]; // BP Motherboard base
 fbase  = [ 63.0, 46.5, thick]; // fethe subboard base
 sbase  = [ 93.0, 56.0, thick]; // simulator base
 
 // BP Motherboard hole locations
 holeb1 = [  2.9 , 42.0,  0];  
 holeb2 = [ 99.6,   6.0,  0];
 holeb3 = [100.4,  58.0,  0];
 
 // FETHE hole locations
 holef1 = [ 3.6, 42.9,  0];
 holef2 = [51.4,  2.5,  0];
 holef3 = [57.2,  6.8,  0];
 holef4 = [57.2, 29.6,  0];
 holef5 = [57.2, 43.0,  0];
 
 // FETHE board positionings
 transf1 = [138.2-126,bwid, 0];
 transf2 = [138.2-63, bwid, 0];
 
 // simulator hole locations
 holes1 = [ 4.9,   5.5, thick];
 holes2 = [ 4.9,  26.0, thick];
 holes3 = [ 4.0,  50.5, thick];
 holes4 = [ 86.0,  5.3, thick];
 holes5 = [ 86.0, 52.0, thick];
 
 
 
 // simulator module
 module sim(a)
 {
     translate(a)
     {
        difference()
        {
            union()
            {
                cube(sbase,false);
                translate([0,-13,0])
                  cube([93.0, 15.0, thick],false);
                post(holes1);
                post(holes2);
                post(holes3);
                post(holes4);
                post(holes5);
            }
            union()
            {
                translate([45,27,25])
                  cube([60,35,50],true);
                holebot([12,-6,0]);
                holebot([78,-6,0]);
                holebot([12,55-4,0]);
                holebot([78,55-4,0]);
            }
        }
    }       
 }
 
 // Bottom holes
 module holebot(a)
 {
     translate(a)
        cylinder (d=2.0,h=50,center=true);
 }
 
 //
 module post(a)
 {
     translate(a)
     {
         difference()
        {
            cylinder(d1=8.0, d2=5.5,h=(4+thick),center=false);
            
            translate([0,0,(4+thick)-3.5])
            cylinder(d1=1.0, d2=2.8,h=3.5,center=false);
        }
     }
     
     
 }
 
 // fethe subboard platform
 module fplat(a)
 {
     translate(a)
     {
        difference()
        {
            union()
            {
                cube(fbase,false);
                post(holef1);
                post(holef2);
                post(holef3);
                post(holef4);
                post(holef5);
            }
            union()
            {
                translate([30,20,25])
                cube([35,25,50],true);
                holebot([3,3,0]);
                holebot([45-3,3,0]);
                holebot([50,45-3,0]);
                holebot([12,45-3,0]);
            }
        }
    }
 }
 module base()
 
 {
     difference()
     {
        union()
        {
            cube(bpbase,false);
            post(holeb1);
            post(holeb2);
            post(holeb3);
        }
        union()
        {
            translate([70,32,25])
                cube([115,35,50],true);
            translate([52.5,22,25])
                cube([80,30,50],true);

        }
     }
 }
 
 module total()
 {
     difference()
     {
         union()
         {
             base();
            fplat(transf1);
            fplat(transf2);
//             translate([0,0,2])
//            sim([20,60+45,0]);
         }
         union()
         {
             holebot([3,3,0]);
             holebot([138.2-3,3,0]);
             holebot([138.2-3,60-3,0]);
             holebot([3,60-3,0]);
             holebot([32,99,0]);
             holebot([98,99,0]);
         }
     }
 }
total();
 translate([0,99.6+25,0])
    sim([27,0,0]);
 translate([35,99.6+8,0]) cube([80,5,1.5],false);
 
 