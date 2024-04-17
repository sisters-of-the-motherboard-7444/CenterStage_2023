package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//REMINDER -- Battery Level effects Motor Power which effects timing
//this auto program will move a preloaded cone to the substation

@Autonomous (name = "CrazyPlay")

//@Disabled

public class CrazyPlay extends LinearOpMode {

    HardwareMapTime Bass_Pro_Shop = new HardwareMapTime();

    @Override

    public void runOpMode() throws InterruptedException {

        System.out.println("Starting up");
        telemetry.addData("init pressed", "about to initialize");
        telemetry.update();

        System.out.println("Initialize Robot");
        Bass_Pro_Shop.InitializeRobot(hardwareMap);
        System.out.println("Robot Initialized");

        telemetry.addData("Status", "Ready!");

        telemetry.update();

        waitForStart();
        //start in same location as R2Ballister, but on the blue side
        Bass_Pro_Shop.DriveStraight(.5,300,1);
        Bass_Pro_Shop.CenterSpin(.8,145, -1);
        Bass_Pro_Shop.CenterSpin(.8,134, 1);
        Bass_Pro_Shop.DriveStraight(.5,50,-1);
        Bass_Pro_Shop.DriveStraight(.5,100,1);
        Bass_Pro_Shop.CenterSpin(.8,194, -1);
        Bass_Pro_Shop.CenterSpin(.8,450, 1);
        Bass_Pro_Shop.DriveStraight(.5,50,-1);
        Bass_Pro_Shop.CenterSpin(.8,330, -1);
        Bass_Pro_Shop.CenterSpin(.8,180, 1);

//park in backstage area
    }}