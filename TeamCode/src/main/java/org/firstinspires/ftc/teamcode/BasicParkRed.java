package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//REMINDER -- Battery Level effects Motor Power which effects timing
//this auto program will move a preloaded cone to the substation

@Autonomous (name = "BasicParkRed")

//@Disabled

public class BasicParkRed extends LinearOpMode {

    HardwareClassCenterStage Bass_Pro_Shop = new HardwareClassCenterStage();

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

        Bass_Pro_Shop.DriveSideways(.5,48,1);//sideways to the right

        Thread.sleep(250);

//park in backstage area
    }}