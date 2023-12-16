package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//REMINDER -- Battery Level effects Motor Power which effects timing
//this auto program will move a preloaded cone to the substation

@Autonomous (name = "R2Ballister")

//@Disabled

public class R2Ballister extends LinearOpMode {

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

        Bass_Pro_Shop.DriveStraight(.6, 30, 1); //drive forward to read signal sheet things

        Thread.sleep(250);

        //Insert reading of the apriltag

        Bass_Pro_Shop.CenterSpin(.6,500,1);

        Thread.sleep(250);

        Bass_Pro_Shop.DriveStraight(.6,30,1); //drive to balck board

        Thread.sleep(250);

        //Insert April Tag Code to place hexagon

        Bass_Pro_Shop.CenterSpin(.6,500,-1); //spin to the left

        Thread.sleep(250);

        Bass_Pro_Shop.DriveStraight(.6,20,1);//wee bit forward

        Thread.sleep(250);

        Bass_Pro_Shop.CenterSpin(.6,500,-1);

        Thread.sleep(250);

        Bass_Pro_Shop.DriveStraight(.6,100,1);//drive to other side of field, assuming can fit under the railing

        Thread.sleep(250);

        // wiggle wiggle
        Bass_Pro_Shop.CenterSpin(.6,100,1);
        Bass_Pro_Shop.CenterSpin(.6,100,-1);
        Bass_Pro_Shop.CenterSpin(.6,100,1);
        Bass_Pro_Shop.CenterSpin(.6,100,-1);
        Bass_Pro_Shop.CenterSpin(.6,100,1);
        Bass_Pro_Shop.CenterSpin(.6,100,-1);

    }
}