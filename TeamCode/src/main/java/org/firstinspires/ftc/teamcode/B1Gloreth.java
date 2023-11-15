package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//REMINDER -- Battery Level effects Motor Power which effects timing
//this auto program will move a preloaded cone to the substation

@Autonomous (name = "Ballister")

//@Disabled

public class B1Gloreth extends LinearOpMode {

    HardwareClassCenterStage Bass_Pro_Shop = new HardwareClassCenterStage();

    @Override

    //I think 1250s is about 1 square across & 500s to make a 90 degree turn

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

        Bass_Pro_Shop.DriveStraight(.5,1250,1); //Drive up to april tags

        Thread.sleep(250);

        //scan april tag

        Bass_Pro_Shop.DriveSideways(.5,1250,1); //Drive right

        Thread.sleep(250);

        Bass_Pro_Shop.DriveStraight(.5,1250,1);

        Thread.sleep(250);

        Bass_Pro_Shop.CenterSpin(.5,500, -1); //turn to the left

        Thread.sleep(250);

        Bass_Pro_Shop.DriveStraight(.5,5000,1);

        Thread.sleep(250);

        Bass_Pro_Shop.DriveSideways(.5,1250,-1); //drive to the left

        Thread.sleep(250);

        Bass_Pro_Shop.DriveStraight(.5,1250,1); //drive up to board things

        Thread.sleep(250);

        //place hexagons things on the board

        //make sure you park in the blue lines

          /* wiggle wiggle=dance?
        Bass_Pro_Shop.CenterSpin(.6,100,1);
        Bass_Pro.Shop.CenterSpin(.6,100,-1);
        Bass_Pro_Shop.CenterSpin(.6,100,1);
        Bass_Pro.Shop.CenterSpin(.6,100,-1);
        Bass_Pro_Shop.CenterSpin(.6,100,1);
        Bass_Pro.Shop.CenterSpin(.6,100,-1);

         */
    }
}

