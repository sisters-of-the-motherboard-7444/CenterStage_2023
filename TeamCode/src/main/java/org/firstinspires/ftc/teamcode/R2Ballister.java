package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//REMINDER -- Battery Level effects Motor Power which effects timing
//this auto program will move a preloaded cone to the substation

@Autonomous (name = "Ballister")

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

        Bass_Pro_Shop.DriveStraight(.6, 1250, 1); //drive forward to read signal sheet things

        Thread.sleep(250);

        //scan april tag

        Bass_Pro_Shop.CenterSpin(.6,1000, 1); //spin to the right

        Thread.sleep(250);

        Bass_Pro_Shop.DriveStraight(.6,1250,1); //drive forward to black board

        Thread.sleep(250);

        //Insert code from April Tags for placement of hexagon

        Bass_Pro_Shop.CenterSpin(.6, 1000, -1); //spin to the left

        Thread.sleep(250);

        Bass_Pro_Shop.DriveStraight(.6, 750,1); //move up a wee bit

        Thread.sleep(250);

        Bass_Pro_Shop.CenterSpin(.6,1000, -1); //spin to face ze trap doors

        Thread.sleep(250);

        Bass_Pro_Shop.DriveStraight(.6, 3125, 1);//drive to other side of field

        Thread.sleep(250);




    }
}