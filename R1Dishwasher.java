package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//REMINDER -- Battery Level effects Motor Power which effects timing

@Autonomous

//@Disabled

public class R1Dishwasher extends LinearOpMode {

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


        //R1Dishwasher
        Bass_Pro_Shop.DriveSideways(0.5, 1600, 1);
        Thread.sleep(200);
        Bass_Pro_Shop.DriveStraight(0.5,1010,-1);

        //add thing to lift claw here
        Thread.sleep(200);
        Bass_Pro_Shop.clawOpen();
        Thread.sleep(200);
        Bass_Pro_Shop.clawClose();
        Thread.sleep(200);
        //lower claw

        Bass_Pro_Shop.clawOpen();
        Bass_Pro_Shop.StopMotion(1000);


    }
}