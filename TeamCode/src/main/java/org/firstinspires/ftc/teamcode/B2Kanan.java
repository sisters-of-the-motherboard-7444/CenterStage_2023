package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//REMINDER -- Battery Level effects Motor Power which effects timing

@Autonomous

//@Disabled

public class B2Kanan extends LinearOpMode {

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

        Bass_Pro_Shop.clawClose(); //close call around pixals

        Bass_Pro_Shop.DriveSideways(.5, 3000, -1); //drive to align with center

        Thread.sleep(800);

        Bass_Pro_Shop.CenterSpin(.5,1500,1); //turn right to correct driftmaxxing

        Thread.sleep(800);

        Bass_Pro_Shop.DriveStraight(.5, 3000, -1); //drive toward board

        Thread.sleep(1000);

        Bass_Pro_Shop.DriveSideways(.5,1500,1); //drive to align with board

        Bass_Pro_Shop.updatePID();

        Thread.sleep(1000);

        Bass_Pro_Shop.outputLow();

        Thread.sleep(3000);

        Bass_Pro_Shop.clawOpen(); //release pixals to score

        Thread.sleep(1000);

        Bass_Pro_Shop.clawClose(); //claw must close to drop down again

        Thread.sleep(1000);

        Bass_Pro_Shop.clawOpen(); //claw must open to begin Tele-Op

        Bass_Pro_Shop.StopMotion(100);





        /* score */
    }
}