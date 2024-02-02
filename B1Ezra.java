package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;

//REMINDER -- Battery Level effects Motor Power which effects timing

@Autonomous

//@Disabled

public class B1Ezra extends LinearOpMode {

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

        Bass_Pro_Shop.DriveSideways(0.8, 200, 1);
        Bass_Pro_Shop.clawClose();
        Thread.sleep(1000);


        //Bass_Pro_Shop.DriveSideways(.5, 1800, -1);
        //Thread.sleep(1000);
        //Bass_Pro_Shop.DriveStraight(.5, 1000, -1);
        //Thread.sleep(1000);

        Bass_Pro_Shop.clawOpen();
        //Bass_Pro_Shop.updatePID();
        Thread.sleep(1000);
        Bass_Pro_Shop.outputLow();
        Thread.sleep(3000);
        Bass_Pro_Shop.clawClose();
        Thread.sleep(1000);
        Bass_Pro_Shop.StopMotion(100);

        /*
        Thread.sleep(1000);

        Bass_Pro_Shop.clawOpen();

        Thread.sleep(1000);

        Bass_Pro_Shop.clawClose();

        Thread.sleep(1000);

        Bass_Pro_Shop.moveArm(-22);

        Thread.sleep(1000);

        Bass_Pro_Shop.clawOpen();
        */


    }
}