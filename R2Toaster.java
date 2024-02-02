package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//REMINDER -- Battery Level effects Motor Power which effects timing

@Autonomous

//@Disabled

public class R2Toaster extends LinearOpMode {

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
        Bass_Pro_Shop.outputGround();
        Bass_Pro_Shop.DriveSideways(0.6, 1100, 1);
        Thread.sleep(400);
        Bass_Pro_Shop.CenterSpin(0.6,125,-1);
        Thread.sleep(400);
        Bass_Pro_Shop.DriveStraight(0.6,600,1);
        Thread.sleep(400);
        Bass_Pro_Shop.DriveSideways(0.6,950,1);
        Thread.sleep(400);
        Bass_Pro_Shop.CenterSpin(0.6,125,-1);
        Thread.sleep(400);
        Bass_Pro_Shop.DriveStraight(0.6,2100,-1);
        Thread.sleep(400);
        Bass_Pro_Shop.DriveSideways(0.6,1050,-1);
        Thread.sleep(200);
        Bass_Pro_Shop.CenterSpin(0.8,60,1);
        Thread.sleep(200);
        //add thing to lift claw here
        Bass_Pro_Shop.DriveStraight(0.8,60,-1);
        Thread.sleep(200);
        Bass_Pro_Shop.clawOpen();
        Thread.sleep(400);
        Bass_Pro_Shop.clawClose();
        //lower claw
        Bass_Pro_Shop.DriveSideways(0.8,1500,-1);
        Thread.sleep(200);
        Bass_Pro_Shop.DriveStraight(0.8,300,-1);
        Thread.sleep(400);
        Bass_Pro_Shop.clawOpen();

        Bass_Pro_Shop.StopMotion(1000);


    }
}