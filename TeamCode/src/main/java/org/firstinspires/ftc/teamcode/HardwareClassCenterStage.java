
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import java.lang.Object;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;


/**
 * Original created by ashley.peake on 8/30/2018.
 * modified by KPSorrells (for teaching RoboLearners) 11/26/2022
 */

//@Disabled
public class HardwareClassCenterStage {

    // drivetrain motors
    public DcMotor motorFrontLeft;
    public DcMotor motorBackLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackRight;
    //public DcMotor slidesRight;
    //public DcMotor slidesLeft;
    public DcMotor armMotor;
    public Servo clawWrist;
    public Servo clawGripper;

    // lift motors
    public DcMotor slidesLeft;
    public DcMotor slidesRight;

    // servos or claw - CR = continuous Servo
    // public CRServo claw;
    public Servo claw = null;

    //Sets variable driveTime as an integer

    int driveTime;

//----------------------------Initialize Robot ---------------------------------
  /*  This method allows us to initialize the motors and sensors only once.
      It is called and used in every other program after "Init" is pressed.
   */

    public void InitializeRobot(HardwareMap hwMap) {

        HardwareMap HWMap = hwMap;

        //initialize Wheels
        motorFrontLeft = HWMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = HWMap.dcMotor.get("motorBackLeft");
        motorFrontRight = HWMap.dcMotor.get("motorFrontRight");
        motorBackRight = HWMap.dcMotor.get("motorBackRight");
        PIDslides leftController = new PIDslides(0.007, 0, 0.00);
        PIDslides rightController = new PIDslides(0.007, 0, 0.00);
        PIDArm armController = new PIDArm(0.007, 0, 0.00045);

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        //motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);//set for PracticeBot
        //motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE); //Set for PracticeBot
        //motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);//Competition Bot PowerPlay
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD); //Practice Bot
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD); //Competition Bot PowerPlay
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE); //Competition Bot & PracticeBot PowerPlay
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE); //Competition Bot & PracticeBot PowerPlay

        //Initialize Lift
        slidesLeft = HWMap.dcMotor.get("slidesLeft");
        slidesRight = HWMap.dcMotor.get("slidesRight");

        //Initialize Servos
        //claw = HWMap.crservo.get("claw");
        claw = HWMap.servo.get("claw");
        slidesLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int slidesTargetPosition = 0;
        int sGROUND_POSITION = 0;
        int sMID_POSITION = 1075;
        int sHIGH_POSITION = 2150;

        int armTargetPosition = 0;
        int aGROUND_POSITION = -30;
        int aLOW_POSITION = -1200;
        int aMID_POSITION = -1100;
        int aHIGH_POSITION = -90;
        int aHANG_POSITION = -800;
        int aPULL_POSITION = -200;

    }  //end of method InitializeRobot


//--------------------------Driving Pathways------------------------------------

  /*  This method allows the robot to drive forward based on encoder values.
      A distance is given that is converted to an encoder position in the code.
      leftDirection and rightDirection set the direction of the motors to allow
      the robot to either move straight ot turn.
   */

    public void DriveStraight(double power, double inches, int Direction) throws InterruptedException {

        //For driving forward or backward
        // declare variables for this method (power, totalSeconds (milliseconds) & Direction)
        //For forwards set direction = 1 (In method call)
        // For backwards set direction = -1 (In method call)
        //example: driveStraight(1, 5, 1) means drive straight at 100% power, for 5 seconds, in forward direction
        //example: driveStraight(1, 5, -1) means drive straight at 100% power, for 5 seconds, in backwards direction

        motorFrontLeft.setPower(power * Direction);
        motorBackLeft.setPower(power * Direction);
        motorFrontRight.setPower(power * Direction);
        motorBackRight.setPower(power * Direction);
        double miliseconds = distanceToSec(inches, power);
        Thread.sleep((long) miliseconds);

        //Stop Robot
        motorFrontLeft.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackRight.setPower(0.0);

    } //End DriveStraight Method

    public void DriveSideways(double power, double inches, int Direction) throws InterruptedException {

        //For strafing to the left or the right
        // declare variables for this method (power, totalSeconds (milliseconds) & Direction)
        //For right motion set direction = 1 (In method call)
        //For left motion set direction = -1 (In method call)
        //example: driveSideways(.5, 3, 1) means drive straight at 50% power, for 3 seconds, in right direction
        //example: driveSideways(.75, 5, -1) means drive straight at 75% power, for 5 seconds, in left direction

        motorFrontLeft.setPower(power * Direction);
        motorBackLeft.setPower(power * -Direction);
        motorFrontRight.setPower(power * -Direction);
        motorBackRight.setPower(power * Direction);
        double miliseconds = distanceToSec(inches, power);
        Thread.sleep((long) miliseconds);

        // stops all motion
        motorFrontLeft.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackRight.setPower(0.0);

    } //Ends DriveSideways Method

    public void DiagonalForward(double power, long inches, int Direction) throws InterruptedException {

        //For driving forward in a diagonal direction
        // declare variables for this method (power, totalSeconds (milliseconds) & Direction)
        //For right motion set direction = 1 (In method call)
        //For left motion set direction = -1 (In method call)
        //example: DiagonalForward(.8, 3, 1) means drive straight at 80% power, for 3 seconds, in forward right direction
        //example: DiagonalForward(.75, 5, -1) means drive straight at 75% power, for 5 seconds, in forward left direction

        if (Direction == 1) {

            motorFrontLeft.setPower(power * Direction);
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackRight.setPower(power * Direction);
            double miliseconds = distanceToSec(inches, power);
            Thread.sleep((long) miliseconds);
        }

        if (Direction == -1) {

            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(power * -Direction);
            motorFrontRight.setPower(power * -Direction);
            motorBackRight.setPower(0);
            double miliseconds = distanceToSec(inches, power);
            Thread.sleep((long) miliseconds);
        }

        // stops all motion

        motorFrontLeft.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackRight.setPower(0.0);

    } //End Diagonal Forward Method


    public void DiagonalBackward(double power, long inches, int Direction) throws InterruptedException {

        //For driving forward in a diagonal direction
        // declare variables for this method (power, totalSeconds (milliseconds) & Direction)
        //For right motion set direction = 1 (In method call)
        //For left motion set direction = -1 (In method call)
        //example: DiagonalBackward(.8, 3, 1) means drive straight at 80% power, for 3 seconds, in back Right direction
        //example: DiagonalForward(.75, 5, -1) means drive straight at 75% power, for 5 seconds, in left direction
        double miliseconds = distanceToSec(inches, power);
        if (Direction == 1) {

            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(power * -Direction);
            motorFrontRight.setPower(power * -Direction);
            motorBackRight.setPower(0);

            Thread.sleep((long) miliseconds);
        }

        if (Direction == -1) {

            motorFrontLeft.setPower(power * Direction);
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackRight.setPower(power * Direction);

            Thread.sleep((long) miliseconds);
        }

        // stops all motion

        motorFrontLeft.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackRight.setPower(0.0);

    } //End Diagonal Backward Method




    public void CenterSpin(double power, long miliseconds, int Direction) throws InterruptedException {

        //For turning robot on center
        // declare variables for this method (power, totalSeconds (milliseconds) & Direction)
        //For right motion set direction = 1 (In method call)
        //For left motion set direction = -1 (In method call)
        //180 degree spin at 100% power takes -----; at 75% takes ----; at 50% takes
        //90 degree spring at 100% power takes----; at 75% takes ----; at 50% takes
        //Check BATTERY Power

        //example: CenterSpin(1, 3, 1) means spin at 100% power, for 3 seconds, to the right

        if (Direction == 1) {

            motorFrontLeft.setPower(power * Direction);
            motorBackLeft.setPower(power * Direction);
            motorFrontRight.setPower(power * -Direction);
            motorBackRight.setPower(power * -Direction);

            Thread.sleep(miliseconds);
        }

        if (Direction == -1) {

            motorFrontLeft.setPower(power * Direction);
            motorBackLeft.setPower(power * Direction);
            motorFrontRight.setPower(power * -Direction);
            motorBackRight.setPower(power * -Direction);

            Thread.sleep(miliseconds);
        }

        // stops all motion

        motorFrontLeft.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackRight.setPower(0.0);

    } //End CenterSpin Method

    public void StopMotion()  {
        // stops all motion

        motorFrontLeft.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackRight.setPower(0.0);
    }
    public void moveLift(double power, long totalSeconds, int Direction) throws InterruptedException{
        slidesLeft.setPower(power * Direction);
        slidesRight.setPower(power * Direction);
        Thread.sleep(totalSeconds);

        //lift.setPower(0);
    }

    public void moveClaw(double power) throws InterruptedException{
        claw.setPosition(power);
        //Thread.sleep(totalSeconds);

        //slidesLeft.setPower(0);
        //slidesRight.setPower(0);

    }

    public double distanceToSec(double inches, double power){
        double miliseconds;
        double diameter = 3.7795; //
        double circumference = 3.14 * diameter;
        double revs = inches / circumference;
        double rpm = 327;
        double minToSec = 60;
        double secToMs = 1000;
        miliseconds = revs / (power * (rpm / minToSec) / secToMs);
        return miliseconds;
    }

}