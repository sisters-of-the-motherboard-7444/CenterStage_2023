

package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;


public class HardwareMapTime {


    public DcMotor motorFrontLeft;
    public DcMotor motorBackLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackRight;


    public DcMotor armMotor;
    public Servo clawWrist;
    public Servo clawGripper;


    public DcMotor slidesLeft;
    public DcMotor slidesRight;


    public CRServo claw;
    int driveTime;
    int slideTargetPosition;
    int sGROUND_POSITION;
    int sMID_POSITION;
    int sHIGH_POSITION;


    int armTargetPosition;
    int aGROUND_POSITION;
    int aLOW_POSITION;
    int aMID_POSITION;
    int aHIGH_POSITION;
    int aHANG_POSITION;
    int aPULL_POSITION;


    double leftPower;
    double rightPower;
    double armPower;


    PIDslides leftController = new PIDslides(0.007, 0, 0.00);
    PIDslides rightController = new PIDslides(0.007, 0, 0.00);
    PIDArm armController = new PIDArm(0.007, 0, 0.00045);


    public void InitializeRobot(HardwareMap hwMap) {


        HardwareMap HWMap = hwMap;


        //initialize Wheels
        motorFrontLeft = HWMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = HWMap.dcMotor.get("motorBackLeft");
        motorFrontRight = HWMap.dcMotor.get("motorFrontRight");
        motorBackRight = HWMap.dcMotor.get("motorBackRight");


        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD); //Practice Bot
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD); //Competition Bot
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE); //Competition Bot & PracticeBot
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE); //Competition Bot & PracticeBot /
        clawWrist = HWMap.get(Servo.class,"clawWrist");
        clawGripper = HWMap.get(Servo.class,"clawGripper");
        slidesLeft = HWMap.dcMotor.get("slidesLeft");
        slidesRight = HWMap.dcMotor.get("slidesRight");
        armMotor = HWMap.dcMotor.get("armMotor");
        slidesLeft.getZeroPowerBehavior();
        slidesRight.getZeroPowerBehavior();
        armMotor.getZeroPowerBehavior();




        leftPower = 0;
        rightPower = 0;
        armPower = 0;


        slideTargetPosition = 0;
        sGROUND_POSITION = 0;
        sMID_POSITION = 1075;
        sHIGH_POSITION = 2150;


        //CHANGE THESE
        armTargetPosition = 0;
        aGROUND_POSITION = -30;
        aLOW_POSITION = -1200;
        aMID_POSITION = -1100;
        aHIGH_POSITION = -90;
        aHANG_POSITION = -800;
        aPULL_POSITION = -200;
    }  //end of method InitializeRobot


    public void DriveStraight(double power, double totalSeconds, int Direction) throws InterruptedException {


        motorFrontLeft.setPower(power * Direction);
        motorBackLeft.setPower(power * Direction);
        motorFrontRight.setPower(power * Direction);
        motorBackRight.setPower(power * Direction);


        Thread.sleep((long) totalSeconds);


        //Stop Robot
        motorFrontLeft.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackRight.setPower(0.0);


    } //End DriveStraight Method


    public void DriveSideways(double power, long totalSeconds, int Direction) throws InterruptedException {


        motorFrontLeft.setPower(power * Direction);
        motorBackLeft.setPower(power * -Direction);
        motorFrontRight.setPower(power * -Direction);
        motorBackRight.setPower(power * Direction);


        Thread.sleep(totalSeconds);


        // stops all motion
        motorFrontLeft.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackRight.setPower(0.0);


    } //Ends DriveSideways Method


    public void DiagonalForward(double power, long totalSeconds, int Direction) throws InterruptedException {




        if (Direction == 1) {


            motorFrontLeft.setPower(power * Direction);
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackRight.setPower(power * Direction);


            Thread.sleep(totalSeconds);
        }


        if (Direction == -1) {


            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(power * -Direction);
            motorFrontRight.setPower(power * -Direction);
            motorBackRight.setPower(0);


            Thread.sleep(totalSeconds);
        }


        // stops all motion


        motorFrontLeft.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackRight.setPower(0.0);


    } //End Diagonal Forward Method




    public void DiagonalBackward(double power, long totalSeconds, int Direction) throws InterruptedException {


        if (Direction == 1) {


            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(power * -Direction);
            motorFrontRight.setPower(power * -Direction);
            motorBackRight.setPower(0);


            Thread.sleep(totalSeconds);
        }


        if (Direction == -1) {


            motorFrontLeft.setPower(power * Direction);
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackRight.setPower(power * Direction);


            Thread.sleep(totalSeconds);
        }


        // stops all motion


        motorFrontLeft.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackRight.setPower(0.0);


    } //End Diagonal Backward Method




    public void CenterSpin(double power, long totalSeconds, int Direction) throws InterruptedException {




        if (Direction == 1) {


            motorFrontLeft.setPower(power * Direction);
            motorBackLeft.setPower(power * Direction);
            motorFrontRight.setPower(power * -Direction);
            motorBackRight.setPower(power * -Direction);


            Thread.sleep(totalSeconds);
        }


        if (Direction == -1) {


            motorFrontLeft.setPower(power * Direction);
            motorBackLeft.setPower(power * Direction);
            motorFrontRight.setPower(power * -Direction);
            motorBackRight.setPower(power * -Direction);


            Thread.sleep(totalSeconds);
        }
        // stops all motion
        motorFrontLeft.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackRight.setPower(0.0);


    } //End CenterSpin Method


    public void StopMotion(double seconds) throws InterruptedException {
        // stops all motion


        motorFrontLeft.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackRight.setPower(0.0);
    }//end StopMotion Method


    public void outputLow() throws InterruptedException {
        //all these do is set the target position and run move arm
        slideTargetPosition = sGROUND_POSITION;
        armTargetPosition = aLOW_POSITION;
        moveArm();
    }


    public void outputGround() throws InterruptedException {


        slideTargetPosition = sGROUND_POSITION;
        armTargetPosition = aGROUND_POSITION;
        moveArm();
    }


    public void outputHigh() throws InterruptedException {
        slideTargetPosition = sHIGH_POSITION;
        armTargetPosition = aHIGH_POSITION;
        moveArm();
    }


    public void outputMid() throws InterruptedException {
        slideTargetPosition = sMID_POSITION;
        armTargetPosition = aMID_POSITION;
        moveArm();
    }


    public void outputHang() throws InterruptedException {
        slideTargetPosition = sHIGH_POSITION;
        armTargetPosition = aHANG_POSITION;
        moveArm();
    }


    public void outputPull() throws InterruptedException {
        slideTargetPosition = sGROUND_POSITION;
        armTargetPosition = aPULL_POSITION;
        moveArm();
    }


    public void clawClose() throws InterruptedException {
        clawGripper.setPosition(0.50);
    }


    public void clawOpen() throws InterruptedException {
        clawGripper.setPosition(0.4);
    }


    public void moveArm() {
        //use this to reposition arm and slide if it droops
        //it moves the arm and slides to the the target position
        while(!inPlace()) {
            //give the power to the motors based on the controllers
            updatePID();
            //needs to run this every time
            slidesLeft.setPower(leftPower);
            slidesRight.setPower(rightPower);
            armMotor.setPower(armPower);


            updatePID();
        }
    }


    public void updatePID() {
        //NO TOUCHY
        //if no work, this is not the issue
        leftPower = leftController.update(slideTargetPosition, slidesLeft.getCurrentPosition());
        rightPower = rightController.update(slideTargetPosition, slidesRight.getCurrentPosition());
        armPower = armController.update(armTargetPosition, armMotor.getCurrentPosition());
    }


    public boolean inPlace() {
        //checks if the power is zero and theoretically that means it is in place
        // if no work make it check if target position is same or close to current position
        if ((leftPower < 0.05) && (leftPower > -0.05)) {
            if ((rightPower < 0.05) && (rightPower > -0.05)) {
                if ((armPower < 0.05) && (armPower > -0.05)) {
                    //don't simply the if statement
                    return true;
                } else {
                    return false;
                }
            } else {
                return false;
            }
        } else {
            return false;
        }
    }


}



