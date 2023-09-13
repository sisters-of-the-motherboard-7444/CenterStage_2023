//--------------------------Driving Pathways------------------------------------

  /*  This method allows the robot to drive forward based on encoder values.
      A distance is given that is converted to an encoder position in the code.
      leftDirection and rightDirection set the direction of the motors to allow
      the robot to either move straight ot turn.
   */

   public void DriveStraight(double power, double totalSeconds, int Direction) throws InterruptedException {

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

       Thread.sleep((long) totalSeconds);

       //Stop Robot
       motorFrontLeft.setPower(0.0);
       motorBackLeft.setPower(0.0);
       motorFrontRight.setPower(0.0);
       motorBackRight.setPower(0.0);

   } //End DriveStraight Method

   public void DriveSideways(double power, long totalSeconds, int Direction) throws InterruptedException {

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

       Thread.sleep(totalSeconds);

       // stops all motion
       motorFrontLeft.setPower(0.0);
       motorBackLeft.setPower(0.0);
       motorFrontRight.setPower(0.0);
       motorBackRight.setPower(0.0);

   } //Ends DriveSideways Method

   public void DiagonalForward(double power, long totalSeconds, int Direction) throws InterruptedException {

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

       //For driving forward in a diagonal direction
       // declare variables for this method (power, totalSeconds (milliseconds) & Direction)
       //For right motion set direction = 1 (In method call)
       //For left motion set direction = -1 (In method call)
       //example: DiagonalBackward(.8, 3, 1) means drive straight at 80% power, for 3 seconds, in back Right direction
       //example: DiagonalForward(.75, 5, -1) means drive straight at 75% power, for 5 seconds, in left direction

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

}
