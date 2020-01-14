
class test extends LinearOpMode {

  @Override
  public void runOpMode() {
    
    // hardwaremap stuff
    
    // arm needs to start in down position
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODERS);
    
    waitForStart();
    
    while (opModeIsActive()) {
      
      // WHEEL CODE
      // negate if going in the wrong direction
      double leftPower = gamepad1.left_stick_y;
      double rightPower = gamepad1.right_stick_y;
      
      leftMotor.setPower(leftPower);
      rightMotor.setPower(rightPower);
      // --------------------------
      
      // CLAW CODE
      if (gamepad2.x) {
        
        gripper.setPosition(1);
        
      }
      if (gamepad2.b) {
        
        gripper.setPosition(0);
        
      }
      
      
      // ARM CODE
      if (gamepad2.y) { // repeat for gamepad2.a   |  setTargetPosition(0);
        
        armMotor.setTargetPosition(500);
        
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        armMotor.setPower(.7);
        
        while (opModeIsActive() && armMotor.isBusy()) {
          
          telemetry.addData("Arm position: ", armMotor.getCurrentPosition());
          telemetry.update();
          
          leftPower = gamepad1.left_stick_y;
          rightPower = gamepad1.right_stick_y;
      
          leftMotor.setPower(leftPower);
          rightMotor.setPower(rightPower);
          
        }
        
        armMotor.setPower(0);
        
      }
      
    }
    
  }

}
