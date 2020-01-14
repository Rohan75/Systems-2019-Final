
class test extends LinearOpMode {

  @Override
  public void runOpMode() {
    
    // hardwaremap stuff
    
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODERS);
    
    waitForStart();
    
    while (opModeIsActive()) {
      
      double leftPower = gamepad1.left_stick_y;
      double rightPower = gamepad1.right_stick_y;
      
      leftMotor.setPower(leftPower);
      rightMotor.setPower(rightPower);
      
      if (gamepad2.y) {
        
        armMotor.setTargetPosition(500);
        
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        armMotor.setPower(.7);
        
        while (opModeIsActive() && armMotor.isBusy()) {
          
          telemetry.addData("Arm position: ", armMotor.getCurrentPosition());
          telemetry.update();
          
        }
        
        armMotor.setPower(0);
        
      }
      
    }
    
  }

}
