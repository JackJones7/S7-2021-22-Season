package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Autonomous(name = "AQLeftBlue (Blocks to Java)")
public class AQLeftBlue extends LinearOpMode {

  private DcMotor back_left;
  private DcMotor back_right;
  private DcMotor front_left;
  private DcMotor front_right;
  private DistanceSensor DistR;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    back_left = hardwareMap.get(DcMotor.class, "back_left");
    back_right = hardwareMap.get(DcMotor.class, "back_right");
    front_left = hardwareMap.get(DcMotor.class, "front_left");
    front_right = hardwareMap.get(DcMotor.class, "front_right");
    DistR = hardwareMap.get(DistanceSensor.class, "DistR");

    back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    front_right.setDirection(DcMotorSimple.Direction.REVERSE);
    back_right.setDirection(DcMotorSimple.Direction.REVERSE);
    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        if (DistR.getDistance(DistanceUnit.INCH) <= 1) {
          back_left.setPower(0);
          back_right.setPower(0);
          front_left.setPower(0);
          front_right.setPower(0);
          break;
        } else {
          back_left.setPower(0.3);
          back_right.setPower(0.3);
          front_left.setPower(0.3);
          front_right.setPower(0.3);
        }
        telemetry.addData("Distance", DistR.getDistance(DistanceUnit.INCH));
        telemetry.update();
      }
      back_left.setPower(0.3);
      back_right.setPower(0.3);
      front_left.setPower(0.3);
      front_right.setPower(0.3);
      sleep(100);
    }
  }
}
