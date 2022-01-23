package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.lib.OpModeBasics;
import org.firstinspires.ftc.teamcode.lib.OpModeBasics.WheelGroup;

@Autonomous

public class OMBTest extends OpMode {
    private Servo bucketServo;
    private DcMotor carouselMotor;
    private DistanceSensor distBack;
    private DistanceSensor distL;
    private DistanceSensor distR;
    private Blinker expansion_Hub_2;
    private Blinker expansion_Hub_3;
    private DcMotor liftMotor;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_left;
    private DcMotor front_right;
    private Servo left_intake;
    private Servo right_intake;
    private BNO055IMU imu;
    
    private OpModeBasics basics;
    private boolean setupComplete = false;
    private WheelGroup wheels;

    // todo: write your code here
    @Override
    public void init() {
        front_right = hardwareMap.dcMotor.get("front_right");
        front_left = hardwareMap.dcMotor.get("front_left");
        back_right = hardwareMap.dcMotor.get("back_right");
        back_left = hardwareMap.dcMotor.get("back_left");
        
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        basics = new OpModeBasics(front_right, front_left, back_right, back_left);
        wheels = basics.createWheelGroup(front_right, front_left, back_right, back_left);

        wheels.setTargetPositions(0);
        wheels.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        
        //parameters.mode = BNO055IMU.SensorMode.IMU;
        //parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.loggingEnabled = false;
        
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        //imu.initialize(parameters);
        
        //telemetry.addData("Imu Status", "Calibrating...");
        //telemetry.update();
    }
    
    @Override
    public void init_loop() {
        //if (!imu.isGyroCalibrated() && !imu.isAccelerometerCalibrated() || setupComplete) {
        //    return;
        //}
        //telemetry.addData("Imu Status", imu.getCalibrationStatus().toString());
        
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 5);
        
        if (gamepad1.a) {
            wheels.setTargetPositions(480);
            wheels.setModes(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("encoder", "yes");
        } else if (gamepad1.b) {
            wheels.setTargetPositions(0);
            wheels.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("encoder", "no");
        }

        
        //telemetry.addData("OMB", "ready");
        telemetry.addData("Hello", "Good job using Android Studio");
        
        telemetry.update();
        setupComplete = true;
    }
    
    @Override
    public void loop() {
        //if (!imu.isGyroCalibrated() && !imu.isAccelerometerCalibrated()) {
        //    return;
        //}
        //telemetry.addData("Imu Status", imu.getCalibrationStatus().toString());
        
        //basics.moveRobotImu(20, 0.4, Axis.Y);
        
        //telemetry.addData("Imu Y Position", imu.getPosition().y);

        if (wheels.getTargetPositions().fr == 0) {
            wheels.setPower(gamepad1.right_stick_y, gamepad1.left_stick_y);
        } else {
            wheels.setPower(0.5);
        }

        WheelGroup.WheelInts positions = wheels.getCurrentPositions();

        telemetry.addData("fr pos", positions.fr);
        telemetry.addData("fl pos", positions.fl);
        telemetry.addData("br pos", positions.br);
        telemetry.addData("bl pos", positions.bl);
        
        telemetry.update();
    }
    

}
