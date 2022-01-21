package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.lib.OpModeBasics;

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

    // todo: write your code here
    @Override
    public void init() {
        front_right = hardwareMap.dcMotor.get("front_right");
        front_left = hardwareMap.dcMotor.get("front_left");
        back_right = hardwareMap.dcMotor.get("back_right");
        back_left = hardwareMap.dcMotor.get("back_left");
        
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        imu.initialize(parameters);
        
        telemetry.addData("Imu Status", "Calibrating...");
        telemetry.update();
    }
    
    @Override
    public void init_loop() {
        if (!imu.isGyroCalibrated() && !imu.isAccelerometerCalibrated() || setupComplete) {
            return;
        }
        telemetry.addData("Imu Status", imu.getCalibrationStatus().toString());
        
        imu.startAccelerationIntegration(new Position(), new Velocity(), 5);
        
        
        basics = new OpModeBasics(front_right, front_left, back_right, back_left, imu);
        
        telemetry.addData("OMB", "ready");
        
        telemetry.update();
        setupComplete = true;
    }
    
    @Override
    public void loop() {
        if (!imu.isGyroCalibrated() && !imu.isAccelerometerCalibrated()) {
            return;
        }
        telemetry.addData("Imu Status", imu.getCalibrationStatus().toString());
        
        basics.moveRobotImu(20, 0.4, Axis.Y);
        
        telemetry.addData("Imu Y Position", imu.getPosition().y);
        
        telemetry.update();
    }
    

}
