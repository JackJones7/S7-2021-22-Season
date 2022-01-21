package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Disabled
@Autonomous

public class ObstacleCourseAug8 extends LinearOpMode {
    private DistanceSensor distanceSensor;
    private Blinker expansion_Hub_3;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_left;
    private DcMotor front_right;
    private BNO055IMU imu;
    
    private int phase = 0;
    private double slowDist = 24;
    private double stopDist = 10;


    // todo: write your code here
    
    public void runOpMode() throws InterruptedException {
        //init code
        
        BNO055IMU.Parameters parameters  = new BNO055IMU.Parameters();
        
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        imu.initialize(parameters);
        
        telemetry.addData("Imu", "Calibrating");
        telemetry.update();
        
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        
        telemetry.addData("Imu calibration status", imu.getCalibrationStatus().toString());
        telemetry.update();
        
        back_left = hardwareMap.dcMotor.get("back_left");
        back_right = hardwareMap.dcMotor.get("back_right");
        front_left = hardwareMap.dcMotor.get("front_left");
        front_right = hardwareMap.dcMotor.get("front_right");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "DistanceSensor");
        
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        
        waitForStart();
        //run code
        back_left.setPower(0.2);
        back_right.setPower(0.2);
        front_right.setPower(0.2);
        front_left.setPower(0.2);
        while (opModeIsActive()) {
            //loop code
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            //turn when it comes up to the wall
            if (phase == 1) {
                front_left.setPower(0.3);
                back_left.setPower(0.3);
                front_right.setPower(-0.3);
                back_right.setPower(-0.3);
                phase++;
            } else if (phase == 4) {
                front_left.setPower(-0.3);
                back_left.setPower(-0.3);
                front_right.setPower(0.3);
                back_right.setPower(0.3);
                phase++;
            }
            
            
            //after turning, move foreward again
            if (phase == 3 || phase == 6) {
                back_left.setPower(0.2);
                back_right.setPower(0.2);
                front_right.setPower(0.2);
                front_left.setPower(0.2);
            }
            //when angle <= -90, stop turning
            if (phase == 2 && angles.firstAngle <= -80) {
                front_left.setPower(0);
                back_left.setPower(0);
                front_right.setPower(0);
                back_right.setPower(0);
                phase++;
            }
            
            if (phase == 5 && angles.firstAngle >= -10) {
                front_left.setPower(0);
                back_left.setPower(0);
                front_right.setPower(0);
                back_right.setPower(0);
                phase++;
            }
            
            
            //stop
            if (distanceSensor.getDistance(DistanceUnit.INCH) <=stopDist && (phase == 0 || phase == 3)) {
                front_left.setPower(0);
                front_right.setPower(0);
                back_left.setPower(0);
                back_right.setPower(0);
                phase++;
            }
            
            telemetry.addData("Imu Angle", angles.firstAngle);
            telemetry.addData("Dist sensor inches", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
            
            idle();
        }
    }
    
}
