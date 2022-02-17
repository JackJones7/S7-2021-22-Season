package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.firstinspires.ftc.teamcode.lib.RobotBasics;

@TeleOp

public class S7TeleOp extends OpMode {
    private Servo bucketServo;
    private DcMotor carouselMotor;
    private DistanceSensor distL;
    private DistanceSensor distR;
    private DistanceSensor distBack;
    private Blinker expansion_Hub_2;
    private Blinker expansion_Hub_3;
    private DcMotor liftMotor;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_left;
    private DcMotor front_right;
    private Gyroscope imu;
    private Servo left_intake;
    private Servo right_intake;
    
    private RobotBasics basics;
    private double bucketPos = 0;
    private boolean limitUnlocked = false;
    
    private double wheelFactor = 1.0;
    private int liftTgt = 0;

    @Override
    public void init() {
        front_right = hardwareMap.dcMotor.get("front_right");
        front_left = hardwareMap.dcMotor.get("front_left");
        back_right = hardwareMap.dcMotor.get("back_right");
        back_left = hardwareMap.dcMotor.get("back_left");
        
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        
        carouselMotor = hardwareMap.dcMotor.get("CarouselMotor");
        liftMotor = hardwareMap.dcMotor.get("LiftMotor");
        
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        
        left_intake = hardwareMap.servo.get("left_intake");
        right_intake = hardwareMap.servo.get("right_intake");
        bucketServo = hardwareMap.servo.get("BucketServo");
        
        bucketServo.scaleRange(0, 0.65);
        
        left_intake.scaleRange(0, 0.3);
        right_intake.scaleRange(0.7, 1);
        right_intake.setDirection(Servo.Direction.REVERSE);

        distR = hardwareMap.get(DistanceSensor.class, "DistR");
        distL = hardwareMap.get(DistanceSensor.class, "DistL");
        distBack = hardwareMap.get(DistanceSensor.class, "DistBack");
        
        basics = new RobotBasics(front_right, front_left, back_right, back_left);
    }
    
    @Override
    public void start() {
        //liftMotor.setTargetPosition(0);
        //liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //liftMotor.setPower(1);
    }
    
    @Override
    public void loop() {
        
        if (gamepad1.right_bumper) {
            wheelFactor = 0.5;
        } else {
            wheelFactor = 1.0;
        }
        
        if (gamepad1.dpad_left) {
            basics.powerMotors(-1 * wheelFactor, 1 * wheelFactor, 1 * wheelFactor, -1 * wheelFactor);
        } else if (gamepad1.dpad_right) {
            basics.powerMotors(1 * wheelFactor, -1 * wheelFactor, -1 * wheelFactor, 1 * wheelFactor);
        } else if (gamepad1.dpad_up) {
            basics.powerMotors(-1 * wheelFactor);
        } else if (gamepad1.dpad_down) {
            basics.powerMotors(1 * wheelFactor);
        } else {
            basics.powerMotors(gamepad1.right_stick_y * wheelFactor, gamepad1.left_stick_y * wheelFactor);
        }
        
        left_intake.setPosition(gamepad2.right_trigger);
        right_intake.setPosition(gamepad2.right_trigger);
        
        carouselMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        
        
        //liftTgt -= gamepad2.left_stick_y * 5;
        //
        //if (liftTgt < 0) {
        //    liftTgt = 0;
        //} else if (liftTgt > 810) {
        //    liftTgt = 810;
        //}
        //
        //liftMotor.setTargetPosition(liftTgt);
        liftMotor.setPower(gamepad2.left_stick_y * 0.4);
        
        bucketPos += gamepad2.right_stick_x / 10;
        
        if (bucketPos > 1) {
            bucketPos = 1;
        } else if (bucketPos < 0.3 && !limitUnlocked) {
            bucketPos = 0.3;
        } else if (bucketPos < 0 && limitUnlocked) {
            bucketPos = 0;
        }
        
        if (gamepad2.a) {
            bucketPos = 0.3;
        }
        
        if (gamepad2.x) {
            limitUnlocked = true;
        } else {
            limitUnlocked = false;
        }
        
        bucketServo.setPosition(bucketPos);
        
        //telemetry.addData("distR distance", distR.getDistance(DistanceUnit.INCH));
        //telemetry.addData("distR OOR", distR.distanceOutOfRange);
        //telemetry.addData("distL distance", distL.getDistance(DistanceUnit.INCH));
        telemetry.addData("arm position", liftMotor.getCurrentPosition());
        telemetry.addData("arm target", liftMotor.getTargetPosition());
        telemetry.addData("back dist", distBack.getDistance(DistanceUnit.INCH));
        telemetry.addData("fl pos", front_left.getCurrentPosition());
        telemetry.update();
        
    }
}
