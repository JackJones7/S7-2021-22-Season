package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.firstinspires.ftc.teamcode.lib.OpModeBasics;

@Autonomous

public class WarehouseSideAutonomus extends OpMode{
    protected Servo bucketServo;
    protected DcMotor carouselMotor;
    protected DistanceSensor distBack;
    protected DistanceSensor distL;
    protected DistanceSensor distR;
    protected Blinker expansion_Hub_2;
    protected Blinker expansion_Hub_3;
    protected DcMotor liftMotor;
    protected DcMotor back_left;
    protected DcMotor back_right;
    protected DcMotor front_left;
    protected DcMotor front_right;
    protected Gyroscope imu;
    protected Servo left_intake;
    protected Servo right_intake;
    
    protected boolean blueTeam;
    protected OpModeBasics basics;
    protected int phase;
    protected boolean firstLoop = true;
    protected double startTime;
    protected double endTime;
    protected ElapsedTime runtime = new ElapsedTime();
    
    
    @Override
    public void init() {
        blueTeam = true;
        
        front_right = hardwareMap.dcMotor.get("front_right");
        front_left = hardwareMap.dcMotor.get("front_left");
        back_right = hardwareMap.dcMotor.get("back_right");
        back_left = hardwareMap.dcMotor.get("back_left");
        
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        
        liftMotor = hardwareMap.dcMotor.get("LiftMotor");
        
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        basics = new OpModeBasics(front_right, front_left, back_right, back_left);
        
        phase = 1;
    }
    
    @Override
    public void start() {
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    @Override
    public void loop() {
        if (phase == 1) {
            phase1();
        } else if (phase == 2) {
            phase2();
        } else if (phase == 3) {
            phase3();
        }
    }
    
    public void phase1() {
        if (firstLoop) {
            liftMotor.setTargetPosition(409);
            liftMotor.setPower(0.5);
            firstLoop = false;
        } else {
            if (!liftMotor.isBusy()) {
                endPhase();
            }
        }
    }
    
    public void phase2() {
        if (firstLoop) {
            if (blueTeam) {
                basics.powerMotors(0.6, -0.6, -0.6, 0.6);
            } else {
                basics.powerMotors(-0.6, 0.6, 0.6, -0.6);
            }
            startTime = runtime.time(TimeUnit.SECONDS);
            firstLoop = false;
        } else {
            endTime = runtime.time(TimeUnit.SECONDS);
            if (endTime - startTime >= 1.5) {
                basics.powerMotors(0);
                endPhase();
            }
        }
    }
    
    public void phase3 () {
        if (firstLoop) {
            basics.powerMotors(0.6);
            startTime = runtime.time(TimeUnit.SECONDS);
            firstLoop = false;
        } else {
            endTime = runtime.time(TimeUnit.SECONDS);
            if (endTime - startTime >= 0.7) {
                basics.powerMotors(0);
                endPhase();
            }
        }
    }
    
    
    
    public void endPhase() {
        phase++;
        firstLoop = true;
    }
}
