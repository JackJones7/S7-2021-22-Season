/*

Basic Java TeleOp program by Jack Jones

*/



package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="JavaProgram")

public class JavaProgram extends LinearOpMode{
    private DcMotor back_Left;
    private DcMotor back_Right;
    private DistanceSensor distanceSensor;
    private Blinker expansion_Hub_3;
    private DcMotor front_Left;
    private DcMotor front_Right;
    private Gyroscope imu;
    private Servo servo1;
    
    private double motorPowerL = 0.0;
    private double motorPowerR = 0.0;
    
    private int mode = 1;
    private int totalModes = 2;
    
    private double strafeDeadzone = 0.2;
    private double turnDeadzone = 0.2; //Only matters for mode 2
    private double slowDeadzone = 0.5;
    
    private boolean lbLock = false;

    @Override
    public void runOpMode()  throws InterruptedException
    {
        //---put init code here---
        
        back_Left = hardwareMap.dcMotor.get("back_left");
        back_Right = hardwareMap.dcMotor.get("back_right");
        front_Left = hardwareMap.dcMotor.get("front_left");
        front_Right = hardwareMap.dcMotor.get("front_right");
        
        front_Left.setDirection(DcMotor.Direction.REVERSE);
        back_Left.setDirection(DcMotor.Direction.REVERSE);
        
        back_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //------------------------
        
        
        waitForStart();
        
        
        //---put run code here---
        
        //-----------------------
        
        while (opModeIsActive())
        {
            
        //---put loop code here---
            
            
            
            //This is the code for moving the motors
            if (mode == 1) {
                
                //In mode 1, the left stick controls the left motors,
                //and the right stick controls the right motors.
                motorPowerL = gamepad1.left_stick_y;
                motorPowerR = gamepad1.right_stick_y;
                
            } else if (mode == 2) {
                
                //In mode 2, the left stick controls all motors.
                motorPowerL = gamepad1.left_stick_y;
                motorPowerR = gamepad1.left_stick_y;
                
            }
            
            
            
            //Turning contorls, for mode 2
            if (mode == 2) {
                
                //If left stick X is past the deadzone in either direction,
                //motor power is negative on one side and positive on the other.
                if (gamepad1.left_stick_x >= turnDeadzone || gamepad1.left_stick_x <= turnDeadzone * -1) {
                    
                    motorPowerL = gamepad1.left_stick_x * -1;
                    motorPowerR = gamepad1.left_stick_x;
                    
                }
                
            }
            
            

            //If the right trigger is held down, speed is cut in half.
            if (gamepad1.right_trigger >= slowDeadzone) {
                
                motorPowerL /= 2;
                motorPowerR /= 2;
                
            }
            
            
            
            //If the right stick X is past the deadzone in either direction,
            //it ignores the motorPower R and L variables and runs the strafing
            //code.
            
            if (gamepad1.right_stick_x >= strafeDeadzone || gamepad1.right_stick_x <= strafeDeadzone * -1) {
                
                //On each side, the wheels are moving in opposite directions.
                //Left side moves outward in opposite directions when strafing right.
                front_Left.setPower(gamepad1.right_stick_x);
                back_Left.setPower(gamepad1.right_stick_x * -1);
                front_Right.setPower(gamepad1.right_stick_x * -1);
                back_Right.setPower(gamepad1.right_stick_x);
                
            } else {
            
            //Otherwise, it sets the motor power on each side
            //to its respective variable.
                front_Left.setPower(motorPowerL);
                back_Left.setPower(motorPowerL);
                front_Right.setPower(motorPowerR);
                back_Right.setPower(motorPowerR);
                
            }
            
            
            //If left bumper is pressed, the mode is incremented by 1
            if (gamepad1.left_bumper && !lbLock) {
                
                mode += 1;
                
                //If mode is higher than the total number of modes, it's set to 1
                if (mode > totalModes) {
                    
                    mode = 1;
                    
                }
                
                //lbLock variable is to keep the mode from repeatedly switching
                //as long as the left bumper is held down.
                lbLock = true;
                
            } else if (!gamepad1.left_bumper) {
                
                lbLock = false;
                
            }
            
            
            telemetry.addData("Mode", mode);
            telemetry.addData("Left Bumper", gamepad1.left_bumper);
            telemetry.update();
            
            
        //-----------------------
        
        }
    }
}
