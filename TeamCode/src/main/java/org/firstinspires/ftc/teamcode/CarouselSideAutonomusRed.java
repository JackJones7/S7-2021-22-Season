package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode.CarosuelSideAutonomus;
import org.firstinspires.ftc.teamcode.lib.OpModeBasics;

@Autonomous

public class CarouselSideAutonomusRed extends CarosuelSideAutonomus{

    // todo: write your code here
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
        
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        
        carouselMotor = hardwareMap.dcMotor.get("CarouselMotor");
        
        distR = hardwareMap.get(DistanceSensor.class, "DistR");
        distL = hardwareMap.get(DistanceSensor.class, "DistL");
        distBack = hardwareMap.get(DistanceSensor.class, "DistBack");
        
        basics = new OpModeBasics(front_right, front_left, back_right, back_left);
        blueTeam = false;
    }
}
