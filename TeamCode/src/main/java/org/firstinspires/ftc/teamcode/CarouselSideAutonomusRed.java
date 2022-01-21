package org.firstinspires.ftc.teamcode;

import java.util.List;
import java.util.logging.Level;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

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

        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        carouselMotor = hardwareMap.dcMotor.get("CarouselMotor");
        liftMotor = hardwareMap.dcMotor.get("LiftMotor");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        distR = hardwareMap.get(DistanceSensor.class, "DistR");
        distL = hardwareMap.get(DistanceSensor.class, "DistL");
        distBack = hardwareMap.get(DistanceSensor.class, "DistBack");

        left_intake = hardwareMap.get(Servo.class, "left_intake");
        right_intake = hardwareMap.get(Servo.class, "right_intake");

        right_intake.setDirection(Servo.Direction.REVERSE);

        vuforiaFreightFrenzy = new VuforiaCurrentGame();
        tfodFreightFrenzy = new TfodCurrentGame();

        vuforiaFreightFrenzy.initialize(
                "", //Vuforia Liscense Key
                hardwareMap.get(WebcamName.class, "Webcam 1"), //Webcam Name
                "", //Webcam Calibration Filename
                false, //Use Extended Tracking
                true, //Enable Camera Monitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, //Camera Monitor Feedback
                0, //dx
                0, //dy
                0, //dz
                AxesOrder.XYZ, //Axes Order
                90, //First Angle
                90, //Second Angle
                0, //Third Angle
                true //Use competition feild target locations
        );

        tfodFreightFrenzy.initialize(
                vuforiaFreightFrenzy, //Vuforia
                (float) 0.5, //Minimum confidence
                true, //Use object tracker
                true //Enable camera monitoring
        );

        tfodFreightFrenzy.activate();
        tfodFreightFrenzy.setZoom(1, 16 / 9);

        basics = new OpModeBasics(front_right, front_left, back_right, back_left);
        blueTeam = false;
    }
}
