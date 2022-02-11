package org.firstinspires.ftc.teamcode;

import java.util.List;

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

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

import org.firstinspires.ftc.teamcode.lib.OpModeBasics;
import org.firstinspires.ftc.teamcode.lib.OpModeBasics.WheelGroup;

@Autonomous

public class CarosuelSideAutonomus extends OpMode{
    protected Servo bucketServo;
    protected DcMotor carouselMotor;
    protected DistanceSensor distL;
    protected DistanceSensor distR;
    protected DistanceSensor distBack;
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
    
    protected VuforiaCurrentGame vuforiaFreightFrenzy;
    protected TfodCurrentGame tfodFreightFrenzy;
    protected List<Recognition> recognitions;
    protected Recognition element;
    
    protected OpModeBasics basics;
    protected WheelGroup wheels;

    protected boolean blueTeam = true;
    protected int phase = 1;
    protected boolean firstLoop = true;
    protected ElapsedTime runtime = new ElapsedTime();
    protected double startTime;
    protected double endTime;
    protected double strafePower = 0.8;
    protected int index;
    protected float elementLeft;
    protected double distToWall = 0;
    protected double distToMove = 0;

    protected double speed = 0.4;
    
    protected enum Level {
        TOP,
        MIDDLE,
        BOTTOM,
        WHAT
    }
    protected Level level;


    //setTeam method
    public void setTeam() {
        blueTeam = true;
    }

    //init function
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


        final String[] labels = {
                "ShippingElement"
        };

        tfodFreightFrenzy.setModelFromAsset("ElementV2.tflite", labels);
        
        tfodFreightFrenzy.initialize(
            vuforiaFreightFrenzy, //Vuforia
            (float) 0.7, //Minimum confidence
            true, //Use object tracker
            true //Enable camera monitoring
        );

        
        tfodFreightFrenzy.activate();
        tfodFreightFrenzy.setZoom(1, 16 / 9);
        
        basics = new OpModeBasics(front_right, front_left, back_right, back_left);
        wheels = basics.createWheelGroup(front_right, front_left, back_right, back_left);
        wheels.frEncoderReverse = true;
        wheels.flEncoderReverse = true;
        speed = 0.4;

        setTeam();
    }
    
    //start function
    @Override
    public void start() {
        vuforiaFreightFrenzy.activate();
        
        left_intake.setPosition(0.03);
        right_intake.setPosition(0.03);
        
        /*front_right.setTargetPosition(0);
        front_left.setTargetPosition(0);
        back_right.setTargetPosition(0);
        back_left.setTargetPosition(0);
        
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
        
        wheels.setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    @Override
    public void loop() {
        if (phase == 1) {
            //detect duck position
            phase1();
        } else if (phase == 2) {
            //elevate lift
            phase2();
        } else if (phase == 3) {
            //move backwards
            phase3();
        } else if (phase == 4) {
            //spin carousel
            phase4();
        } else if (phase == 5) {
            //strafe to shipping hub
            phase5();
        } else if (phase == 6) {
            //move up to hub
            phase6();
        } else if (phase == 7) {
            //drop freight
            phase7();
        } else if (phase == 8) {
            //back up to wall
            phase8();
        } else if (phase == 9) {
            //strafe to storage unit
            phase9();
        }
        telemetry.addData("phase", phase);
        telemetry.update();
    }
    
    public void phase1() {
        if (firstLoop) {
            /*basics.powerMotors(-0.3);*/
            recognitions = tfodFreightFrenzy.getRecognitions();
            
            if (recognitions.size() == 0) {
                level = Level.WHAT;
                distToWall = 36.0;
                telemetry.addData("what", "there's no element");
            } else {
                index = 0;
                for (Recognition rec : recognitions) {
                    if (rec.getLabel() == "ShippingElement") {
                        element = rec;
                        elementLeft = element.getLeft();
                        
                       if (elementLeft >= 450) {
                            level = Level.TOP;
                            distToWall = 39.0;
                        } else if (elementLeft <= 350 && elementLeft >= 100) {
                            level = Level.MIDDLE;
                            distToWall = 36.0;
                        } else {
                            level = Level.BOTTOM;
                            distToWall = 36.0;
                        }
                        
                        
                        tfodFreightFrenzy.deactivate();
                    }
                }
            }
            firstLoop = false;
        } else {
            /*telemetry.addData("Back Dist", distBack.getDistance(DistanceUnit.INCH));
            if (distBack.getDistance(DistanceUnit.INCH) <= 9.9) {
                basics.powerMotors(0);
                endPhase();
            }*/
            endPhase();
        }
    }

    public void phase2() {
        if (firstLoop) {
            if (level == Level.TOP) {
                liftMotor.setTargetPosition(643);
            } else if (level == Level.MIDDLE) {
                liftMotor.setTargetPosition(409);
            } else {
                liftMotor.setTargetPosition(190);
            }
            liftMotor.setPower(0.5);
            firstLoop = false;
        } else {
            if (!liftMotor.isBusy()) {
                endPhase();
            }
        }
    }

    public void phase3() {
        if (firstLoop) {
            distToMove = 10 - distBack.getDistance(DistanceUnit.INCH);
            basics.moveRobotEncoder(wheels, -speed, distToMove, 480, 12.12);
            firstLoop = false;
        } else {
            basics.update();
            if (!basics.isActionInProgress()) {
                endPhase();
            }
        }
    }

    public void phase4() {
        if (firstLoop) {
            if (blueTeam) {
                carouselMotor.setPower(1);
            } else {
                carouselMotor.setPower(-1);
            }
            startTime = runtime.time(TimeUnit.SECONDS);
            firstLoop = false;
        } else {
            endTime = runtime.time(TimeUnit.SECONDS);
            if (endTime - startTime >= 5) {
                carouselMotor.setPower(0);
                endPhase();
            }
        }
    }

    public void phase5() {
        if (firstLoop) {
            
            if (blueTeam) {
                basics.moveRobotEncoder(wheels, -speed, speed - 0.05, speed, -speed, 48, 480, 12.12);
            } else {
                basics.moveRobotEncoder(wheels, speed, -speed, -speed, speed - 0.05, 48, 480, 12.12);
            }

            firstLoop = false;
        } else {

            basics.update();
            if (!basics.isActionInProgress()) {
                endPhase();
            }
            
        }
    }

    public void phase6() {
        if (firstLoop) {
            distToMove = distToWall - distBack.getDistance(DistanceUnit.INCH);
            basics.moveRobotEncoder(wheels, speed, distToMove, 480, 12.12);
            firstLoop = false;
        } else {
            basics.update();
            if (!basics.isActionInProgress()) {
                endPhase();
            }
            telemetry.addData("action in progress", basics.isActionInProgress());
        }
    }
    
    public void phase7() {
        if (firstLoop) {
            left_intake.setPosition(0.1);
            right_intake.setPosition(0.1);
            startTime = runtime.time(TimeUnit.SECONDS);
            firstLoop = false;
        } else {
            endTime = runtime.time(TimeUnit.SECONDS);
            if (endTime - startTime >= 0.2) {
                endPhase();
            }
        }
    }

    public void phase8() {
        if (firstLoop) {
            distToMove = distBack.getDistance(DistanceUnit.INCH) - 5;
            basics.moveRobotEncoder(wheels, -speed, distToMove, 480, 12.12);
            firstLoop = false;
        } else {
            basics.update();
            if (basics.isActionInProgress()) {
                return;
            }
            endPhase();
        }
    }

    public void phase9() {
        if (firstLoop) {
            //left (right) 15in
            if (blueTeam) {
                basics.moveRobotEncoder(wheels, speed, -speed, -speed, speed, 20, 480, 12.12);
            } else {
                basics.moveRobotEncoder(wheels, -speed, speed, speed, -speed, 20, 480, 12.12);
            }

            firstLoop = false;
        } else {
            basics.update();
            if (basics.isActionInProgress()) {
                return;
            }
            endPhase();
        }
    }
    
    
    public void endPhase() {
        phase++;
        firstLoop = true;
    }
    
    @Override
    public void stop() {
        
    }
}
