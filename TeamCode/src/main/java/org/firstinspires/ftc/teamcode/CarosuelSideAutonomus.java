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
    protected Recognition duck;
    
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
    protected float duckLeft;
    protected double distToWall = 0;
    double distToMove = 0;
    int ticksToMove = 0;
    
    protected enum Level {
        TOP,
        MIDDLE,
        BOTTOM,
        WHAT
    }
    protected Level level;

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
        
        tfodFreightFrenzy.initialize(
            vuforiaFreightFrenzy, //Vuforia
            (float) 0.5, //Minimum confidence
            true, //Use object tracker
            true //Enable camera monitoring
        );
        
        tfodFreightFrenzy.activate();
        tfodFreightFrenzy.setZoom(1, 16 / 9);
        
        basics = new OpModeBasics(front_right, front_left, back_right, back_left);
        wheels = basics.createWheelGroup(front_right, front_left, back_right, back_left);
        wheels.frEncoderReverse = true;
        wheels.flEncoderReverse = true;
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
        
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    @Override
    public void loop() {
        if (phase == 1) {
            //detect duck position
            phase1();
        } else if (phase == 2) {
            //Strafe to align with shipping hub
            phase2();
        } else if (phase == 3) {
            //Elevate lift to correct level
            phase3();
        } else if (phase == 4) {
            //calculate dist to hub and move forward to shipping hub
            phase4();
        } else if (phase == 5) {
            //Drop freight
            phase5();
        } else if (phase == 6) {
            //Strafe into wall
            phase6();
        } else if (phase == 7) {
            //Back up to carousel
            phase7();
        } else if (phase == 8) {
            //Spin Carousel
            phase8();
        } else if (phase == 9) {
            //Strafe straigt to storage unit
            phase9();
        } else if (phase == 10) {
            //Adjust to be completely in storage unit
            //phase10();
        }
        telemetry.addData("phase", phase);
        telemetry.addData("ticks to move", ticksToMove);
        telemetry.update();
    }
    
    public void phase1() {
        if (firstLoop) {
            /*basics.powerMotors(-0.3);*/
            recognitions = tfodFreightFrenzy.getRecognitions();
            
            if (recognitions.size() == 0) {
                level = Level.WHAT;
                distToWall = 34.0;
                telemetry.addData("what", "there's no duck");
            } else {
                index = 0;
                for (Recognition rec : recognitions) {
                    if (rec.getLabel() == "Duck") {
                        duck = rec;
                        duckLeft = duck.getLeft();
                        
                       if (duckLeft >= 450) {
                            level = Level.TOP;
                            distToWall = 39.0;
                        } else if (duckLeft <= 350 && duckLeft >= 100) {
                            level = Level.MIDDLE;
                            distToWall = 34.0;
                        } else {
                            level = Level.BOTTOM;
                            distToWall = 34.0;
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
    
    //Old
    /*public void phase2() {
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
            telemetry.addData("carousel motor power", carouselMotor.getPower());
            if (endTime - startTime >= 4.5) {
                carouselMotor.setPower(0);
                endPhase();
            }
        }
    }*/
    
    //New
    public void phase2() {
        if (firstLoop) {
            //int ticks = basics.inchToTick(28.0, 480, 12.12);
            
            /*front_right.setTargetPosition(ticks);
            front_left.setTargetPosition(ticks);
            back_right.setTargetPosition(ticks);
            back_left.setTargetPosition(ticks);*/
            
            if (blueTeam) {
                basics.powerMotors(-0.5, 0.5, 0.5, -0.5);
            } else {
                basics.powerMotors(0.5, -0.5, -0.5, 0.5);
            }
            
            firstLoop = false;
        } else {
            
            int ticks = basics.inchToTick(40.0, 480, 12.12);
            
            if (front_right.getCurrentPosition() <= -ticks &&
            front_left.getCurrentPosition() >= ticks &&
            back_right.getCurrentPosition() >= ticks &&
            back_left.getCurrentPosition() <= -ticks && blueTeam) {
                basics.powerMotors(0);
                resetEncoders();
                endPhase();
            } else if (front_right.getCurrentPosition() >= ticks &&
            front_left.getCurrentPosition() <= -ticks &&
            back_right.getCurrentPosition() <= -ticks &&
            back_left.getCurrentPosition() >= ticks && !blueTeam) {
                basics.powerMotors(0);
                resetEncoders();
                endPhase();
            }
            
            
            telemetry.addData("tgt ticks", ticks);
            telemetry.addData("fr pos", front_right.getCurrentPosition());
            telemetry.addData("fl pos", front_left.getCurrentPosition());
            telemetry.addData("br pos", back_right.getCurrentPosition());
            telemetry.addData("bl pos", back_left.getCurrentPosition());
            
        }
    }
    
    //old
    /*public void phase3() {
        if (firstLoop) {
            if (blueTeam) {
                basics.powerMotors(-strafePower, strafePower, strafePower, -strafePower);
            } else {
                basics.powerMotors(strafePower, -strafePower, -strafePower, strafePower);
            }
            startTime = runtime.time(TimeUnit.SECONDS);
            firstLoop = false;
        } else {
            endTime = runtime.time(TimeUnit.SECONDS);
            if (endTime - startTime >= 1) {
                basics.powerMotors(0);
                endPhase();
            }
        }
    }*/
    
    public void phase3() {
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
    
    //old
    /*public void phase4() {
        if (firstLoop) {
            basics.powerMotors(-0.4);
            firstLoop = false;
        } else {
            if (distBack.getDistance(DistanceUnit.INCH) <= 5) {
                basics.powerMotors(0);
            }
        }
    }*/
    
    public void phase4() {
        if (firstLoop) {
            distToMove = distToWall - distBack.getDistance(DistanceUnit.INCH);
            ticksToMove = basics.inchToTick(distToMove, 480, 12.12);
            activateMotors();
            basics.powerMotors(0.4);
            firstLoop = false;
        } else {
            if (front_right.getCurrentPosition() >= ticksToMove ||
            front_left.getCurrentPosition() >= ticksToMove ||
            back_right.getCurrentPosition() >= ticksToMove ||
            back_left.getCurrentPosition() >= ticksToMove) {
                basics.powerMotors(0);
                resetEncoders();
                endPhase();
            }
            telemetry.addData("fr pos", front_right.getCurrentPosition());
            telemetry.addData("fl pos", front_left.getCurrentPosition());
            telemetry.addData("br pos", back_right.getCurrentPosition());
            telemetry.addData("bl pos", back_left.getCurrentPosition());
        }
    }
    
    public void phase5() {
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
    
    public void phase6() {
        if (firstLoop) {
            activateMotors();
            if (blueTeam) {
                basics.powerMotors(0.5, -0.5, -0.5, 0.5);
            } else {
                basics.powerMotors(-0.5, 0.5, 0.5, -0.5);
            }
            
            firstLoop = false;
        } else {
            int ticks = basics.inchToTick(40.0, 480, 12.12);
            
            if (front_right.getCurrentPosition() >= ticks &&
            front_left.getCurrentPosition() <= -ticks &&
            back_right.getCurrentPosition() <= -ticks &&
            back_left.getCurrentPosition() >= ticks && blueTeam) {
                basics.powerMotors(0);
                resetEncoders();
                endPhase();
            } else if (front_right.getCurrentPosition() <= -ticks &&
            front_left.getCurrentPosition() >= ticks &&
            back_right.getCurrentPosition() >= ticks &&
            back_left.getCurrentPosition() <= -ticks && !blueTeam) {
                basics.powerMotors(0);
                resetEncoders();
                endPhase();
            }
        }
    }

    public void phase7() {
        if (firstLoop) {
            distToMove = 11 - distBack.getDistance(DistanceUnit.INCH);
            ticksToMove = basics.inchToTick(distToMove, 480, 12.12);
            activateMotors();
            basics.powerMotors(-0.5);
            firstLoop = false;
        } else {
            if (front_right.getCurrentPosition() <= ticksToMove ||
                    front_left.getCurrentPosition() <= ticksToMove ||
                    back_right.getCurrentPosition() <= ticksToMove ||
                    back_left.getCurrentPosition() <= ticksToMove) {
                basics.powerMotors(0);
                resetEncoders();
                endPhase();
            }
        }
    }

    public void phase8() {
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

    public void phase9() {
        if (firstLoop) {

            if (blueTeam) {
                basics.powerMotors(-0.5, 0.5, 0.5, -0.5);
            } else {
                basics.powerMotors(0.5, -0.5, -0.5, 0.5);
            }

            activateMotors();

            firstLoop = false;
        } else {

            int ticks = basics.inchToTick(26.0, 480, 12.12);

            if (front_right.getCurrentPosition() <= -ticks &&
                    front_left.getCurrentPosition() >= ticks &&
                    back_right.getCurrentPosition() >= ticks &&
                    back_left.getCurrentPosition() <= -ticks && blueTeam) {
                basics.powerMotors(0);
                resetEncoders();
                endPhase();
            } else if (front_right.getCurrentPosition() >= ticks &&
                    front_left.getCurrentPosition() <= -ticks &&
                    back_right.getCurrentPosition() <= -ticks &&
                    back_left.getCurrentPosition() >= ticks && !blueTeam) {
                basics.powerMotors(0);
                resetEncoders();
                endPhase();
            }
        }
    }
    
    
    public void endPhase() {
        phase++;
        firstLoop = true;
    }
    
    public void resetEncoders() {
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    public void activateMotors() {
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    @Override
    public void stop() {
        
    }
}
