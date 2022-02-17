package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

import org.firstinspires.ftc.teamcode.lib.RobotBasics;

import java.util.List;

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

    protected VuforiaCurrentGame vuforiaFreightFrenzy;
    protected TfodCurrentGame tfodFreightFrenzy;
    protected List<Recognition> recognitions;
    protected Recognition element;

    protected double distToWall;
    
    protected boolean blueTeam;
    protected RobotBasics basics;
    protected int phase;
    protected boolean firstLoop = true;
    protected double startTime;
    protected double endTime;
    protected ElapsedTime runtime = new ElapsedTime();

    protected RobotBasics.WheelGroup wheels;

    protected enum Level {
        TOP,
        MIDDLE,
        BOTTOM,
        WHAT
    }

    protected Level level;
    
    public void setTeam() {
        blueTeam = true;
    }

    @Override
    public void init() {
        setTeam();
        
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

        right_intake = hardwareMap.get(Servo.class, "right_intake");
        left_intake = hardwareMap.get(Servo.class, "left_intake");

        right_intake.setDirection(Servo.Direction.REVERSE);
        
        liftMotor = hardwareMap.dcMotor.get("LiftMotor");
        
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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

        
        basics = new RobotBasics(front_right, front_left, back_right, back_left);
        wheels = basics.createWheelGroup(front_right, front_left, back_right, back_left);
        
        phase = 1;
    }
    
    @Override
    public void start() {
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vuforiaFreightFrenzy.activate();

        right_intake.setPosition(0.03);
        left_intake.setPosition(0.03);
    }
    
    @Override
    public void loop() {
        if (phase == 1) {
            //Detect Duck level
            phase1();
        } else if (phase == 2) {
            //Raise arm to correct level
            phase2();
        } else if (phase == 3) {
            //Move forward to hub
            phase3();
        } else if (phase == 4) {
            //Strafe about halfway to hub
            phase4();
        } else if (phase == 5) {
            //Turn 90 Degrees towards hub
            //phase5();
            //vertical dist = 11.25, horizontal dist = 13.25
        }
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
                for (Recognition rec : recognitions) {
                    if (rec.getLabel() == "ShippingElement") {
                        element = rec;
                        double elementLeft = element.getLeft();

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

            if (liftMotor.isBusy()) {
                return;
            }

            endPhase();
        }
    }
    
    public void phase3 () {
        if (firstLoop) {
            basics.moveRobotEncoder(wheels, 0.4, 24, 480, 12.12);
            firstLoop = false;
        } else {
            basics.update();
            if (basics.isActionInProgress()) {
                return;
            }

            endPhase();
        }
    }

    public void phase4() {
        if (firstLoop) {
            if (blueTeam) {
                basics.moveRobotEncoder(wheels, 0.4, -0.4, -0.4, 0.4, 14, 480, 12.12);
            } else {
                basics.moveRobotEncoder(wheels, -0.4, 0.4, 0.4, -0.4, 14, 480, 12.12);
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
}
