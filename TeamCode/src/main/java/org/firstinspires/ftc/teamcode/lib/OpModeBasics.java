//Motor order: fr, fl, br, bl
package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class OpModeBasics {
    
//functionality variables >>
    //define variables used for alignment correction
    private double correctAngle;
    
    //Variables for whether OMB has access to hardware
    private boolean hasMotors;
    private boolean hasImu;
    
    //Variables for actions
    private int actionId;
    private double actionVal;
    //soon to be obsolete hopefully, hence this fancy new sub class:
    private action currentAction;
    
    //This might be useful
    
//----------------------- <<
   
//Hardware variables >>

    //Define variables for motors
    private DcMotor fr;
    private DcMotor fl;
    private DcMotor br;
    private DcMotor bl;

    //Variables for encoder directions
    public EncoderDirection frEncoderDir = EncoderDirection.FORWARD;
    public EncoderDirection flEncoderDir = EncoderDirection.FORWARD;
    public EncoderDirection brEncoderDir = EncoderDirection.FORWARD;
    public EncoderDirection blEncoderDir = EncoderDirection.FORWARD;
    
    //Define imu variable
    private BNO055IMU imu;
    
//------------------ <<


    
    //Constructor
    public OpModeBasics(DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl, BNO055IMU imu) {
        //This will take four motors
        this.fr = fr;
        this.fl = fl;
        this.br = br;
        this.bl = bl;
        //It will also take the imu info.
        this.imu = imu;
    
        //has access to all hardware
        hasMotors = true;
        hasImu = true;
    }
    
    //Other Constructor
    public OpModeBasics(DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl) {
        //This will take four motors
        this.fr = fr;
        this.fl = fl;
        this.br = br;
        this.bl = bl;
    
        //Only has motors
        hasMotors = true;
        hasImu = false;
    }

    //Encoder Direction Enum
    public enum EncoderDirection {
        FORWARD,
        REVERSE
    }
    
// -Basic power motors commands-
    
    //power motors (double power)
    public void powerMotors(double power) {
        //Does basics have motors?
        if (hasMotors) {
            //if yes, set all 4 motors to power variable
            fr.setPower(power);
            fl.setPower(power);
            br.setPower(power);
            bl.setPower(power);
        }
    }
    
    //Overload - power motors (double pr, double pl)
    public void powerMotors(double pr, double pl) {
        //does basics have motors?
        if (hasMotors) {
            //set right motors to pr
            fr.setPower(pr);
            br.setPower(pr);
            //set left motors to pl
            fl.setPower(pl);
            bl.setPower(pl);
        }
    }
    
    //Overload - power motors (double frPower, flPower, brPower, blPower)
    public void powerMotors(double frPower, double flPower, double brPower, double blPower) {
    //does Basics have motors?
        if (hasMotors) {
            //set each motor to respective power variable
            fr.setPower(frPower);
            fl.setPower(flPower);
            br.setPower(brPower);
            bl.setPower(blPower);
        }
    }
    
// ----------------------------
        
        
// -Move robot with imu alignment correction-
        
    //Move with correction (double power)
    //set current angle variable to current imu angle
    //set all motors to power
    
    //Similar overloads to setPower
    
    //Cancel correction
    //cancel correction automatically when other power cmnd is used as well
        
// ------------------------------------------
        
    
// -Turn robot-
        public void turnRobot(double degrees, double power) {
            /*
            //Check if Basics has Motors and Imu
            if (hasMotors && hasImu && degrees != 0) {
                //Set action Id to 1 for turning
                actionId = 1;
                //Set action val to current imu angle + target degrees
                actionVal = getAngle() - degrees;
                
                //If degrees is negative, turn right
                if (degrees > 0) {
                    powerMotors(power * -1, power);
                } else {
                    //Otherwise, turn left
                    powerMotors(power, power * -1);
                }
            }
            */
            
            //set current action to turning, with specified parameters
            currentAction = new Turn(degrees, power);
            //run execute method
            currentAction.execute();
            
        }
// ------------


//Move robot commands

    //Move robot with imu (distance, power, axis)
    public void moveRobotImu(double dist, double power, Axis axis) {
        
        //set current action to new move action with specified params
        currentAction = new MoveRobot(dist, power, axis);
        
        //start it off
        currentAction.execute();
        
    }
    
    
//Inches -> ticks converter
    
    public int inchToTick(double inch, int tpr, double circ) {
        return Math.toIntExact(Math.round(inch/circ * tpr));
    }
        
        
// -Function for each loop-
    
    public void basicsUpdate() {
        //turn towards correctAngle, unless correctAngle > 360
        
        /*
        //If action Id is 1...
        if (actionId == 1) {
            //if fr power > 0...
            if (fl.getPower() > 0) {
                //if imu angle > action val, stop action
                if (getAngle() < actionVal) {
                    powerMotors(0);
                    actionId = 0;
                    actionVal = 0;
                }
            } else {
                if (getAngle() > actionVal) {
                    powerMotors(0);
                    actionId = 0;
                    actionVal = 0;
                }
            }
        }
        */
        if (currentAction.loop()) {
            currentAction = new action();
        }
    }
        
// ------------------------



    
    //action class
    private class action {
        private boolean done;
        
        public void execute() {}
        public boolean loop() {return true;}
    }


    public class MoveRobotEncoder {

        private double dist;
        private int tpr;
        private double circumference;

        private double frPower;
        private double flPower;
        private double brPower;
        private double blPower;

        private int ticks;

        private int frTgt;
        private int flTgt;
        private int brTgt;
        private int blTgt;


        //constructors (all have dist, tpr, circumference)
        //Move all simultaneously

        //Move both sides independently

        //Move all independently


        //execute function
        //calculate dist -> ticks
        //set motor target positions depending on motor power and encoder direction
        //power motors
        //(if motors are in run to position mode, set motor positions to targets)

        //loop function
        //if motors are in run to position mode, check if busy
        //if not busy, power motors off and return true
        //if motors are in other mode, check if they're past their target
        //if past target, power motors off and return true
        //otherwise, return false
    }
    
    
    //action class children
    public class Turn extends action {
        
        Orientation globalAngle;
        Orientation lastAngles;
        
        private double tgtAngle = 0;
        private double power = 0;
        private double realAngle = 0;
        
        
        public Turn (double tgtAngle, double power) {
            this.tgtAngle = tgtAngle;
            this.power = power;
            
            globalAngle = new Orientation();
            lastAngles = new Orientation();
        }
        
        public void execute() {
            //Check if Basics has Motors and Imu
            if (hasMotors && hasImu && tgtAngle != 0) {
                //Set target angle to current imu angle + target degrees
                realAngle = getAngle().thirdAngle + tgtAngle;
                
                //If degrees is negative, turn left
                if (tgtAngle > 0) {
                    powerMotors(power, power * -1);
                } else {
                    //Otherwise, turn right
                    powerMotors(power * -1, power);
                }
            }
        }
        
        public boolean loop() {
            //if fr power > 0...
            if (fr.getPower() > 0) {
                //if imu angle <= tgt angle, stop action
                if (getAngle().thirdAngle <= realAngle) {
                    powerMotors(0);
                    return true;
                }
            } else {
                if (getAngle().thirdAngle >= realAngle) {
                    powerMotors(0);
                    return true;
                }
            }
            return false;
        }
        
        //getAngle method

        public Orientation getAngle() {
        
            //This method is "borrowed" from an example by StemRobotics.
            //https://github.com/stemrobotics/Tetrix-Exercises/blob/master/DriveAvoidImu.java
            //line 149
            //Thanks, man
        
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
            double deltaAngleZ = angles.firstAngle - lastAngles.firstAngle;
            double deltaAngleY = angles.secondAngle - lastAngles.secondAngle;
            double deltaAngleX = angles.thirdAngle - lastAngles.thirdAngle;
        
        
            if (deltaAngleX < -180) {
                deltaAngleX += 360;
            } else if (deltaAngleX > 180) {
                deltaAngleX -= 360;
            }
            if (deltaAngleY < -180) {
                deltaAngleY += 360;
            } else if (deltaAngleX > 180) {
                deltaAngleY -= 360;
            }
            if (deltaAngleZ < -180) {
                deltaAngleZ += 360;
            } else if (deltaAngleX > 180) {
                deltaAngleZ -= 360;
            }
        
            globalAngle.firstAngle += deltaAngleX;
            globalAngle.secondAngle += deltaAngleY;
            globalAngle.thirdAngle += deltaAngleZ;
            
            lastAngles = angles;
        
            return globalAngle;
        
        }
    }
    
    public class MoveRobot extends action {
        private double tgtDist;
        private Axis axis;
        private Position startPos;
        private Position endPos;
        
        private double movedDist;
        
        private double frPower;
        private double flPower;
        private double brPower;
        private double blPower;
        
        public MoveRobot(double tgtDist, double power, Axis axis) {
            this.tgtDist = tgtDist;
            this.frPower = power;
            this.flPower = power;
            this.brPower = power;
            this.blPower = power;
        }
        
        public void execute() {
            //get the starting position
            startPos = imu.getPosition();
            //power motors to specified power
            powerMotors(frPower, flPower, brPower, blPower);
        }
        
        public boolean loop() {
            //get travelled dist according to specified axis
            endPos = imu.getPosition();
            if (axis == Axis.X) {
                movedDist = endPos.x - startPos.x;
            } else if (axis == Axis.Y) {
                movedDist = endPos.y - startPos.y;
            } else if (axis == Axis.Z) {
                movedDist = endPos.z - startPos.z;
            } else {
                //what the heck just happened
                //I quit
                powerMotors(0);
                return true;
            }
            
            //if travelled dist >= target dist, end action
            if (movedDist >= tgtDist) {
                powerMotors(0);
                return true;
            }
            
            //otherwise, the action is still in progress
            return false;
        }
    }
    
}
