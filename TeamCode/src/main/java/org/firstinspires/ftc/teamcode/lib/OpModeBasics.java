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


    //Wheel group
    public class WheelGroup {
        private DcMotor fr;
        private DcMotor fl;
        private DcMotor br;
        private DcMotor bl;

        public boolean frEncoderReverse;
        public boolean flEncoderReverse;
        public boolean brEncoderReverse;
        public boolean blEncoderReverse;

        //Constructor: sets up all motors
        public WheelGroup(DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl) {
            this.fr = fr;
            this.fl = fl;
            this.br = br;
            this.bl = bl;
        }

        //Power 4 motors
        public void powerWheels(double power) {
            fr.setPower(power);
            fl.setPower(power);
            br.setPower(power);
            bl.setPower(power);
        }

        //Power 2 sides
        public void powerWheels(double pr, double pl) {
            fr.setPower(pr);
            br.setPower(pr);
            fl.setPower(pl);
            bl.setPower(pl);
        }

        //Power all independently
        public void powerWheels(double frPower, double flPower, double brPower, double blPower) {
            fr.setPower(frPower);
            fl.setPower(flPower);
            br.setPower(brPower);
            bl.setPower(blPower);
        }


        //Set 4 positions
        public void setTargetPositions(int position) {
            WheelInts dirs = getIntEncoderDirections();
            fr.setTargetPosition(position * dirs.fr);
            fl.setTargetPosition(position * dirs.fl);
            br.setTargetPosition(position * dirs.br);
            bl.setTargetPosition(position * dirs.bl);
        }

        //Set side positions
        public void setTargetPositions(int pr, int pl) {
            WheelInts dirs = getIntEncoderDirections();
            fr.setTargetPosition(pr * dirs.fr);
            fl.setTargetPosition(pl * dirs.fl);
            br.setTargetPosition(pr * dirs.br);
            bl.setTargetPosition(pl * dirs.bl);
        }

        //Set all positions independently
        public void setTargetPositions(int frTgt, int flTgt, int brTgt, int blTgt) {
            WheelInts dirs = getIntEncoderDirections();
            fr.setTargetPosition(frTgt * dirs.fr);
            fl.setTargetPosition(flTgt * dirs.fl);
            br.setTargetPosition(brTgt * dirs.br);
            bl.setTargetPosition(blTgt * dirs.bl);
        }


        //change mode for all motors
        public void setModes(DcMotor.RunMode mode) {
            fr.setMode(mode);
            fl.setMode(mode);
            br.setMode(mode);
            bl.setMode(mode);
        }


        //return motor powers
        public WheelDoubles getWheelPowers() {
            return new WheelDoubles(fr.getPower(), fl.getPower(), br.getPower(), bl.getPower());
        }

        //return motor target positions
        public WheelInts getWheelTargetPositions() {
            return new WheelInts(fr.getTargetPosition(), fl.getTargetPosition(), br.getTargetPosition(), bl.getTargetPosition());
        }

        //return motor positions
        public WheelInts getWheelCurrentPositions() {
            return new WheelInts(fr.getCurrentPosition(), fl.getCurrentPosition(), br.getCurrentPosition(), bl.getCurrentPosition());
        }


        public WheelInts getIntEncoderDirections() {
            int frDir;
            int flDir;
            int brDir;
            int blDir;

            if (frEncoderReverse) {frDir = -1;} else {frDir = 1;}
            if (flEncoderReverse) {flDir = -1;} else {flDir = 1;}
            if (brEncoderReverse) {brDir = -1;} else {brDir = 1;}
            if (blEncoderReverse) {blDir = -1;} else {blDir = 1;}

            return new WheelInts(frDir, flDir, brDir, blDir);

        }


        //Subclasses
        private class WheelDoubles {
            public double fr;
            public double fl;
            public double br;
            public double bl;

            public WheelDoubles (double fr, double fl, double br, double bl) {
                this.fr = fr;
                this.fl = fl;
                this.br = br;
                this.bl = bl;
            }
        }

        private class WheelInts {
            public int fr;
            public int fl;
            public int br;
            public int bl;

            public WheelInts (int fr, int fl, int br, int bl) {
                this.fr = fr;
                this.fl = fl;
                this.br = br;
                this.bl = bl;
            }
        }

    }


    //Actions \/\/\/\/\/\/\/
    //action class
    private class action {
        private boolean done;
        
        public void execute() {}
        public boolean loop() {return true;}
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



    //Enums \/\/\/\/\/\/\/
    public enum Wheel {
        FR,
        FL,
        BR,
        BL
    }

}
