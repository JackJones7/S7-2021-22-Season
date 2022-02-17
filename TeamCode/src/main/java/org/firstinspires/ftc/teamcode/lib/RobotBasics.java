//Motor order: fr, fl, br, bl
package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.robotcore.external.navigation.Axis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


public class RobotBasics {
    
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
    private Action currentAction;
    private boolean actionInProgress;
    
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
    public RobotBasics(DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl, BNO055IMU imu) {
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

        //No action running
        actionInProgress = false;
    }
    
    //Other Constructor
    public RobotBasics(DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl) {
        //This will take four motors
        this.fr = fr;
        this.fl = fl;
        this.br = br;
        this.bl = bl;
    
        //Only has motors
        hasMotors = true;
        hasImu = false;

        //I hope nothing explodes
        actionInProgress = false;
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
            actionInProgress = true;
            
        }
// ------------


//Move robot commands

    //Move robot with imu (distance, power, axis)
    public void moveRobotImu(double dist, double power, Axis axis) {
        
        //set current action to new move action with specified params
        currentAction = new MoveRobot(dist, power, axis);
        
        //start it off
        currentAction.execute();
        actionInProgress = true;
        
    }

    //Move robot with encoder (wheel group, power, dist, tpr, circumference)
    public void moveRobotEncoder(WheelGroup wheels, double power, double dist, int tpr, double circumference) {
        currentAction = new MoveRobotEncoder(wheels, power, dist, tpr, circumference);
        currentAction.execute();
        actionInProgress = true;
    }

    //Move robot with encoder (wheel group, right power, left power, dist, tpr, circumference)
    public void moveRobotEncoder(WheelGroup wheels, double pr, double pl, double dist, int tpr, double circumference) {
        currentAction = new MoveRobotEncoder(wheels, pr, pl, dist, tpr, circumference);
        currentAction.execute();
        actionInProgress = true;
    }

    //Move robot with encoder (wheel group, fr power, fl power, br power, bl power, dist, tpr, circumference)
    public void moveRobotEncoder(WheelGroup wheels, double frPower, double flPower,
                                 double brPower, double blPower, double dist, int tpr, double circumference) {
        currentAction = new MoveRobotEncoder(wheels, frPower, flPower, brPower, blPower, dist, tpr, circumference);
        currentAction.execute();
        actionInProgress = true;
    }
    
    
//Inches -> ticks converter
    
    public int inchToTick(double inch, int tpr, double circ) {
        return Math.toIntExact(Math.round(inch/circ * tpr));
    }


//Is action in progress?
    public boolean isActionInProgress() {
        return actionInProgress;
    }

        
// -Function for each loop-
    
    public void update() {
        if (actionInProgress) {
            if (currentAction.loop()) {
                actionInProgress = false;
            }
        }
    }
        
// ------------------------


    //Wheel group
    public class WheelGroup {
        public DcMotor fr;
        public DcMotor fl;
        public DcMotor br;
        public DcMotor bl;

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
        public void setPower(double power) {
            fr.setPower(power);
            fl.setPower(power);
            br.setPower(power);
            bl.setPower(power);
        }

        //Power 2 sides
        public void setPower(double pr, double pl) {
            fr.setPower(pr);
            br.setPower(pr);
            fl.setPower(pl);
            bl.setPower(pl);
        }

        //Power all independently
        public void setPower(double frPower, double flPower, double brPower, double blPower) {
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
        public WheelDoubles getPowers() {
            return new WheelDoubles(fr.getPower(), fl.getPower(), br.getPower(), bl.getPower());
        }

        //return motor target positions
        public WheelInts getTargetPositions() {
            return new WheelInts(fr.getTargetPosition(), fl.getTargetPosition(), br.getTargetPosition(), bl.getTargetPosition());
        }

        //return motor positions
        public WheelInts getCurrentPositions() {
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
        public class WheelDoubles {
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

        public class WheelInts {
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

    //Create and return Wheel Group
    public WheelGroup createWheelGroup(DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl) {
        return new WheelGroup(fr, fl, br, bl);
    }




    //Actions \/\/\/\/\/\/\/
    //action class
    private abstract class Action {
        private boolean done;
        
        public void execute() {}
        public boolean loop() {return true;}
    }


    public class MoveRobotEncoder extends Action {

        private WheelGroup wheels;

        private double dist;
        private int tpr;
        private double circumference;

        private double frPower;
        private double flPower;
        private double brPower;
        private double blPower;

        private int ticks;

        private int baseTarget;
        private int frTgt;
        private int flTgt;
        private int brTgt;
        private int blTgt;

        private boolean frDone = false;
        private boolean flDone = false;
        private boolean brDone = false;
        private boolean blDone = false;

        private DcMotor.RunMode startMode;

        //constructors (all have wheel group, dist, tpr, circumference)
        //Move all simultaneously
        public MoveRobotEncoder(WheelGroup wheels, double power, double dist,
                                int tpr, double circumference) {
            this.wheels = wheels;

            this.frPower = power;
            this.flPower = power;
            this.brPower = power;
            this.blPower = power;

            this.dist = dist;
            this.tpr = tpr;
            this.circumference = circumference;

            this.startMode = wheels.fr.getMode();
        }

        //Move both sides independently
        public MoveRobotEncoder(WheelGroup wheels, double pr, double pl, double dist,
                                int tpr, double circumference) {
            this.wheels = wheels;

            this.frPower = pr;
            this.flPower = pl;
            this.brPower = pr;
            this.blPower = pl;

            this.dist = dist;
            this.tpr = tpr;
            this.circumference = circumference;

            this.startMode = wheels.fr.getMode();
        }

        //Move all independently
        public MoveRobotEncoder(WheelGroup wheels, double fr, double fl, double br,
                                double bl, double dist, int tpr, double circumference) {
            this.wheels = wheels;

            this.frPower = fr;
            this.flPower = fl;
            this.brPower = br;
            this.blPower = bl;

            this.dist = dist;
            this.tpr = tpr;
            this.circumference = circumference;

            this.startMode = wheels.fr.getMode();
        }


        //execute function
        @Override
        public void execute() {
            //store abs of dist -> ticks
            wheels.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            baseTarget = inchToTick(dist, tpr, circumference);

            WheelGroup.WheelInts positions = wheels.getCurrentPositions();

            //Set targets for all motors
            frTgt = Math.abs(baseTarget + positions.fr);
            flTgt = Math.abs(baseTarget + positions.fl);
            brTgt = Math.abs(baseTarget + positions.br);
            blTgt = Math.abs(baseTarget + positions.bl);

            //power motors
            wheels.setPower(frPower, flPower, brPower, blPower);
            //(if motors are in run to position mode, set motor positions to targets)
        }

        @Override
        public boolean loop() {
            //loop function
            wheels.setModes(startMode);

            WheelGroup.WheelInts positions = wheels.getCurrentPositions();
            if (Math.abs(positions.fr) >= frTgt) {
                wheels.fr.setPower(0);
                frDone = true;
            } if (Math.abs(positions.fl) >= flTgt) {
                wheels.fl.setPower(0);
                flDone = true;
            } if (Math.abs(positions.br) >= brTgt) {
                wheels.br.setPower(0);
                brDone = true;
            } if (Math.abs(positions.bl) >= blTgt) {
                wheels.bl.setPower(0);
                blDone = true;
            }

            if (frDone && flDone && brDone && blDone) {
                return true;
            }

            return false;

           // } else {

           //     return true;

            //}

        }

    }
    
    
    //action class children
    public class Turn extends Action {
        
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
    
    public class MoveRobot extends Action {
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
