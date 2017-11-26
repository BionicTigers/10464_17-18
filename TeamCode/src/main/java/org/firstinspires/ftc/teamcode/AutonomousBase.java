package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class AutonomousBase extends OpMode {
    public final double HEADING_TOLERANCE = 7; //tolerance for heading calculations
    public final double DISTANCE_TOLERANCE = 1.0 / 12; //tolerance for heading calculations
    public final double DEGREES_TO_FEET = 3.96 * Math.PI / 1120 / 12;

    //EXPLANATION:
    // (wheel diameter) * pi / (encoder ticks per rotation) /(inches in a foot)
    // This converts encoder ticks into feet.
    //**WARNING** Always calculate distance CHANGED, since encoders have no
    // concept of direction, and we are moving across a 2D plane.

    public static class MoveState   {
        public static final int STOP = 0;
        public static final int FORWARD = 1;
        public static final int BACKWARD = 2;
        public static final int LEFT = 3;
        public static final int RIGHT = 4;
        public static final int TURN_TOWARDS_GOAL = 5;
        public static final int BACKWARD_SLOW = 9;
        public static final int FULL_STOP = 12;
        public static final int STRAFE_TOWARDS_GOAL = 15;
        public static final int TURN_TOWARDS_ANGLE = 16;
        public static final int LEFT_SLOW = 17;
        public static final int RIGHT_SLOW = 18;
        public static final int TURN_TOWARDS_ANGLE_SLOW = 19;
        public static final int DEFINE_COLOR = 901937;  }

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorConveyor;
    Servo servo;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    GyroSensor gyro;

    //We stateful now
    int gameState;
    int moveState;

    double power;
    double heading;
    double desiredAngle;
    boolean turnRight;
    int cDistF, lDistF, dDistF; //Forward distance variables
    int cDistS, lDistS, dDistS; //Sideways distance variables
    int cDistW, lDistW, dDistW; //Sideways distance variables
    double sTime; //Shooting timer
    double pTime; //Button presser timer
    double tDiff;

    int startPos = 6;
    Map map = new Map(startPos); //this map object will allow for easy manipulations.

    final double SCALE_FACTOR = 255;
    float hsvValues[] = {0F, 0F, 0F};

    public void init()  {
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        motorConveyor = hardwareMap.dcMotor.get("conveyor");

        servo = hardwareMap.servo.get("collector");

        I2cAddr colorAddrLeft = I2cAddr.create8bit(0x3C);
        I2cAddr colorAddrRight = I2cAddr.create8bit(0x4C);
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();   }

    public void moveState() {
        // heading = gyro.getHeading();
        switch (moveState)  {
            case MoveState.STOP:
                // Halts all drivetrain movement of the robot
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                break;
            case MoveState.FORWARD:
                // Moves the bot forward at half speed
                power = 1; //power coefficient
                if (map.distanceToGoal() > DISTANCE_TOLERANCE)  {
                    motorFrontRight.setPower(power);
                    motorFrontLeft.setPower(power);
                    motorBackLeft.setPower(power);
                    motorBackRight.setPower(power); }
                break;
            case MoveState.BACKWARD:
                // Moves the bot backwards at half speed
                power = -1; //power coefficient
                if (map.distanceToGoal() > DISTANCE_TOLERANCE)  {
                    motorFrontRight.setPower(power);
                    motorFrontLeft.setPower(power);
                    motorBackLeft.setPower(power);
                    motorBackRight.setPower(power); }
                break;
            case MoveState.BACKWARD_SLOW:
                // Moves the bot backwards at minimum speed
                power = -.2; //power coefficient
                if (map.distanceToGoal() > DISTANCE_TOLERANCE)  {
                    motorFrontRight.setPower(power);
                    motorFrontLeft.setPower(power);
                    motorBackLeft.setPower(power);
                    motorBackRight.setPower(power); }
                //servoLeftButton.setPosition(.5); // HACK
                break;
            case MoveState.LEFT:
                // Moves the bot left at half speed
                power = -1; //power coefficient
                if (map.distanceToGoal() > DISTANCE_TOLERANCE)  {
                    motorFrontRight.setPower(-power);
                    motorFrontLeft.setPower(power);
                    motorBackLeft.setPower(power);
                    motorBackRight.setPower(-power);    }
                break;
            case MoveState.LEFT_SLOW:
                // Moves the bot left at half speed
                power = -.5; //power coefficient
                if (map.distanceToGoal() > DISTANCE_TOLERANCE)  {
                    motorFrontRight.setPower(-power);
                    motorFrontLeft.setPower(power);
                    motorBackLeft.setPower(power);
                    motorBackRight.setPower(-power);    }
                break;
            case MoveState.RIGHT:
                // Moves the bot right at half speed
                power = 1; //power coefficient
                if (map.distanceToGoal() > DISTANCE_TOLERANCE)  {
                    motorFrontRight.setPower(power);
                    motorFrontLeft.setPower(-power);
                    motorBackLeft.setPower(-power);
                    motorBackRight.setPower(power); }
                break;
            case MoveState.RIGHT_SLOW:
                // Moves the bot right at half speed
                power = .5; //power coefficient
                if (map.distanceToGoal() > DISTANCE_TOLERANCE)  {
                    motorFrontRight.setPower(power);
                    motorFrontLeft.setPower(-power);
                    motorBackLeft.setPower(-power);
                    motorBackRight.setPower(power); }
                break;
            case MoveState.STRAFE_TOWARDS_GOAL:
                // Moves the bot towards the goal, while always pointing at desiredAngle
                double P = 1;
                double H = Math.toRadians(heading);
                double Ht = Math.toRadians(map.angleToGoal());

                motorFrontRight.setPower(-P * Math.sin(H - Ht));
                motorFrontLeft.setPower(-P * Math.sin(H - Ht));
                motorBackLeft.setPower(P * Math.cos(H - Ht));
                motorBackRight.setPower(P * Math.cos(H - Ht));
                break;
            case MoveState.TURN_TOWARDS_GOAL:
                // Orients the bot to face the goal
                power = .25;
                if (heading <= 180) {
                    turnRight = heading <= map.angleToGoal() && heading + 180 >= map.angleToGoal(); }
                else    {
                    turnRight = !(heading >= map.angleToGoal() && heading - 180 <= map.angleToGoal());  }

                if (turnRight)  {
                    motorFrontRight.setPower(power);
                    motorFrontLeft.setPower(-power);
                    motorBackLeft.setPower(-power);
                    motorBackRight.setPower(power); }
                else    {
                    motorFrontRight.setPower(-power);
                    motorFrontLeft.setPower(power);
                    motorBackLeft.setPower(power);
                    motorBackRight.setPower(-power);    }

                break;
            case MoveState.TURN_TOWARDS_ANGLE:
                // Orients the bot to face at desiredAngle.
                power = .3;
                if (heading <= 180) {
                    turnRight = heading <= desiredAngle && heading + 180 >= desiredAngle;   }
                else    {
                    turnRight = !(heading >= desiredAngle && heading - 180 <= desiredAngle);    }

                if (turnRight)  {
                    motorFrontRight.setPower(power);
                    motorFrontLeft.setPower(-power);
                    motorBackLeft.setPower(power);
                    motorBackRight.setPower(-power);    }
                else    {
                    motorFrontRight.setPower(-power);
                    motorFrontLeft.setPower(power);
                    motorBackLeft.setPower(power);
                    motorBackRight.setPower(-power);    }
                break;
            case MoveState.DEFINE_COLOR:
                Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues); }

        //break;
            /*case MoveState.TURN_TOWARDS_ANGLE_SLOW:
                // Orients the bot to face at desiredAngle.
                power = .2;
                if(heading<=180){
                    turnRight = heading <= desiredAngle && heading + 180 >= desiredAngle;
                }else{
                    turnRight = !(heading >= desiredAngle && heading - 180 <= desiredAngle);
                }

                if(turnRight){
                    motorFrontRight.setPower(power);
                    motorFrontLeft.setPower(-power);
                    motorBackLeft.setPower(-power);
                    motorBackRight.setPower(power);
                }else{
                    motorFrontRight.setPower(-power);
                    motorFrontLeft.setPower(power);
                    motorBackLeft.setPower(power);
                    motorBackRight.setPower(-power);
                }
                break;*/

       // case MoveState.FULL_STOP:
        // Stop ALL robot movement, and resets servo to default pos

        servo.setPosition(.5);
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorConveyor.setPower(0);  }

        // break;

    //  map.moveRobot(dDistS * DEGREES_TO_FEET, (heading+90%360));
    // map.moveRobot(dDistF * DEGREES_TO_FEET, heading);
//}

    public void gameState() {
        //heading = gyro.getHeading();

        lDistF = cDistF;
        cDistF = ( motorBackLeft.getCurrentPosition()
                 + motorBackRight.getCurrentPosition()) / 2;
        dDistF = cDistF - lDistF;

        lDistS = cDistS;
        cDistS = ( motorFrontRight.getCurrentPosition()
                 + motorFrontLeft.getCurrentPosition()) / 2;
        dDistW = cDistW - lDistW;

        if(tDiff == 0){
            tDiff = getRuntime();   }   }

    public void telemetry() {
        telemetry.addData("angle to goal ",map.angleToGoal());
        telemetry.addData("Runtime ",getRuntime());

        telemetry.addData("dist from goal ",map.distanceToGoal());
        telemetry.addData("goal (x,y) ","(" +
          map.getGoalX() + "," + 
          map.getGoalY() + ")");
        telemetry.addData("Robot(x,y) ","(" +
          map.getRobotX() + "," + 
          map.getRobotY() + ")");
        telemetry.addData("robot theta",heading);
        telemetry.addData("Am I lined up?", linedUpAngle(5));
        telemetry.addData("Desired Angle", desiredAngle);
        telemetry.addData("moveState", moveState);
        telemetry.addData("gameState", gameState);  }

    @Override
    public void loop()  {
        gameState();
        moveState();
        telemetry();    }
    public boolean linedUp()    {
        if (Math.abs(heading - map.angleToGoal()) < HEADING_TOLERANCE || (heading > 360 - HEADING_TOLERANCE && map.angleToGoal() < HEADING_TOLERANCE || (heading < HEADING_TOLERANCE && map.angleToGoal() > 360 - HEADING_TOLERANCE))) {
            return true;    }
        else    {
            return false;   }   }
    public boolean linedUpAngle()   {
        if (Math.abs(heading - desiredAngle) < HEADING_TOLERANCE || (heading > 360 - HEADING_TOLERANCE && desiredAngle < HEADING_TOLERANCE || (heading < HEADING_TOLERANCE && desiredAngle > 360 - HEADING_TOLERANCE))) {
            return true;    }
        else    {
            return false;   }   }

    public boolean linedUpAngle(int HEADING_TOLERANCE) {
        if (Math.abs(heading - desiredAngle) < HEADING_TOLERANCE || (heading > 360 - HEADING_TOLERANCE && desiredAngle < HEADING_TOLERANCE || (heading < HEADING_TOLERANCE && desiredAngle > 360 - HEADING_TOLERANCE))) {
            return true;    }
        else    {
            return false;   }   }

    public boolean linedUpRev() {
        if (Math.abs(heading - map.angleToGoalRev()) < HEADING_TOLERANCE || (heading > 360 - HEADING_TOLERANCE && map.angleToGoalRev() < HEADING_TOLERANCE || (heading < HEADING_TOLERANCE && map.angleToGoalRev() > 360 - HEADING_TOLERANCE))) {
            return true;    }
        else    {
            return false;   }    }

    public double actualRuntime() {
        return getRuntime() - tDiff;    }   }