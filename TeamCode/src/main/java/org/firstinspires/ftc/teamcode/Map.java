package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Controls all map logic for autonomous
 */
@Autonomous(name="Map")

public abstract class Map extends LinearOpMode {

    public static DcMotor motorFrontLeft;
    public static DcMotor motorBackRight;
    public static DcMotor motorFrontRight;
    public static DcMotor motorBackLeft;

    static double goalX = 0;
    static double goalY = 0;
    static double curX = 0;
    static double curY = 0;


    @Override
    public void runOpMode() {
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("init", "complete");
    }


    public static double revTicks = 173.333333333333333;

    /**
     * Precondition: Must call {@code setRobot} first
     */
    public static void setGoal(double x, double y) {
        //somechthing is returning null
        goalX = x;
        goalY = y;

        updateXY();
    }

    public static void setRobot(double x, double y) {
        curX = x;
        curY = y;
    }

    private static void updateXY() {
        y = goalY - curY;
        x = goalX - curX;
    }

    public static double x = 0;
    public static double y = 0;

    public static double strafeDist;

    public static void updateStrafeDist() {
        strafeDist = Math.sqrt(x * x + y * y) * revTicks;

    }


    public static void driveToGoal(){

        motorBackRight.setTargetPosition((int)strafeDist);
        motorFrontRight.setTargetPosition((int)strafeDist);
        motorBackLeft.setTargetPosition((int)strafeDist);
        motorFrontLeft.setTargetPosition((int)strafeDist);

        if(Math.abs(x) > Math.abs(y)){
            y = y/x;
            x = x/x;
         } else {
            x = x/y;
            y = y/y;
        }

        double P = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x);
        double sinRAngle = Math.sin(robotAngle);
        double cosRAngle = Math.cos(robotAngle);

        final double v1 = (P * sinRAngle) + (P * cosRAngle);
        final double v2 = (P * sinRAngle) - (P * cosRAngle);
        final double v3 = (P * sinRAngle) - (P * cosRAngle);
        final double v4 = (P * sinRAngle) + (P * cosRAngle);

        motorFrontLeft.setPower (v1);
        motorBackRight.setPower (v2);
        motorBackLeft.setPower  (v3);
        motorFrontRight.setPower(v4);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motorBackRight.isBusy() && motorFrontRight.isBusy() && motorBackLeft.isBusy() && motorFrontLeft.isBusy())  {
        }
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


}







//
//    public double getGoalX(){
//        return goalX;
//    }
//
//    public double getGoalY(){
//        return goalY;
//    }
//
//    public double getRobotX(){
//        return ((int)(robotX*1000))/1000.0;
//    }
//
//    public double getRobotY(){
//        return ((int)(robotY*1000))/1000.0;
//    }
//
//    public static double angleToGoal(){
//        double dX = goalX-robotX;
//        double dY = goalY-robotY;
//        return (((Math.atan2(dY, dX) * 180) / Math.PI) + 450) % 360;
//    }
//
//    public static double distanceToGoal(){
//        double dX = goalX-robotX;/
//        double dY = goalY-robotY;
//        return Math.sqrt(dX * dX + dY * dY); //return length of hypotenuse
//    }
//
//    public static void moveRobot(double feet,double heading) {
//        robotX -= feet * Math.cos(Math.toRadians((heading + 450) % 360));
//        robotY -= feet * Math.sin(Math.toRadians((heading + 450) % 360));
//    }
//
//    public double angleToGoalRev() {
//        return ((angleToGoal() + 180) % 360);
//    }
//
//}
