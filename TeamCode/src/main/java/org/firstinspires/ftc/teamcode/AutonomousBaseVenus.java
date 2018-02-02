package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

import static org.firstinspires.ftc.teamcode.Map.angleToGoal;
import static org.firstinspires.ftc.teamcode.Map.distanceToGoal;
import static org.firstinspires.ftc.teamcode.Map.moveRobot;


public abstract class AutonomousBaseVenus extends LinearOpMode {

    public final double HEADING_TOLERANCE = 7; //tolerance for heading calculations
    public final double DISTANCE_TOLERANCE = 1.0 / 10; //tolerance for heading calculations
    public final double DEGREES_TO_FEET = 3.96 * Math.PI / 1120 / 12;

    //EXPLANATION:
    // (wheel diameter) * pi / (encoder ticks per rotation) /(inches in a foot)
    // This converts encoder ticks into feet.
    //**WARNING** Always calculate distance CHANGED, since encoders have no
    // concept of direction, and we are moving across a 2D plane.

    public static class MoveState {

        public static final int STOP = 0;
        public static final int FORWARD = 1;
        public static final int BACKWARD = 2;
        public static final int LEFT = 3;
        public static final int RIGHT = 4;
        public static final int TURN_TOWARDS_GOAL = 5;
        public static final int STRAFE_TOWARDS_GOAL = 15;
        public static final int TURN_TOWARDS_ANGLE = 16;
        public static final int FULL_STOP = 17;
    }

    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor evangelino;
    public DcMotor wilbert;
    public static BNO055IMU imu;


    public int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    public VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    public ElapsedTime runtime = new ElapsedTime();

    public Orientation angles;
    public double heading;
    public double desiredAngle;
    public double tDiff;
    public double power;

    public int moveState;
    public int gameState;
    public int cDistF, lDistF, dDistF; //Forward distance variables
    public int cDistS, lDistS, dDistS; //Sideways distance variables
    public int cDistW, lDistW, dDistW; //Sideways distance variables
    public int startPos = 6;
    public int waitTime;

    boolean turnRight;
    double formatAngle;
    Map map = new Map(startPos);

    @Override
    public void runOpMode() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        heading = angles.firstAngle;

        this.desiredAngle = 90;

        Servo eddie = hardwareMap.servo.get("eddie");
        Servo clark = hardwareMap.servo.get("clark");
        ColorSensor roger = hardwareMap.colorSensor.get("roger");
        ColorSensor leo = hardwareMap.colorSensor.get("leo");


        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        imu = (BNO055IMU) hardwareMap.i2cDevice.get("imu");
        Servo hamilton = hardwareMap.servo.get("hamilton");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        time = getRuntime();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                motorFrontLeft.getCurrentPosition(),
                motorBackRight.getCurrentPosition(),
                motorFrontRight.getCurrentPosition(),
                motorBackLeft.getCurrentPosition());

        telemetry.update();

        // can help in debugging; otherwise not necessary

        //telemetry.addData(">", "Press Play to start");
        //telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AfBkGLH/////AAAAGUUS7r9Ue00upoglw/0yqTBLwhqYHpjwUK9zxmWMMFGuNGPjo/RjNOTsS8POmdQLHwe3/75saYsyb+mxz6p4O8xFwDT7FEYMmKW2NKaLKCA2078PZgJjnyw+34GV8IBUvi2SOre0m/g0X5eajpAhJ8ZFYNIMbUfavjQX3O7P0UHyXsC3MKxfjMzIqG1AgfRevcR/ONOJlONZw7YIZU3STjODyuPWupm2p7DtSY4TRX5opqFjGQVKWa2IlNoszsN0szgW/xJ1Oz5VZp4oDRS8efG0jOq1QlGw7IJOs4XXZMcsk0RW/70fVeBiT+LMzM8Ih/BUxtVVK4pcLMpb2wlzdKVLkSD8LOpaFWmgOhxtNz2M";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        telemetry.addData("vuMark", vuMark);
        waitForStart();

        while(opModeIsActive()){
            calibrateIMU();
            gameState();
            moveState();
        }
    }


        public void moveState() {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu.initialize(parameters);

            BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
            String filename = "BNO055IMUCalibration.json";
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, calibrationData.serialize());
            telemetry.log().add("saved to '%s'", filename);


            switch (moveState) {
                case MoveState.STOP:
                    // Halts all drivetrain movement of the robot
                    motorFrontRight.setPower(0);
                    motorFrontLeft.setPower(0);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                    break;

                case MoveState.FORWARD:
                    // Moves the bot forward at half speed
                    power = .75;

                    if (distanceToGoal() > DISTANCE_TOLERANCE) {
                        motorFrontRight.setPower(power);
                        motorFrontLeft.setPower(power);
                        motorBackLeft.setPower(power);
                        motorBackRight.setPower(power);
                    }
                    break;

                case MoveState.BACKWARD:
                    // Moves the bot backwards at half speed
                    power = .75; //power coefficient
                    if (distanceToGoal() > DISTANCE_TOLERANCE) {
                        motorFrontRight.setPower(-power);
                        motorFrontLeft.setPower(-power);
                        motorBackLeft.setPower(-power);
                        motorBackRight.setPower(-power);
                    }
                    break;

                case MoveState.LEFT:
                    // Moves the bot left at half speed
                    power = .75; //power coefficient
                    if (distanceToGoal() > DISTANCE_TOLERANCE) {
                        motorFrontRight.setPower(power);
                        motorFrontLeft.setPower(-power);
                        motorBackLeft.setPower(power);
                        motorBackRight.setPower(-power);
                    }
                    break;

                case MoveState.RIGHT:
                    // Moves the bot right at half speed
                    power = .75; //power coefficient
                    if (distanceToGoal() > DISTANCE_TOLERANCE) {
                        motorFrontRight.setPower(-power);
                        motorFrontLeft.setPower(power);
                        motorBackLeft.setPower(-power);
                        motorBackRight.setPower(power);
                    }
                    break;


                case MoveState.STRAFE_TOWARDS_GOAL:
                    // Moves the bot towards the goal, while always pointing at desiredAngle
                    double P = .50;
                    double H = Math.toRadians(angles.firstAngle);
                    double Ht = Math.toRadians(angleToGoal());

                    motorFrontRight.setPower(-P * Math.sin(H - Ht));
                    motorFrontLeft.setPower(-P * Math.sin(H - Ht));
                    motorBackLeft.setPower(P * Math.cos(H - Ht));
                    motorBackRight.setPower(P * Math.cos(H - Ht));
                    break;

                case MoveState.TURN_TOWARDS_GOAL:
                    // Orients the bot to face the goal
                    power = .50;
                    boolean turnRight;
                    if (heading <= 180) {
                        turnRight = heading <= angleToGoal() && heading + 180 >= angleToGoal();
                    } else {
                        turnRight = !(heading >= angleToGoal() && heading - 180 <= angleToGoal());
                    }

                    if (turnRight) {
                        motorFrontRight.setPower(power);
                        motorFrontLeft.setPower(-power);
                        motorBackLeft.setPower(-power);
                        motorBackRight.setPower(power);
                    } else {
                        motorFrontRight.setPower(-power);
                        motorFrontLeft.setPower(power);
                        motorBackLeft.setPower(power);
                        motorBackRight.setPower(-power);
                    }
                    break;

                case MoveState.TURN_TOWARDS_ANGLE:
                    // Orients the bot to face at desiredAngle.
                    power = .50;
                    if (heading <= 180) {
                        turnRight = heading <= desiredAngle && heading + 180 >= desiredAngle;
                    } else {
                        turnRight = !(heading >= desiredAngle && heading - 180 <= desiredAngle);
                    }

                    if (turnRight) {
                        motorFrontRight.setPower(power);
                        motorFrontLeft.setPower(-power);
                        motorBackLeft.setPower(power);
                        motorBackRight.setPower(-power);
                    } else {
                        motorFrontRight.setPower(-power);
                        motorFrontLeft.setPower(power);
                        motorBackLeft.setPower(power);
                        motorBackRight.setPower(-power);
                    }
                    break;
                case MoveState.FULL_STOP:
                    // Stop ALL robot movement, and resets servo to default pos

                    motorFrontLeft.setPower(0);
                    motorFrontRight.setPower(0);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                    break;
            }


            moveRobot(dDistS * DEGREES_TO_FEET, (heading+90%360));
            moveRobot(dDistF * DEGREES_TO_FEET, heading);
    }


        public void gameState(){
            heading = angles.firstAngle;

            lDistF = cDistF;
            cDistF = ( motorBackLeft.getCurrentPosition()
                    + motorBackRight.getCurrentPosition()
            ) / 2;
            dDistF = cDistF - lDistF;

            lDistS = cDistS;
            cDistS = ( motorFrontLeft.getCurrentPosition()
                    + motorFrontRight.getCurrentPosition()
            ) / 2;
            dDistS = cDistS - lDistS;

            lDistW = cDistW;

            dDistW = cDistW - lDistW;

            if(tDiff == 0){
                tDiff = getRuntime();
            }
        }


    public static void calibrateIMU() {
        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();

        String filename = "AdafruitIMUCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());
    }


    public void telemetry() {
        telemetry.addData("angle to goal ", map.angleToGoal());
        telemetry.addData("Runtime ", getRuntime());

        telemetry.addData("dist from goal ", map.distanceToGoal());
        telemetry.addData("goal (x,y) ", "(" +
                map.getGoalX() + "," +
                map.getGoalY() + ")");
        telemetry.addData("Robot(x,y) ", "(" +
                map.getRobotX() + "," +
                map.getRobotY() + ")");
        telemetry.addData("robot theta", heading);
        telemetry.addData("lined up?", linedUpAngle(5));
        telemetry.addData("desired angle", desiredAngle);
        telemetry.addData("moveState", moveState);
    }


    public boolean linedUp() {
        return Math.abs(heading - map.angleToGoal()) < HEADING_TOLERANCE || (heading > 360 - HEADING_TOLERANCE && map.angleToGoal() < HEADING_TOLERANCE || (heading < HEADING_TOLERANCE && map.angleToGoal() > 360 - HEADING_TOLERANCE));
    }

    public boolean linedUpAngle() {
        return Math.abs(heading - desiredAngle) < HEADING_TOLERANCE || (heading > 360 - HEADING_TOLERANCE && desiredAngle < HEADING_TOLERANCE || (heading < HEADING_TOLERANCE && desiredAngle > 360 - HEADING_TOLERANCE));
    }

    public boolean linedUpAngle(int HEADING_TOLERANCE) {
        return Math.abs(heading - desiredAngle) < HEADING_TOLERANCE || (heading > 360 - HEADING_TOLERANCE && desiredAngle < HEADING_TOLERANCE || (heading < HEADING_TOLERANCE && desiredAngle > 360 - HEADING_TOLERANCE));
    }

    public boolean linedUpRev() {
        return Math.abs(heading - map.angleToGoalRev()) < HEADING_TOLERANCE || (heading > 360 - HEADING_TOLERANCE && map.angleToGoalRev() < HEADING_TOLERANCE || (heading < HEADING_TOLERANCE && map.angleToGoalRev() > 360 - HEADING_TOLERANCE));
    }
}







