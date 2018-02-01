package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

import static org.firstinspires.ftc.teamcode.Map.angleToGoal;
import static org.firstinspires.ftc.teamcode.Map.distanceToGoal;


@Autonomous(name="Red Front", group ="Red")

public class VenusAutoRedFront extends LinearOpMode {


    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    public VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    private BNO055IMU imu;

    private int moveState;
    private double heading;
    private double desiredAngle;
    public int cDistF, lDistF, dDistF; //Forward distance variables
    public int cDistS, lDistS, dDistS; //Sideways distance variables
    public int cDistW, lDistW, dDistW; //Sideways distance variables
    public ElapsedTime runtime = new ElapsedTime();

    public int startPos = 6;
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
        waitForStart();

        while (opModeIsActive()) {
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
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));
                if (pose != null) {

                    telemetry.addData("position", "vuforia is null");

                }
            } else {
                telemetry.addData("VuMark", "not visible");
            }


                telemetry.update();

                clark.setPosition(0.23);
                sleep(3000);
                telemetry.addData("leo", leo.blue());
                telemetry.addData("leo", leo.red());
                if (leo.blue() < leo.red()) {
                    telemetry.addData("leo", leo.blue());
                    telemetry.addData("leo", leo.red());
                    eddie.setPosition(0.3);

                    sleep(2000);
                } else if (leo.blue() > leo.red()) {
                    telemetry.addData("leo", leo.blue());
                    telemetry.addData("leo", leo.red());
                    eddie.setPosition(0.65);
                    sleep(1000);

                } else {
                    telemetry.addData("leo", leo.blue());
                    telemetry.addData("leo", leo.red());
                    telemetry.addData("eddie", "did not work");
                    sleep(250);
                }
                eddie.setPosition(0.55);
                sleep(1000);
                clark.setPosition(0.8);
                sleep(1000);

                Map.setRobot(10, 2);

                switch (vuMark) {
                    case LEFT:
                        Map.setGoal(6.5, 1);
                        moveState = MoveState.STRAFE_TOWARDS_GOAL;

                        if (distanceToGoal() <= .1) {
                            moveState = MoveState.STOP;
                        }
                    case RIGHT:
                        Map.setGoal(7.5, 1);
                        moveState = MoveState.STRAFE_TOWARDS_GOAL;

                        if (distanceToGoal() <= .1) {
                            moveState = MoveState.STOP;
                        }
                    case CENTER:
                        Map.setGoal(7, 1);
                        moveState = MoveState.STRAFE_TOWARDS_GOAL;

                        if (distanceToGoal() <= .1) {
                            moveState = MoveState.STOP;
                        }
                    case UNKNOWN:
                        Map.setGoal(7, 1);
                        moveState = MoveState.STRAFE_TOWARDS_GOAL;

                        if (distanceToGoal() <= .1) {
                            moveState = MoveState.STOP;
                        }


            }
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }


    private static class MoveState {
        private static final int STOP = 0;
        private static final int FORWARD = 1;
        private static final int BACKWARD = 2;
        private static final int LEFT = 3;
        private static final int RIGHT = 4;
        private static final int TURN_TOWARDS_GOAL = 5;
        private static final int STRAFE_TOWARDS_GOAL = 15;
        private static final int TURN_TOWARDS_ANGLE = 16;
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

        double DISTANCE_TOLERANCE = 1.0 / 10;
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
                double power = .75;

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
                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
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
        }

    }
    public void driveForward(double power, int distance){
        motorBackRight.setTargetPosition(distance);
        motorFrontRight.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(distance);
        motorFrontLeft.setTargetPosition(distance);

        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(power);

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

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
    }

    public void driveBackward(double power, int distance){
        motorBackRight.setTargetPosition(distance);
        motorFrontRight.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(distance);
        motorFrontLeft.setTargetPosition(distance);

        motorFrontLeft.setPower(-power);
        motorBackRight.setPower(-power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(-power);

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

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
    }

}


