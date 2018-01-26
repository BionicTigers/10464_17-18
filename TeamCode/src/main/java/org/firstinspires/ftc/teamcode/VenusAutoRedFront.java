package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.sun.tools.javac.util.Constants.format;


@Autonomous(name="Red Front", group ="Red")

public class VenusAutoRedFront extends LinearOpMode {

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor wilbert; //Four Bar Right
    public DcMotor evangelino; //Four Bar Left
    public Servo hamilton; //Glyph Flipper
    //public Servo burr; //Glyph Flipper 2
    public ElapsedTime runtime = new ElapsedTime();
    int i;
    boolean blue;
    private Servo clark; //drop down servo (for color sensor)
    private Servo eddie; //swing servo (for color sensor)
    private ColorSensor roger; //right color sensor
    private ColorSensor leo; //left color sensor
    private double waitTime;
    private int gameState;

    @Override
    public void runOpMode() {

        eddie = hardwareMap.servo.get("eddie"); //swing servo
        clark = hardwareMap.servo.get("clark"); //drop down servo
        roger = hardwareMap.colorSensor.get("roger"); //right color sensor
        leo = hardwareMap.colorSensor.get("leo"); //left color sensor
        blue = false;
        gameState = 0;
        waitTime = 0;

        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        hamilton = hardwareMap.servo.get("hamilton");

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AfBkGLH/////AAAAGUUS7r9Ue00upoglw/0yqTBLwhqYHpjwUK9zxmWMMFGuNGPjo/RjNOTsS8POmdQLHwe3/75saYsyb+mxz6p4O8xFwDT7FEYMmKW2NKaLKCA2078PZgJjnyw+34GV8IBUvi2SOre0m/g0X5eajpAhJ8ZFYNIMbUfavjQX3O7P0UHyXsC3MKxfjMzIqG1AgfRevcR/ONOJlONZw7YIZU3STjODyuPWupm2p7DtSY4TRX5opqFjGQVKWa2IlNoszsN0szgW/xJ1Oz5VZp4oDRS8efG0jOq1QlGw7IJOs4XXZMcsk0RW/70fVeBiT+LMzM8Ih/BUxtVVK4pcLMpb2wlzdKVLkSD8LOpaFWmgOhxtNz2M";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessa
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        waitForStart();
        relicTrackables.activate();


            telemetry.update();

//
            switch (vuMark) {
                case RIGHT: //Go to Right column
                    telemetry.addData("vumark", "right");

                    clark.setPosition(0.25);
                    sleep(3000);


                    if (leo.blue() < roger.blue()) {
                        eddie.setPosition(0.3); //flick right

                        blue = true;
                        sleep(2000);
                    } else if (leo.blue() > roger.blue()) {
                        eddie.setPosition(0.65);
                        sleep(1000);

                        blue = false;
                    } else {
                        telemetry.addData("eddie", "did not work");
                        sleep(250);
                    }

                    eddie.setPosition(0.55);
                    sleep(1000);

                    clark.setPosition(0.8);
                    sleep(1000);

                    //MOVE

                    driveBackward(.5, 500);
                    sleep(1000);

                    pointTurnRight(.5, 260);
                    sleep(1000);

                    strafeRight(.5, 90);

                    driveBackward(.5, 100);
                    sleep(1000);

                    hamilton.setPosition(1);
                    sleep(1000);

                    driveBackward(.5, 100);
                    sleep(500);

                    driveBackward(.5, -120);
                    sleep(500);

                    hamilton.setPosition(.3);
                    sleep(500);

                    stop();

                    break;

                case LEFT: //Go to Left column

                    telemetry.addData("vumark", "you dumbass, its not reading");
                    clark.setPosition(0.25);
                    sleep(3000);


                    if (leo.blue() < roger.blue()) {
                        eddie.setPosition(0.3);

                        blue = true;
                        sleep(2000);
                    } else if (leo.blue() > roger.blue()) {
                        eddie.setPosition(0.65);
                        sleep(1000);

                        blue = false;
                    } else {
                        telemetry.addData("eddie", "did not work");
                        sleep(250);
                    }

                    eddie.setPosition(0.55);
                    sleep(1000);

                    clark.setPosition(0.8);
                    sleep(1000);

                    //MOVE

                    driveBackward(.5, 580);
                    sleep(1000);

                    pointTurnRight(.5, 260);
                    sleep(1000);

                    driveBackward(.5, 100);
                    sleep(1000);

                    hamilton.setPosition(1);
                    sleep(1000);

                    driveBackward(.5, 100);
                    sleep(500);

                    driveBackward(.5, -120);
                    sleep(500);

                    hamilton.setPosition(.3);
                    sleep(500);

                    stop();

                    break;

                case CENTER: //Go to Center column
                    telemetry.addData("vumark", "you dumbass, its not reading");
                    clark.setPosition(0.25);
                    sleep(3000);


                    if (leo.blue() < roger.blue()) {
                        eddie.setPosition(0.3);

                        blue = true;
                        sleep(2000);
                    } else if (leo.blue() > roger.blue()) {
                        eddie.setPosition(0.65);
                        sleep(1000);

                        blue = false;
                    } else {
                        telemetry.addData("eddie", "did not work");
                        sleep(250);
                    }

                    eddie.setPosition(0.55);
                    sleep(1000);

                    clark.setPosition(0.8);
                    sleep(1000);

                    //MOVE

                    driveBackward(.5, 540);
                    sleep(1000);

                    pointTurnRight(.5, 260);
                    sleep(1000);

                    driveBackward(.5, 100);
                    sleep(1000);

                    hamilton.setPosition(1);
                    sleep(1000);

                    driveBackward(.5, 100);
                    sleep(500);

                    driveBackward(.5, -120);
                    sleep(500);

                    hamilton.setPosition(.3);
                    sleep(500);

                    stop();

                    break;

                default: //ACTUAL START OF PROGRAM
                    telemetry.addData("vumark", "you dumbass, its not reading");
                    clark.setPosition(0.22);
                    sleep(3000);


                    if (leo.blue() < roger.blue()) {
                        eddie.setPosition(0.3);

                        blue = true;
                        sleep(2000);
                    } else if (leo.blue() > roger.blue()) {
                        eddie.setPosition(0.65);
                        sleep(1000);

                        blue = false;
                    } else {
                        telemetry.addData("eddie", "did not work");
                        sleep(250);
                    }

                    eddie.setPosition(0.55);
                    sleep(1000);

                    clark.setPosition(0.8);
                    sleep(1000);

                    //MOVE

                    driveBackward(.5, 540);
                    sleep(1000);

                    pointTurnRight(.5, 268);
                    sleep(1000);

                    driveBackward(.5, 80);
                    sleep(1000);

                    hamilton.setPosition(1);
                    sleep(1000);

                    driveBackward(.5, -75);

                    driveBackward(.5, 100);
                    sleep(500);

                    driveBackward(.5, -110);
                    sleep(500);

                    hamilton.setPosition(.3);
                    sleep(500);

                    stop();

                    break;

            }

            relicTrackables.activate();

            while (opModeIsActive()) {
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    telemetry.addData("VuMark",
                            "%s visible", vuMark);
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                    telemetry.addData("Pose", format(pose));
                    if (pose != null) {
                        VectorF trans = pose.getTranslation();
                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                        // Extract the X, Y, and Z components of the offset of the target relative to the robot
                        double tX = trans.get(0);
                        double tY = trans.get(1);
                        double tZ = trans.get(2);

                        // Extract the rotational components of the target relative to the robot
                        double rX = rot.firstAngle;
                        double rY = rot.secondAngle;
                        double rZ = rot.thirdAngle;
                    }
                } else {
                    telemetry.addData("VuMark", "not visible");
                }



            }
            idle();
            telemetry.update();
        }


    public void driveForward ( double power, int distance){

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

    public void driveBackward(double power, int distance) {

        motorBackRight.setTargetPosition(-distance);
        motorFrontRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(-distance);
        motorFrontLeft.setTargetPosition(-distance);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(-power);
        motorBackRight.setPower(-power*1.45);
        motorBackLeft.setPower(-power*0.65);
        motorFrontRight.setPower(-power);

        while (motorFrontLeft.isBusy() && motorBackRight.isBusy() && motorBackLeft.isBusy() && motorFrontRight.isBusy()) {

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

    public void pointTurnRight(double power, int distance) {

        motorBackRight.setTargetPosition(-distance);
        motorFrontRight.setTargetPosition(-distance);
        motorFrontLeft.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(distance);

        motorFrontLeft.setPower(power);
        motorBackRight.setPower(-power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(-power);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motorFrontLeft.isBusy() && motorBackRight.isBusy() && motorBackLeft.isBusy() && motorFrontRight.isBusy()) {

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

    public void pointTurnLeft(double power, int distance) {

        motorBackRight.setTargetPosition(distance);
        motorFrontRight.setTargetPosition(distance);
        motorFrontLeft.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(distance);

        motorFrontLeft.setPower(-power);
        motorBackRight.setPower(power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(power);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motorFrontLeft.isBusy() && motorBackRight.isBusy() && motorBackLeft.isBusy() && motorFrontRight.isBusy()) {

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


    public void strafeLeft(double power, int distance) {

        motorBackRight.setTargetPosition(distance);
        motorFrontRight.setTargetPosition(distance);
        motorFrontLeft.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(distance);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(-power);
        motorBackRight.setPower(power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(power);

        while (motorFrontLeft.isBusy() && motorBackRight.isBusy() && motorBackLeft.isBusy() && motorFrontRight.isBusy()) {

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


    public void strafeRight(double power, int distance) {
        motorBackRight.setTargetPosition(distance);
        motorFrontRight.setTargetPosition(distance);
        motorFrontLeft.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(-distance);

        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(-power);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motorFrontLeft.isBusy() && motorBackRight.isBusy() && motorBackLeft.isBusy() && motorFrontRight.isBusy()) {

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


