package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="Blue Corner", group ="Blue")

public class VenusAutoBlueCorner extends LinearOpMode {

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
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
        roger = hardwareMap.colorSensor.get( "roger"); //right color sensor
        leo = hardwareMap.colorSensor.get("leo"); //left color sensor
        blue = false;
        gameState = 0;
        waitTime = 0;

        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");

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

//
        //telemetr.addData(">", "Press Play to start");
        //telemetry.update();
        waitForStart();


        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        telemetry.addData("VuMark", vuMark);

        switch (vuMark) {
            case RIGHT:
                clark.setPosition(0.18);
                sleep(3000);


                if (leo.blue() < roger.blue()) {
                    eddie.setPosition(0.6);

                    blue = true;
                    sleep(1000);
                } else if (leo.blue() < roger.blue()) {
                    eddie.setPosition(0.4);
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

                driveForward(.5, 345);
                sleep(1000);

                driveBackward(.5, 25);
                sleep(2000);

                strafeLeft(.7, 220);
                sleep(1000);

                //PointTurnRight(.5,80);
                driveForward(.5, 75);
                sleep(2000);

                pointTurnLeft(0.5,440);

                //hamilton.setPosition(1);
                //burr.setPosition(1);
                sleep(500);

                driveBackward(.5, 30);
                sleep(1000);

                //hamilton.setPosition(.3);
                //burr.setPosition(0.3);
                sleep(1000);
                break;
            case LEFT:
                clark.setPosition(0.18);
                sleep(3000);


                if (leo.blue() < roger.blue()) {
                    eddie.setPosition(0.6);

                    blue = true;
                    sleep(1000);
                }
                else if (leo.blue() > roger.blue()) {
                    eddie.setPosition(0.4);
                    sleep(1000);

                    blue = false;
                }
                else {
                    telemetry.addData("eddie", "did not work");
                    sleep(250);
                }

                eddie.setPosition(0.55);
                sleep(1000);

                clark.setPosition(0.8);
                sleep(1000);

                //MOVE

                driveForward(.5, 345);
                sleep(1000);

                driveBackward(.5,25);
                sleep(2000);

                strafeLeft(.7, 220);
                sleep(1000);
//TURN SOMETIME

                //PointTurnRight(.5,80);
                driveForward(.5,75);
                sleep(2000);

                //hamilton.setPosition(1);
                //burr.setPosition(1);
                sleep(500);

                driveBackward(.5,30);
                sleep(1000);

                //hamilton.setPosition(.3);
                //burr.setPosition(0.3);
                sleep(1000);


                break;
            case CENTER:
                clark.setPosition(0.18);
                sleep(3000);


                if (leo.blue() < roger.blue()) {
                    eddie.setPosition(0.6);

                    blue = true;
                    sleep(1000);
                } else if (leo.blue() > roger.blue()) {
                    eddie.setPosition(0.4);
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

                driveForward(.5, 280);
                sleep(1000);

                strafeRight(.7, 200);
                sleep(1000);

                //PointTurnRight(.5,80);

                //hamilton.setPosition(1);
                //burr.setPosition(1);
                sleep(500);

                driveForward(.5, 80);
                sleep(1000);

                driveBackward(.5, 80);
                sleep(1000);

                //hamilton.setPosition(.3);
                //burr.setPosition(0.3);
                sleep(1000);

            default:
                clark.setPosition(0.18);
                sleep(3000);


                if (leo.blue() < roger.blue()) {
                    eddie.setPosition(0.6);

                    blue = true;
                    sleep(1000);
                } else if (leo.blue() > roger.blue()) {
                    eddie.setPosition(0.4);
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

                driveBackward(.5, 345);
                sleep(1000);

                driveForward(.5, 25);
                sleep(2000);

                strafeRight(.7, 220);
                sleep(1000);

                //PointTurnRight(.5,80);
                driveBackward(.5, 75);
                sleep(2000);

                //hamilton.setPosition(1);
                //burr.setPosition(1);
                sleep(500);

                driveForward(.5, 30);
                sleep(1000);

                //hamilton.setPosition(.3);
                //burr.setPosition(0.3);
                sleep(1000);
        }

                relicTrackables.activate();


        while (opModeIsActive()) {


        }

        idle();
        telemetry.update();
    }

    public void driveForward ( double power, int distance){

        motorFrontLeft.setTargetPosition(distance);
        motorBackRight.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(distance);
        motorFrontRight.setTargetPosition(distance);

        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);
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

    }


    public void pointTurnLeft(double power, int distance) {

        motorFrontLeft.setTargetPosition(-distance);
        motorBackRight.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(-distance);
        motorFrontRight.setTargetPosition(distance);

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

    }

    public void driveBackward(double power, int distance) {

        motorFrontLeft.setTargetPosition(-distance);
        motorBackRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(-distance);
        motorFrontRight.setTargetPosition(-distance);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(-power);
        motorBackRight.setPower(-power);
        motorBackLeft.setPower(-power);
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

    }


    public void strafeLeft(double power, int distance) {

        motorFrontLeft.setTargetPosition(distance);
        motorBackRight.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(-distance);
        motorFrontRight.setTargetPosition(-distance);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorBackLeft.setPower(-power);
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

    }


    public void strafeRight(double power, int distance) {

        motorFrontLeft.setTargetPosition(-distance);
        motorBackRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(distance);
        motorFrontRight.setTargetPosition(distance);

        motorFrontLeft.setPower(-power);
        motorBackRight.setPower(-power);
        motorBackLeft.setPower(power);
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

    }


}



