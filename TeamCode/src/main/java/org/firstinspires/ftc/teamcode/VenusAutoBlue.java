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


@Autonomous(name="Auto Test", group ="Vuforia")

public class VenusAutoBlue extends LinearOpMode {

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor wilbert;
    public DcMotor evangelino;
    public Servo hamilton;
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

        telemetry.addData("FL Enc", motorFrontLeft.getCurrentPosition());
        telemetry.addData("FR Enc", motorFrontRight.getCurrentPosition());
        telemetry.addData("BL Enc", motorBackLeft.getCurrentPosition());
        telemetry.addData("BR Enc", motorBackRight.getCurrentPosition());
        telemetry.update();
        waitForStart();

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

        switch(gameState) {
            case 0: //preset variables
                clark.setPosition(0.18);
                gameState = 1;
                waitTime = getRuntime(); //get current runTime
                break;

            case 1://delay to allow servo to drop
                if (getRuntime() > waitTime + 3.0) {
                    gameState = 2;
                }
                break;

            case 2: //detect color sensor and choose direction
                if (leo.blue() < roger.blue()) {
                    eddie.setPosition(0.65);
                    //eddie.setPosition(0.5);
                    gameState = 3;
                    blue = true;
                }
                else if (leo.blue() > roger.blue()) {
                    eddie.setPosition(0.45);
                    //eddie.setPosition(0.5);
                    gameState = 3;
                    blue = false;
                } else {
                    gameState = 3;
                }
                waitTime = getRuntime(); //get current runTime
                break;

            case 3://delay to allow turn
                if(getRuntime() > waitTime + 2.0) {
                    gameState = 4;
                }
                break;

            case 4: //stop all motors, pull servo up
                eddie.setPosition(0.55);
                waitTime = getRuntime();
                gameState = 5;
                break;

            case 5://delay to allow turn
                if(getRuntime() > waitTime + 2.0) {
                    gameState = 6;
                }
                break;

            case 6:
                clark.setPosition(0.8);
                break;
        }



        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        telemetry.addData("VuMark", vuMark);

        switch (vuMark) {
            case RIGHT:
                DriveForward(.5,100);
//                DriveForward(.5,450);
//                DriveForward(.5,450);

                PointTurnLeft(.5,1440);

                DropGlyph(.5,16);

                DriveForward(.5,250);

                sleep(250);

                DriveBackward(.5,500);
                PointTurnLeft(.5,2880);

                break;
            case LEFT:
                DriveForward(.5,100);
//                DriveForward(.5,450);
//                DriveForward(.5,450);

                PointTurnLeft(.5,1440);

                DropGlyph(.5,16);

                DriveForward(.5,250);

                sleep(250);

                DriveBackward(.5,500);
                PointTurnLeft(.5,2880);

                break;
            case CENTER:
                DriveForward(.5,100);
//                DriveForward(.5,450);
//                DriveForward(.5,450);

                PointTurnLeft(.5,1440);

                DropGlyph(.5,16);

                DriveForward(.5,250);

                sleep(250);

                DriveBackward(.5,500);
                PointTurnLeft(.5,2880) ;

                break;
            default:
                DriveForward(.5,100);
//                DriveForward(.5,450);
//                DriveForward(.5,450);

                PointTurnLeft(.5,1440);

                DropGlyph(.5,16);

                DriveForward(.5,250);

                sleep(250);

                DriveBackward(.5,500);
                PointTurnLeft(.5,2880);

                break;

        }


        relicTrackables.activate();

        while (opModeIsActive()) {


        }

        idle();
        telemetry.update();
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void DriveForward ( double power, int distance){

        motorFrontLeft.setTargetPosition(distance);
        motorBackRight.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(distance);
        motorFrontRight.setTargetPosition(distance);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);
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

    public void PointTurnRight(double power, int distance) {

        motorFrontLeft.setTargetPosition(distance);
        motorBackRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(distance);
        motorFrontRight.setTargetPosition(-distance);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(power);
        motorBackRight.setPower(-power);
        motorBackLeft.setPower(power);
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

    public void PointTurnLeft(double power, int distance) {

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

    }

    public void DriveBackward(double power, int distance) {

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
    public void DropGlyph ( double power, int distance){
        wilbert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        evangelino.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wilbert.setTargetPosition(distance);
        evangelino.setTargetPosition(distance);

        wilbert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        evangelino.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wilbert.setPower(power);
        evangelino.setPower(power);

        while (evangelino.isBusy() && wilbert.isBusy()) {
        }
        wilbert.setPower(0);
        evangelino.setPower(0);

        wilbert.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        evangelino.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hamilton.setPosition((1));

        sleep(500);

    }

}



