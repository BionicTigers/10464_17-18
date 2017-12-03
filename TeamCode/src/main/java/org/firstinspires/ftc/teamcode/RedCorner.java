package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

@Autonomous(name="Red Corner", group="Red")
public class RedCorner extends AutonomousBase {

    double xTime;
    int i;
    private OpenGLMatrix lastLocation;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor top;
    private DcMotor front;
    private Servo servo;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackable relicTemplate;
//private int startDeg;
    private int gameState;
    private ColorSensor sensorColor;
    private boolean started;
    private double waitTime;

    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        top = hardwareMap.dcMotor.get("top");
        front = hardwareMap.dcMotor.get("front");
        servo = hardwareMap.servo.get("servo");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensorColor");
        //startDeg = 0;
        gameState = 0;
        started = false;
        waitTime = 0;
        map.setRobot(10, 8);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void loop() {
        //super.gameState();
        if (!started) {
            started = true;
            waitTime = getRuntime() + 3;
        }
        switch (gameState) {
            case 0:
                servo.setPosition(0.92);
                telemetry.addData("Time", getRuntime());

                sTime = getRuntime();
                map.setRobot(10, 8);

                if (waitTime + 1 <= sTime) {
                    waitTime = getRuntime() + .5;
                    gameState = 1;
                }

                break;

            case 1:
//telemetry.addData("Time left", waitTime - System.currentTimeMillis());
//startDeg = motorBackRight.getCurrentPosition();
//telemetry.addData("After startDeg", 3);
                sTime = getRuntime();
                if (sensorColor.red() > sensorColor.blue()) {
                    motorFrontLeft.setPower(-.6);
                    motorBackRight.setPower(.6);
                }
                else {
                    motorBackRight.setPower(-.6);
                    motorFrontLeft.setPower(.6);
                }
                if (waitTime + 1 <= sTime) {
                    gameState = 2;
                }
                    break;

// commented vuforia goes here

            case 2:
                map.setGoal(9, 11);
                moveState = MoveState.STRAFE_TOWARDS_GOAL;

                break;
        }
        telemetry.addData("Motor degrees", motorBackRight.getCurrentPosition());
//telemetry.addData("Start degrees", startDeg);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}

