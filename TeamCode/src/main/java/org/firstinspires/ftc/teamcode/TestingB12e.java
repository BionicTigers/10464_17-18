package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;


@Autonomous(name="testingB12e", group="Red")


public class TestingB12e extends AutonomousBase {
    double xTime;
    int i;
    private OpenGLMatrix lastLocation;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor top;
    private DcMotor front;
    private Servo franny = null; //left servo
    private Servo mobert = null; //right servo
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
        franny = hardwareMap.servo.get("franny");
        mobert = hardwareMap.servo.get("mobert");
        servo = hardwareMap.servo.get("servo");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensorColor");
        //startDeg = 0;
        gameState = 0;
        started = false;
        waitTime = 0;
        map.setRobot(10, 2);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void loop() {
        //super.gameState();
        if (!started) {
            started = true;
        }
        switch (gameState) {
            case 0:
                franny.setPosition(.35);
                mobert.setPosition(.32);
                map.setGoal(10, 4);
                //moveState =
                motorFrontRight.setPower(0.5);
                motorFrontLeft.setPower(0.5);
                motorBackRight.setPower(-0.5);
                motorBackLeft.setPower(-0.5);
                if (map.distanceToGoal() <= .1) {
                    moveState = MoveState.STOP;
                }
                gameState = 1;
                break;

            case 1:
                map.setGoal(10, 4);
                moveState = MoveState.FORWARD;
                if (map.distanceToGoal() <= .1) {
                    moveState = MoveState.STOP;
                }

                break;

        }
    }
}
