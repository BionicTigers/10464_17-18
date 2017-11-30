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
    private long waitTime;

    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backRight");
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
            waitTime = System.currentTimeMillis() + 1000;
        }
        switch (gameState) {
            case 0: //Start
//                if (waitTime <= System.currentTimeMillis()) {
//                    //May be cause of nullPointer
//                    //sTime = getRuntime();
                //7map.setRobot(10,8);
                servo.setPosition(0.92);
                telemetry.addData("Servo Position", servo.getPosition());

                gameState = 1;

                if(gameState == 0) {

                    sensorColor.red();
                    sensorColor.blue();
                    telemetry.addData("State", gameState);
                    telemetry.addData("Color value blue", sensorColor.blue());
                    telemetry.addData("Color value red", sensorColor.red());
                }

            case 1:

                //telemetry.addData("Time left", waitTime - System.currentTimeMillis());

                //startDeg = motorBackRight.getCurrentPosition();
                //telemetry.addData("After startDeg", 3);
                if (sensorColor.red() > sensorColor.blue()) {

                    motorFrontLeft.setPower(-.6);
                    motorBackRight.setPower(-.6);

                }
                else if (sensorColor.red() < sensorColor.blue()) {

                    motorBackRight.setPower(.6);
                    motorFrontLeft.setPower(.6);

                }


                    gameState = 2;
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

