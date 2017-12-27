package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="SwingServo", group="Red")
public class SwingServo extends AutonomousBaseMercury{

    int i;
    public Orientation angles;
    //private OpenGLMatrix lastLocation;

    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;

    private Servo eddie; //drop down servo (for color sensor)
    private Servo clark; //swing servo (for color sensor)
    private ColorSensor roger; //right color sensor
    private ColorSensor leo; //left color sensor

    BNO055IMU imu;
    private boolean blue;//true if blue detected

    private double waitTime;
    private int gameState;


    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");

        eddie = hardwareMap.servo.get("eddie"); //drop down servo
        clark = hardwareMap.servo.get("clark"); //swing servo
        roger = hardwareMap.colorSensor.get( "roger"); //right color sensor
        leo = hardwareMap.colorSensor.get("leo"); //left color sensor

        startDeg = 0;
        gameState = 0;
        waitTime = getRuntime();

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void loop() {
        //super.gameState();

        switch(gameState) {
            case 0: //preset variables
                waitTime = getRuntime(); //get current runTime
                clark.setPosition(.9);
                gameState = 1;
                break;

            case 1://delay to allow servo to drop
                if(getRuntime() > waitTime + .75) {
                    gameState = 2;
                }
                break;
            case 2: //detect color sensor and choose direction
                waitTime = getRuntime(); //get current runTime
                if (roger.blue() < leo.blue()) {
                    eddie.setPosition(0.25);
                    gameState = 3;
                }
                else{
                    eddie.setPosition(0.75);
                    gameState = 3;
                }
                break;
            case 3://delay to allow turn
                if(getRuntime() > waitTime + 2.0) {
                    gameState = 4;
                }
                break;
            case 4: //stop all motors, pull servo up
                clark.setPosition(0.4);
                break;

        }
        telemetry.addData("State", gameState);
        telemetry.addData("Color value blue", roger.blue());
        telemetry.addData("Color value blue2", leo.blue());
        telemetry.addData("Current runtime", getRuntime());

    }


}


