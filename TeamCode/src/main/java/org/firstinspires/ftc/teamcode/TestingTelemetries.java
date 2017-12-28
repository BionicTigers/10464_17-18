package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;


@Autonomous(name="Testing Telemetries", group="Red")


public class TestingTelemetries extends OpMode {

    int i;
//    public Orientation angles;
//    private OpenGLMatrix lastLocation;
//    private DcMotor motorFrontRight;
//    private DcMotor motorBackLeft;
//    private DcMotor motorFrontLeft;
//    private DcMotor motorBackRight;
//    private DcMotor top;
//    private DcMotor front;
//    private Servo franny = null; //left servo
//    private Servo mobert = null; //right servo
//    private Servo servo;
//    private VuforiaLocalizer vuforia;
//    private VuforiaTrackable relicTemplate;
//    private RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
//    private int startDeg;
//    private int gameState;
//    private boolean jewelRot;
//    private ColorSensor sensorColor;
    private Servo eddie;
//    private boolean started;
//    private double waitTime;
//    private double heading;
//    BNO055IMU imu;
    private boolean blue;//true if blue d

    public void init() {
//        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
//        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
//        motorBackRight = hardwareMap.dcMotor.get("backRight");
//        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
//        top = hardwareMap.dcMotor.get("top");
//        front = hardwareMap.dcMotor.get("front");
//        franny = hardwareMap.servo.get("franny");
//        mobert = hardwareMap.servo.get("mobert");
//        servo = hardwareMap.servo.get("servo");
//        sensorColor = hardwareMap.get(ColorSensor.class, "sensorColor");
//        //startDeg = 0;
//        gameState = 0;
//        started = false;
//        waitTime = 0;
        eddie = hardwareMap.servo.get("eddie");
        blue = false;
//        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop() {
        eddie.setPosition(0.75);
        blue = true;
        eddie.setPosition(0.25);
        blue = false;
        telemetry.addData("Blue?", blue);

    }
}