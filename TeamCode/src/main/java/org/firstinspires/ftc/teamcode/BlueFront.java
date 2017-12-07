package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

@Autonomous(name="Blue Front", group="Blue")
public class BlueFront extends AutonomousBase {

    int i;
    public Orientation angles;
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
    private RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
    private int startDeg;
    private int gameState;
    private boolean jewelRot;
    private ColorSensor sensorColor;
    private boolean started;
    private double waitTime;
    BNO055IMU imu;
    ElapsedTime time = new ElapsedTime();
    //use time.reset() and time.seconds()



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
        startDeg = 0;
        gameState = 0;
        started = false;
        waitTime = 0;
        map.setRobot(10, 2);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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

                gameState = 1;

                break;

            case 1: //Innitialize positions in map class, of the flipper servos, and color sensor servo. Waits 2 second before moving to case 1.
                time.reset();
                franny.setPosition(.35);
                mobert.setPosition(.32);
                servo.setPosition(0.92);

                map.setRobot(2, 2);

                if (time.seconds() > 6) {
                    gameState = 2;
                    telemetry.addData("time", time.seconds());
                } else {
                    telemetry.addData("this.getRuntime", "not greater than 6 (1)");
                }

                break;

            case 2://Pulls values from color sensor and compares them. Rotates robot about center to knock off jewel.
                telemetry.addData("red", sensorColor.red());
                telemetry.addData("blue", sensorColor.blue());

                time.reset();

                if (sensorColor.red() > sensorColor.blue()) {
                    jewelRot = true;
                    motorBackLeft.setPower(.7);
                    motorFrontRight.setPower(.65);
                } else {
                    jewelRot = false;
                    motorBackLeft.setPower(-.65);
                    motorFrontRight.setPower(-.7);
                }


                if (time.seconds() > 2) {
                    motorFrontRight.setPower(0);
                    motorBackLeft.setPower(0);
                } else {
                    telemetry.addData("this.getRuntime", "not greater than 2 (2)");
                }


                break;

            case 3:
                servo.setPosition(.52);
                telemetry.addData("jewelRot", jewelRot);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);

                break;

                //return to original rotation to read pictograph
//                if (jewelRot = false) {
//
//                    motorBackLeft.setPower(-.7);
//                    motorFrontRight.setPower(-.65);
//                } else {
//
//                    motorBackLeft.setPower(.7);
//                    motorFrontRight.setPower(.65);
//                }
//                if (this.getRuntime() > 3.0) {
//                    gameState = 4;
//                } else {
//                    telemetry.addData("Its not working,", "damn it 3");
//                }
//                break;

//            }
        }
    }
}