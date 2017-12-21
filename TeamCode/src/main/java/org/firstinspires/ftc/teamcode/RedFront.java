package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

@Autonomous(name = "Red Front", group = "Red")
public class RedFront extends AutonomousBaseMercury {

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
    public boolean started;
    private ColorSensor sensorColor;
    private double waitTime;
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    VuforiaLocalizer.Parameters vulocal = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    BNO055IMU imu;
    private boolean blue;//true if blue detected


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
        waitTime = 0;
        startDeg = 0;
        gameState = 0;
        started = false;
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

        vulocal.vuforiaLicenseKey = "AfBkGLH/////AAAAGUUS7r9Ue00upoglw/0yqTBLwhqYHpjwUK9zxmWMMFGuNGPjo/RjNOTsS8POmdQLHwe3/75saYsyb+mxz6p4O8xFwDT7FEYMmKW2NKaLKCA2078PZgJjnyw+34GV8IBUvi2SOre0m/g0X5eajpAhJ8ZFYNIMbUfavjQX3O7P0UHyXsC3MKxfjMzIqG1AgfRevcR/ONOJlONZw7YIZU3STjODyuPWupm2p7DtSY4TRX5opqFjGQVKWa2IlNoszsN0szgW/xJ1Oz5VZp4oDRS8efG0jOq1QlGw7IJOs4XXZMcsk0RW/70fVeBiT+LMzM8Ih/BUxtVVK4pcLMpb2wlzdKVLkSD8LOpaFWmgOhxtNz2M";
        vulocal.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        //VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(vulocal);

        blue = false;
    }


    public void loop() {
        //super.gameState();

        telemetry.addData("Motor degrees", motorBackRight.getCurrentPosition());
        telemetry.addData("State", gameState);
        telemetry.addData("Color value blue", sensorColor.blue());
        telemetry.addData("Current runtime", getRuntime());
        telemetry.addData("Blue", blue);

        switch (gameState) {
            case 0: //preset variables
                waitTime = getRuntime(); //get current runTime
                gameState = 1;
                servo.setPosition(.9);
                franny.setPosition(.35);
                mobert.setPosition(.32);

                break;

            case 1://delay to allow servo to drop
                if (getRuntime() > waitTime + 2.0) {
                    gameState = 2;
                }
                break;
            case 2: //detect color sensor and choose direction
                waitTime = getRuntime(); //get current runTime

                if (sensorColor.blue() < 1) { //red
                    motorFrontLeft.setPower(-.5);
                    motorBackRight.setPower(-.55);
                    gameState = 3;
                    blue = false;

                } else {
                    motorFrontLeft.setPower(.55);
                    motorBackRight.setPower(.5);
                    gameState = 3;
                    blue = true;
                }

                break;

            case 3://delay to allow turn
                if (getRuntime() > waitTime + 2.0) {
                    gameState = 4;
                }
                break;
            case 4: //stop all motors, pull servo up
                moveState = MoveState.STOP;
                servo.setPosition(0.52);

                if(vuMark != RelicRecoveryVuMark.UNKNOWN){
                    telemetry.addData("VuMark", vuMark);
                    gameState = 6;
                }
                else{
                    telemetry.addData("Vumark is not seen", 0);
                }



                break;


//            case 5:
//
//                map.setGoal(11, 5);
//                //moveState =
//                motorFrontRight.setPower(0.25);
//                motorFrontLeft.setPower(-0.25);
//                motorBackRight.setPower(0.25);
//                motorBackLeft.setPower(-0.25);
//                gameState = 6;
//                break;


            case 6:
                int choosen = Vuforia(cameraMonitorViewId, "Red", vuforia);

                switch (choosen) {
                    case (1):
                        map.setGoal(11, 5.4);
                        moveState = MoveState.LEFT_SLOW;
                        break;
                    case (2):
                        map.setGoal(11,5);
                        moveState = MoveState.LEFT_SLOW;
                        break;
                    case (3):
                        map.setGoal(11,4.6);
                        moveState = MoveState.LEFT_SLOW;
                        break;
                    default:
                        map.setGoal(11,5);
                        moveState = MoveState.LEFT_SLOW;
                        break;
                }
                gameState = 7;
                break;

            case 7:
                while (waitTime < 25) {
                    top.setPower(.5);
                    front.setPower(.5);

                    break;
                }
                break;
        }
    }
}





