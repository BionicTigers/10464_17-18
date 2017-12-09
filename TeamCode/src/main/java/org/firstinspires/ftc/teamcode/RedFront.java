package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

@Autonomous(name = "Red Front", group = "Red")
public class RedFront extends AutonomousBase {

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
        blue = false;


    }

    public void loop() {
        //super.gameState();


        telemetry.addData("Motor degrees", motorBackRight.getCurrentPosition());
        telemetry.addData("State", gameState);
        telemetry.addData("Color value blue", sensorColor.blue());
        telemetry.addData("Current runtime", getRuntime());
        telemetry.addData("blue", blue);


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
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                motorFrontRight.setPower(0);
                servo.setPosition(0.52);
                gameState = 5;
                break;

            case 5:

                map.setGoal(11, 5);
                //moveState =
                motorFrontRight.setPower(0.25);
                motorFrontLeft.setPower(-0.25);
                motorBackRight.setPower(0.25);
                motorBackLeft.setPower(-0.25);
                gameState = 6;
                break;

            case 6:
                map.setGoal(11, 5);
                moveState = MoveState.FORWARD;
                if (map.distanceToGoal() <= .1) {
                    moveState = MoveState.STOP;
                    gameState = 7;
                }

                break;

            case 7:
                while (waitTime < 25) {
                    top.setPower(.5);
                    front.setPower(.5);

                    break;
                }
                break;


//                telemetry.addData("jewelRot", jewelRot);
//
//                //return to original rotation to read pictograph
//                if(jewelRot = true){
//
//                    motorBackLeft.setPower(-.5);
//                    motorFrontRight.setPower(-.5);
//                }
//                else{
//
//                    motorBackLeft.setPower(.5);
//                    motorFrontRight.setPower(.5);
//                }
//                if(this.getRuntime() > 2.0) {
//                    gameState = 6;
//                }
//                break;
//
//            case 6:
//
//                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//                parameters.vuforiaLicenseKey = "AfBkGLH/////AAAAGUUS7r9Ue00upoglw/0yqTBLwhqYHpjwUK9zxmWMMFGuNGPjo/RjNOTsS8POmdQLHwe3/75saYsyb+mxz6p4O8xFwDT7FEYMmKW2NKaLKCA2078PZgJjnyw+34GV8IBUvi2SOre0m/g0X5eajpAhJ8ZFYNIMbUfavjQX3O7P0UHyXsC3MKxfjMzIqG1AgfRevcR/ONOJlONZw7YIZU3STjODyuPWupm2p7DtSY4TRX5opqFjGQVKWa2IlNoszsN0szgW/xJ1Oz5VZp4oDRS8efG0jOq1QlGw7IJOs4XXZMcsk0RW/70fVeBiT+LMzM8Ih/BUxtVVK4pcLMpb2wlzdKVLkSD8LOpaFWmgOhxtNz2M";
//
//                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
//                this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//
//                VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//                relicTemplate = relicTrackables.get(0);
//                relicTemplate.setName("relicVuMarkTemplate");
//
//                relicTrackables.activate();
//
//                if(getRuntime() > waitTime + 2) {
//                    gameState = 7;
//                }
//
//                break;
//
//
//            case 7:
//
//                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
//
//                    telemetry.addData("VuMark", "%s visible", vuMark);
//                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
//                    telemetry.addData("Pose", format(pose));
//
//                    if (pose != null) {
//                        VectorF trans = pose.getTranslation();
//                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//
//                        // Extract the X, Y, and Z components of the offset of the target relative to the robot
//                        double tX = trans.get(0);
//                        double tY = trans.get(1);
//                        double tZ = trans.get(2);
//
//                        // Extract the rotational components of the target relative to the robot
//                        double rX = rot.firstAngle;
//                        double rY = rot.secondAngle;
//                        double rZ = rot.thirdAngle;
//
//                        telemetry.addData("tX",rX);
//                        telemetry.addData("rX",tX);
//                    }
//                }
//                else {
//                    telemetry.addData("VuMark", "not visible");
//                }
//                if (getRuntime() > waitTime + 2){
//                    gameState = 8;
//                    startDeg = motorBackRight.getCurrentPosition();
//                }
//                break;
//
//            case 8: // moving robot to correct position in safe zone
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                heading = angles.firstAngle;
//
//                if(vuMark == RelicRecoveryVuMark.RIGHT){
//                    map.setGoal(11,5.4);
//                    moveState = MoveState.STRAFE_TOWARDS_GOAL;
//                    if(map.distanceToGoal()< + .1) {
//                        moveState = MoveState.STOP;
//                        gameState = 9;
//                    }
//                }
//                if(vuMark == RelicRecoveryVuMark.CENTER){
//                    map.setGoal(11,5);
//                    moveState = MoveState.STRAFE_TOWARDS_GOAL;
//                    if(map.distanceToGoal()< + .1) {
//                        moveState = MoveState.STOP;
//                        gameState = 9;
//                    }
//                }
//                if(vuMark == RelicRecoveryVuMark.LEFT){
//                    map.setGoal(11,4.6);
//                    moveState = MoveState.STRAFE_TOWARDS_GOAL;
//                    if(map.distanceToGoal()< + .1) {
//                        moveState = MoveState.STOP;
//                        gameState = 9;
//                    }
//                }
//
//                break;
//
//
//            case 9:
//
//                front.setPower(.4);
//                top.setPower(.4);
//
//                if(this.getRuntime() > 3.0) {
//                    gameState = 3;
//                }
//                else {
//                    telemetry.addData("Its not working,", "go again 4");
//                }
//
//                break;
//
//        }
//        telemetry.addData("Motor degrees", motorBackRight.getCurrentPosition());
//        telemetry.addData("State", gameState);
//        telemetry.addData("Color value blue", sensorColor.blue());
//        telemetry.addData("Current runtime", getRuntime());
//        telemetry.addData("Target runtime", sTime);
//    }
//
////    String format(OpenGLMatrix transformationMatrix) {
////        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
//
        }
    }
}