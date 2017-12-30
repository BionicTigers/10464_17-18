package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

@Autonomous(name = "Blue Front")
@Disabled
public class BlueFront extends AutonomousBaseMercury {

    int i;
    public Orientation angles;
    private OpenGLMatrix lastLocation;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor top;
    private DcMotor front;
    Servo franny; //left servo
    Servo mobert; //right servo
    //private Servo tiffany; //up/down
    //private Servo mrClean; // right/left
    private VuforiaLocalizer vuforia;
    private VuforiaTrackable relicTemplate;
    private RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
    private int startDeg;
    private int gameState;
    private boolean jewelRot;
    private ColorSensor sensorColor;
    private boolean started;
    private double waitTime;
    private double heading;
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
        //servo = hardwareMap.servo.get("servo");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensorColor");
        startDeg = 0;
        gameState = 0;
        started = false;
        heading = angles.firstAngle;
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

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;

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
//                tiffany.setPosition(.9);
                map.setRobot(2,2);
                break;

            case 1://delay to allow servo to drop
                if (getRuntime() > waitTime + 2.0) {
                    gameState = 2;
                }
                break;

            case 2: //detect color sensor and choose direction
                waitTime = getRuntime(); //get current runTime
                while (waitTime < 7) {
                    if (sensorColor.blue() > 3) { //red
                        motorFrontLeft.setPower(.5);
                        motorBackRight.setPower(.55);
                        gameState = 3;
                        blue = true;
                        break;
                    } else {
                        motorFrontLeft.setPower(-.55);
                        motorBackRight.setPower(-.5);
                        gameState = 3;
                        blue = false;
                        break;
                    }
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
                //servo.setPosition(0.52);
                break;

            case 5:
                map.setGoal(1, 5);
                //moveState =
                motorFrontRight.setPower(0.25);
                motorFrontLeft.setPower(-0.25);
                motorBackRight.setPower(0.25);
                motorBackLeft.setPower(-0.25);
                gameState = 6;
                break;

            case 6:
                map.setGoal(1, 5);
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
                telemetry.addData("jewelRot", jewelRot);


                //return to original rotation to read pictograph
                    if(jewelRot = true){

                        motorBackLeft.setPower(-.5);
                        motorFrontRight.setPower(-.5);
                    }
                    else{

                        motorBackLeft.setPower(.5);
                        motorFrontRight.setPower(.5);
                    }
                    if(this.getRuntime() > 2.0) {
                        gameState = 6;
                    }
                    break;


                case 10: // moving robot to correct position in safe zone
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    heading = angles.firstAngle;

                    if(vuMark == RelicRecoveryVuMark.RIGHT){
                        map.setGoal(1,5.4);
                        moveState = MoveState.STRAFE_TOWARDS_GOAL;
                        if(map.distanceToGoal()< + .1) {
                            moveState = MoveState.STOP;
                            gameState = 9;
                        }
                    }
                    if(vuMark == RelicRecoveryVuMark.CENTER){
                        map.setGoal(1,5);
                        moveState = MoveState.STRAFE_TOWARDS_GOAL;
                        if(map.distanceToGoal()< + .1) {
                            moveState = MoveState.STOP;
                            gameState = 9;
                        }
                    }
                    if(vuMark == RelicRecoveryVuMark.LEFT){
                        map.setGoal(1,4.6);
                        moveState = MoveState.STRAFE_TOWARDS_GOAL;
                        if(map.distanceToGoal()< + .1) {
                            moveState = MoveState.STOP;
                            gameState = 9;
                        }
                    }

                    break;


                case 11:

                    front.setPower(.4);
                    top.setPower(.4);

                    if(this.getRuntime() > 3.0) {
                        gameState = 3;
                    }
                    else {
                        telemetry.addData("Its not working,", "go again 4");
                    }

                    break;

        }
        telemetry.addData("Motor degrees", motorBackRight.getCurrentPosition());
        telemetry.addData("State", gameState);
        telemetry.addData("Color value blue", sensorColor.blue());
        telemetry.addData("Current runtime", getRuntime());
        telemetry.addData("Target runtime", sTime);
    }

//    String format(OpenGLMatrix transformationMatrix) {
//        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";

}
