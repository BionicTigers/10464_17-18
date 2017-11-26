package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="Red Corner", group="Red")
public class RedCorner extends OpMode {

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
    private int startDeg;
    private int gameState2;
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
        startDeg = 0;
        gameState2 = 0;
        started = false;
        waitTime = 0;

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//        // OR...  Do Not Activate the Camera Monitor View, to save power
//        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        /*
//         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
//         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
//         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
//         * web site at https://developer.vuforia.com/license-manager.
//         *
//         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
//         * random data. As an example, here is a example of a fragment of a valid key:
//         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
//         * Once you've obtained a license key, copy the string from the Vuforia web site
//         * and paste it in to your code onthe next line, between the double quotes.
//         */
//        parameters.vuforiaLicenseKey = "AfBkGLH/////AAAAGUUS7r9Ue00upoglw/0yqTBLwhqYHpjwUK9zxmWMMFGuNGPjo/RjNOTsS8POmdQLHwe3/75saYsyb+mxz6p4O8xFwDT7FEYMmKW2NKaLKCA2078PZgJjnyw+34GV8IBUvi2SOre0m/g0X5eajpAhJ8ZFYNIMbUfavjQX3O7P0UHyXsC3MKxfjMzIqG1AgfRevcR/ONOJlONZw7YIZU3STjODyuPWupm2p7DtSY4TRX5opqFjGQVKWa2IlNoszsN0szgW/xJ1Oz5VZp4oDRS8efG0jOq1QlGw7IJOs4XXZMcsk0RW/70fVeBiT+LMzM8Ih/BUxtVVK4pcLMpb2wlzdKVLkSD8LOpaFWmgOhxtNz2M";
//
//        /*
//         * We also indicate which camera on the RC that we wish to use.
//         * Here we chose the back (HiRes) camera (for greater range), but
//         * for a competition robot, the front camera might be more convenient.
//         */
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//
//        /**
//         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
//         * in this data set: all three of the VuMarks in the game were created from this one template,
//         * but differ in their instance id information.
//         * @see VuMarkInstanceId
//         */
//        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        relicTemplate = relicTrackables.get(0);
//        relicTemplate.setName("relicVuMarkTemplate");
//
//        relicTrackables.activate();
//    }

    public void loop() {
        //super.gameState();
        if (!started) {
            started = true;
            waitTime = System.currentTimeMillis() + 2000;
        }
        switch (gameState2) {
            case 0: //Start
                if (waitTime <= System.currentTimeMillis()) {
                    gameState2 = 1;
                    //May be cause of nullPointer
                    //sTime = getRuntime();
                    //map.setRobot(10,8);
                }
                servo.setPosition(0.9);
                telemetry.addData("Time left", waitTime - System.currentTimeMillis());
                break;

            case 1:
                startDeg = motorBackRight.getCurrentPosition();
                if (sensorColor.red() > sensorColor.blue()) {
                    //motorBackLeft.setTargetPosition(1);
                    //motorBackLeft.setPower(.6);
                    //motorFrontRight.setTargetPosition(1);
                    motorFrontLeft.setPower(-.6);
                    motorBackRight.setPower(.6);
                    //motorFrontRight.setPower(-.6);
                } else {

                //if (sensorColor.red() < sensorColor.blue()) {
                    //motorBackLeft.setTargetPosition(1);
                    motorBackRight.setPower(-.6);
                    //motorFrontRight.setTargetPosition(1);
                    //motorFrontRight.setPower(.6);
                    //motorBackLeft.setPower(-.6);
                    motorFrontLeft.setPower(.6);
                }

                gameState2 = 2;
                break;

            case 2:
                if (Math.abs(startDeg - motorBackRight.getCurrentPosition()) > 300) {
                    motorBackLeft.setPower(-motorBackLeft.getPower());
                    motorFrontRight.setPower(-motorFrontRight.getPower());
                    motorBackRight.setPower(-motorBackRight.getPower());
                    motorFrontLeft.setPower(-motorFrontLeft.getPower());
                    gameState2 = 3;
                    startDeg = motorBackRight.getCurrentPosition();
                    servo.setPosition(.5);
                }
                break;

            case 3: //stops robot, why?
                if (Math.abs(startDeg - motorBackRight.getCurrentPosition()) > 300) {
                    motorBackLeft.setPower(0);
                    motorFrontRight.setPower(0);
                    motorBackRight.setPower(0);
                    motorFrontLeft.setPower(0);
                    gameState2 = 4;
                    waitTime = System.currentTimeMillis()+3000;
                    startDeg = motorBackRight.getCurrentPosition();
                }
                break;

            case 4:
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                    telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                    telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                    if (pose != null) {
                        VectorF trans = pose.getTranslation();
                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                        // Extract the X, Y, and Z components of the offset of the target relative to the robot
                        double tX = trans.get(0);
                        double tY = trans.get(1);
                        double tZ = trans.get(2);

                        // Extract the rotational components of the target relative to the robot
                        double rX = rot.firstAngle;
                        double rY = rot.secondAngle;
                        double rZ = rot.thirdAngle;
                    }
                }
                else {
                    telemetry.addData("VuMark", "not visible");
                }
                if (waitTime <= System.currentTimeMillis()){
                    gameState2 = 5;
                    startDeg = motorBackRight.getCurrentPosition();
                }
                break;

            case 5:
                motorBackRight.setPower(-0.5);
                motorBackLeft.setPower(-0.5);
                motorFrontLeft.setPower(0.5);
                motorFrontRight.setPower(0.5);

        }
        telemetry.addData("Motor degrees", motorBackRight.getCurrentPosition());
        telemetry.addData("Start degrees", startDeg);
        telemetry.addData("State", gameState2);
        telemetry.addData("Color value blue", sensorColor.blue());
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
