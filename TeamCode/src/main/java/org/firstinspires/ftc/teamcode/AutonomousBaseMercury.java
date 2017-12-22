package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public abstract class AutonomousBaseMercury extends OpMode {
    public final double HEADING_TOLERANCE = 7; //tolerance for heading calculations
    public final double DISTANCE_TOLERANCE = 1.0 / 10; //tolerance for heading calculations
    public final double DEGREES_TO_FEET = 3.96 * Math.PI / 1120 / 12;

    //EXPLANATION:
    // (wheel diameter) * pi / (encoder ticks per rotation) /(inches in a foot)
    // This converts encoder ticks into feet.
    //**WARNING** Always calculate distance CHANGED, since encoders have no
    // concept of direction, and we are moving across a 2D plane.

    public static class MoveState {
        public static final int STOP = 0;
        public static final int FORWARD = 1;
        public static final int BACKWARD = 2;
        public static final int LEFT = 3;
        public static final int RIGHT = 4;
        public static final int TURN_TOWARDS_GOAL = 5;
        public static final int BACKWARD_SLOW = 9;
        public static final int FULL_STOP = 12;
        public static final int STRAFE_TOWARDS_GOAL = 15;
        public static final int TURN_TOWARDS_ANGLE = 16;
        public static final int LEFT_SLOW = 17;
        public static final int RIGHT_SLOW = 18;
        public static final int TURN_TOWARDS_ANGLE_SLOW = 19;
        public static final int DEFINE_COLOR = 901937;
    }

    int i;
    public OpenGLMatrix lastLocation;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor top;
    public DcMotor front;
    public Servo franny = null; //left servo
    public Servo mobert = null; //right servo
    public Servo servo;
    public VuforiaLocalizer vuforia;
    public VuforiaTrackable relicTemplate;
    public RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
    public int startDeg;
    public boolean started;
    public ColorSensor sensorColor;
    public double waitTime;
    public int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    public VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    public BNO055IMU imu;
    public boolean blue;//true if blue detected

    //We stateful now
    int gameState;
   public int moveState;

   public double power;
   public Orientation angles;
   public Acceleration gravity;
   public double heading;
   public double desiredAngle;
   public boolean turnRight;
   public int cDistF, lDistF, dDistF; //Forward distance variables
   public int cDistS, lDistS, dDistS; //Sideways distance variables
   public int cDistW, lDistW, dDistW; //Sideways distance variables
   public double sTime; //Shooting timer
   public double pTime; //Button presser timer
   public double tDiff;
   public ElapsedTime runtime = new ElapsedTime();

   public int startPos = 6;
    Map map = new Map(startPos); //this map object will allow for easy manipulations.

    final double SCALE_FACTOR = 255;
    float hsvValues[] = {0F, 0F, 0F};

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

//        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        I2cAddr colorAddrLeft = I2cAddr.create8bit(0x3C);
        I2cAddr colorAddrRight = I2cAddr.create8bit(0x4C);
        // sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        // sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void moveState() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
        String filename = "BNO055IMUCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());
        telemetry.log().add("saved to '%s'", filename);

        switch (moveState) {
            case MoveState.STOP:
                // Halts all drivetrain movement of the robot
                motorFrontRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                break;
            case MoveState.FORWARD:
                // Moves the bot forward at half speed
                power = .50; //power coefficient
                if (map.distanceToGoal() > DISTANCE_TOLERANCE) {
                    motorFrontRight.setPower(power);
                    motorFrontLeft.setPower(power);
                    motorBackLeft.setPower(power);
                    motorBackRight.setPower(power);
                }
                break;
            case MoveState.BACKWARD:
                // Moves the bot backwards at half speed
                power = .50; //power coefficient
                if (map.distanceToGoal() > DISTANCE_TOLERANCE) {
                    motorFrontRight.setPower(-power);
                    motorFrontLeft.setPower(-power);
                    motorBackLeft.setPower(-power);
                    motorBackRight.setPower(-power);
                }
                break;
            case MoveState.BACKWARD_SLOW:
                // Moves the bot backwards at minimum speed
                power = .25; //power coefficient
                if (map.distanceToGoal() > DISTANCE_TOLERANCE) {
                    motorFrontRight.setPower(-power);
                    motorFrontLeft.setPower(-power);
                    motorBackLeft.setPower(-power);
                    motorBackRight.setPower(-power);
                }
                //servoLeftButton.setPosition(.5); // HACK
                break;
            case MoveState.LEFT:
                // Moves the bot left at half speed
                power = .50; //power coefficient
                if (map.distanceToGoal() > DISTANCE_TOLERANCE) {
                    motorFrontRight.setPower(power);
                    motorFrontLeft.setPower(-power);
                    motorBackLeft.setPower(-power);
                    motorBackRight.setPower(power);
                }
                break;
            case MoveState.LEFT_SLOW:
                // Moves the bot left at half speed
                power = .25; //power coefficient
                if (map.distanceToGoal() > DISTANCE_TOLERANCE) {
                    motorFrontRight.setPower(power);
                    motorFrontLeft.setPower(-power);
                    motorBackLeft.setPower(-power);
                    motorBackRight.setPower(power);
                }
                break;
            case MoveState.RIGHT:
                // Moves the bot right at half speed
                power = .50; //power coefficient
                if (map.distanceToGoal() > DISTANCE_TOLERANCE) {
                    motorFrontRight.setPower(-power);
                    motorFrontLeft.setPower(power);
                    motorBackLeft.setPower(power);
                    motorBackRight.setPower(-power);
                }
                break;
            case MoveState.RIGHT_SLOW:
                // Moves the bot right at half speed
                power = .25; //power coefficient
                if (map.distanceToGoal() > DISTANCE_TOLERANCE) {
                    motorFrontRight.setPower(-power);
                    motorFrontLeft.setPower(power);
                    motorBackLeft.setPower(power);
                    motorBackRight.setPower(-power);
                }
                break;
            case MoveState.STRAFE_TOWARDS_GOAL:
                // Moves the bot towards the goal, while always pointing at desiredAngle
                double P = .50;
                double H = Math.toRadians(heading);
                double Ht = Math.toRadians(map.angleToGoal());

                motorFrontRight.setPower(-P * Math.sin(H - Ht));
                motorFrontLeft.setPower(-P * Math.sin(H - Ht));
                motorBackLeft.setPower(P * Math.cos(H - Ht));
                motorBackRight.setPower(P * Math.cos(H - Ht));
                break;
            case MoveState.TURN_TOWARDS_GOAL:
                // Orients the bot to face the goal
                power = .25;
                if (heading <= 180) {
                    turnRight = heading <= map.angleToGoal() && heading + 180 >= map.angleToGoal();
                } else {
                    turnRight = !(heading >= map.angleToGoal() && heading - 180 <= map.angleToGoal());
                }

                if (turnRight) {
                    motorFrontRight.setPower(power);
                    motorFrontLeft.setPower(-power);
                    motorBackLeft.setPower(-power);
                    motorBackRight.setPower(power);
                } else {
                    motorFrontRight.setPower(-power);
                    motorFrontLeft.setPower(power);
                    motorBackLeft.setPower(power);
                    motorBackRight.setPower(-power);
                }

                break;
            case MoveState.TURN_TOWARDS_ANGLE:
                // Orients the bot to face at desiredAngle.
                power = .3;
                if (heading <= 180) {
                    turnRight = heading <= desiredAngle && heading + 180 >= desiredAngle;
                } else {
                    turnRight = !(heading >= desiredAngle && heading - 180 <= desiredAngle);
                }

                if (turnRight) {
                    motorFrontRight.setPower(power);
                    motorFrontLeft.setPower(-power);
                    motorBackLeft.setPower(power);
                    motorBackRight.setPower(-power);
                } else {
                    motorFrontRight.setPower(-power);
                    motorFrontLeft.setPower(power);
                    motorBackLeft.setPower(power);
                    motorBackRight.setPower(-power);
                }
                break;
            case MoveState.DEFINE_COLOR:
                Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
        }

        //break;
            /*case MoveState.TURN_TOWARDS_ANGLE_SLOW:
                // Orients the bot to face at desiredAngle.
                power = .2;
                if(heading<=180){
                    turnRight = heading <= desiredAngle && heading + 180 >= desiredAngle;
                }else{
                    turnRight = !(heading >= desiredAngle && heading - 180 <= desiredAngle);
                }

                if(turnRight){
                    motorFrontRight.setPower(power);
                    motorFrontLeft.setPower(-power);
                    motorBackLeft.setPower(-power);
                    motorBackRight.setPower(power);
                }else{
                    motorFrontRight.setPower(-power);
                    motorFrontLeft.setPower(power);
                    motorBackLeft.setPower(power);
                    motorBackRight.setPower(-power);
                }
                break;*/

        // case MoveState.FULL_STOP:
        // Stop ALL robot movement, and resets servo to default pos

        // servo.setPosition(.5);
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
    // motorConveyor.setPower(0);  }

    // break;

    //  map.moveRobot(dDistS * DEGREES_TO_FEET, (heading+90%360));
    // map.moveRobot(dDistF * DEGREES_TO_FEET, heading);
//}

    public void gameState() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
        String filename = "BNO055IMUCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());
        telemetry.log().add("saved to '%s'", filename);

        lDistF = cDistF;
        cDistF = (motorBackLeft.getCurrentPosition()
                + motorBackRight.getCurrentPosition()) / 2;
        dDistF = cDistF - lDistF;

        lDistS = cDistS;
        cDistS = (motorFrontRight.getCurrentPosition()
                + motorFrontLeft.getCurrentPosition()) / 2;
        dDistW = cDistW - lDistW;

        if (tDiff == 0) {
            tDiff = getRuntime();
        }
    }

    public void telemetry() {
        telemetry.addData("angle to goal ", map.angleToGoal());
        telemetry.addData("Runtime ", getRuntime());

        telemetry.addData("dist from goal ", map.distanceToGoal());
        telemetry.addData("goal (x,y) ", "(" +
                map.getGoalX() + "," +
                map.getGoalY() + ")");
        telemetry.addData("Robot(x,y) ", "(" +
                map.getRobotX() + "," +
                map.getRobotY() + ")");
        telemetry.addData("robot theta", heading);
        telemetry.addData("Am I lined up?", linedUpAngle(5));
        telemetry.addData("Desired Angle", desiredAngle);
        telemetry.addData("moveState", moveState);
        telemetry.addData("gameState", gameState);
    }

    @Override
    public void loop() {
        gameState();
        moveState();
        telemetry();
    }

    public boolean linedUp() {
        return Math.abs(heading - map.angleToGoal()) < HEADING_TOLERANCE || (heading > 360 - HEADING_TOLERANCE && map.angleToGoal() < HEADING_TOLERANCE || (heading < HEADING_TOLERANCE && map.angleToGoal() > 360 - HEADING_TOLERANCE));
    }

    public boolean linedUpAngle() {
        return Math.abs(heading - desiredAngle) < HEADING_TOLERANCE || (heading > 360 - HEADING_TOLERANCE && desiredAngle < HEADING_TOLERANCE || (heading < HEADING_TOLERANCE && desiredAngle > 360 - HEADING_TOLERANCE));
    }

    public boolean linedUpAngle(int HEADING_TOLERANCE) {
        return Math.abs(heading - desiredAngle) < HEADING_TOLERANCE || (heading > 360 - HEADING_TOLERANCE && desiredAngle < HEADING_TOLERANCE || (heading < HEADING_TOLERANCE && desiredAngle > 360 - HEADING_TOLERANCE));
    }

    public boolean linedUpRev() {
        return Math.abs(heading - map.angleToGoalRev()) < HEADING_TOLERANCE || (heading > 360 - HEADING_TOLERANCE && map.angleToGoalRev() < HEADING_TOLERANCE || (heading < HEADING_TOLERANCE && map.angleToGoalRev() > 360 - HEADING_TOLERANCE));
    }

    public double actualRuntime() {
        return getRuntime() - tDiff;
    }

    public class ConceptVuMarkIdentification extends LinearOpMode {

        public static final String TAG = "Vuforia VuMark";

        OpenGLMatrix lastLocation = null;

        /**
         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
         * localization engine.
         */
        VuforiaLocalizer vuforia;

        @Override
        public void runOpMode() {

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            // OR...  Do Not Activate the Camera Monitor View, to save power
            // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
            parameters.vuforiaLicenseKey = "AfBkGLH/////AAAAGUUS7r9Ue00upoglw/0yqTBLwhqYHpjwUK9zxmWMMFGuNGPjo/RjNOTsS8POmdQLHwe3/75saYsyb+mxz6p4O8xFwDT7FEYMmKW2NKaLKCA2078PZgJjnyw+34GV8IBUvi2SOre0m/g0X5eajpAhJ8ZFYNIMbUfavjQX3O7P0UHyXsC3MKxfjMzIqG1AgfRevcR/ONOJlONZw7YIZU3STjODyuPWupm2p7DtSY4TRX5opqFjGQVKWa2IlNoszsN0szgW/xJ1Oz5VZp4oDRS8efG0jOq1QlGw7IJOs4XXZMcsk0RW/70fVeBiT+LMzM8Ih/BUxtVVK4pcLMpb2wlzdKVLkSD8LOpaFWmgOhxtNz2M";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

            /**
             * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
             * in this data set: all three of the VuMarks in the game were created from this one template,
             * but differ in their instance id information.
             * @see VuMarkInstanceId
             */
            VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

            telemetry.addData(">", "Press Play to start");
            telemetry.update();
            waitForStart();

            relicTrackables.activate();

            while (opModeIsActive()) {

                /**
                 * See if any of the instances of {@link relicTemplate} are currently visible.
                 * {@link RelicRecoveryVuMark} is an enum which can have the following values:
                 * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
                 * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
                 */
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                    telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
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
                } else {
                    telemetry.addData("VuMark", "not visible");
                }


                telemetry.update();
            }
        }

        String format(OpenGLMatrix transformationMatrix) {
            return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
        }
    }
}