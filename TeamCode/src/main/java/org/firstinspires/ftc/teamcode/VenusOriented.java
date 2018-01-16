package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;


@TeleOp(name="Venus Oriented", group="Venus")


public class VenusOriented extends OpMode {

    //DRIVETRAIN\\
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;
    //CAR WASHER\\
    public DcMotor billiam;
    //GLYPH FLIPPER\\
    public Servo hamilton = null;
    //LIFT\\
    public DcMotor evangelino; //Left
    public DcMotor wilbert; //Right
    //HAMMER\\
    public Servo eddie = null; //Flicker
    public Servo clark = null; //Dropper
    //RELIC\\
    //public DcMotor georgery; //Extender
    public Servo brandy = null; //Elbow
    public Servo franny = null; //Left Finger
    //public Servo mobert = null; //Right Finger
    //IMU\\
    BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    //VARIABLES\\
    public int calibToggle;
    public double elbowPos;
    private double time = getRuntime();
    private int targetPos;
    private double topPos;


public void init() {

    ///DRIVETRAIN\\
    motorFrontRight = hardwareMap.dcMotor.get("frontRight");
    motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
    motorBackRight = hardwareMap.dcMotor.get("backRight");
    motorBackLeft = hardwareMap.dcMotor.get("backLeft");

    motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    //CAR WASHER\\
    billiam = hardwareMap.dcMotor.get("billiam");
    //GLYPH FLIPPER\\
    hamilton = hardwareMap.servo.get("hamilton");
    //LIFT\\
    evangelino = hardwareMap.dcMotor.get("evangelino");
    wilbert = hardwareMap.dcMotor.get("wilbert");

//    evangelino.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    evangelino.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
//    wilbert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    wilbert.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //HAMMER\\
    eddie = hardwareMap.servo.get("eddie");
    clark = hardwareMap.servo.get("clark");
    //RELIC\\
//    georgery = hardwareMap.dcMotor.get("georgery");
    brandy = hardwareMap.servo.get("brandy");
    franny = hardwareMap.servo.get("franny");
//    mobert = hardwareMap.servo.get("mobert");
    //IMU\\
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
    parameters.loggingEnabled = true;
    parameters.loggingTag = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);
    //VARIABLES\\
    calibToggle = 0;
    motorBackRight.setDirection(DcMotor.Direction.REVERSE);
    motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
    elbowPos = 0.00;
    targetPos = 0;
    topPos = 0;
}


public void loop() {
    telemetry.addData("evangelino", evangelino.getCurrentPosition());
    telemetry.addData("TopPos", topPos);
    telemetry.addData("targetPos", targetPos);
    telemetry.update();
    topPos = evangelino.getCurrentPosition();

    ///////////////
    // GAMEPAD 1 //
    ///////////////

    if (gamepad1.y) { //orientation calibration
        // Get the calibration data
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
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
    }

    if (gamepad1.x) { //toggle on
        calibToggle = 1;
    }

    if (gamepad1.b) { //toggle off
        calibToggle = 0;
    }

    if (calibToggle == 1) { //when toggled we are oriented with this math
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double P = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
        double rightX = -gamepad1.right_stick_x;


        final double v5 = P * Math.sin(robotAngle - angles.firstAngle) + P * Math.cos(robotAngle - angles.firstAngle) - rightX;
        final double v6 = P * Math.sin(robotAngle - angles.firstAngle) - P * Math.cos(robotAngle - angles.firstAngle) + rightX;
        final double v7 = P * Math.sin(robotAngle - angles.firstAngle) - P * Math.cos(robotAngle - angles.firstAngle) - rightX;
        final double v8 = P * Math.sin(robotAngle - angles.firstAngle) + P * Math.cos(robotAngle - angles.firstAngle) + rightX;

        motorFrontLeft.setPower(v5);//1
        motorFrontRight.setPower(v6);//2
        motorBackLeft.setPower(v7);//3
        motorBackRight.setPower(v8);//4

        //some telemetry for testing purposes
        telemetry.addData("robotAngle", robotAngle);
        telemetry.addData("P", P);
        telemetry.addData("rightX", rightX);
        telemetry.addData("v5", v5);
        telemetry.addData("v6", v6);
        telemetry.addData("angles.firstAngle", angles.firstAngle);

    } else if (calibToggle == 0) { //regular drive
        double P = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
        double rightX = -gamepad1.right_stick_x;
        double sinRAngle = Math.sin(robotAngle);
        double cosRAngle = Math.cos(robotAngle);

        final double v1 = (P * sinRAngle) + (P * cosRAngle) - rightX;
        final double v2 = (P * sinRAngle) - (P * cosRAngle) + rightX;
        final double v3 = (P * sinRAngle) - (P * cosRAngle) - rightX;
        final double v4 = (P * sinRAngle) + (P * cosRAngle) + rightX;

        motorFrontRight.setPower(v1);
        motorFrontLeft.setPower(v2);
        motorBackRight.setPower(v3);
        motorBackLeft.setPower(v4);
    }
    ///////////////
    // GAMEPAD 2 //
    ///////////////

    //FOUR BAR WITH ENCODERS//
//    if (evangelino.getCurrentPosition() < targetPos - 10) { //up
//        evangelino.setPower(-0.85);
//        wilbert.setPower(0.85);
//        evangelino.getCurrentPosition();
//        telemetry.update();
//    } else if (evangelino.getCurrentPosition() > targetPos + 10) { //down
//        evangelino.setPower(0.85);
//        wilbert.setPower(-0.85);
//        evangelino.getCurrentPosition();
//        telemetry.update();
//    } else {
//        evangelino.setPower(0);
//        wilbert.setPower(0);
//        evangelino.getCurrentPosition();
//        telemetry.update();
//    }
//
    if (gamepad2.a) {
        evangelino.setPower(-.85);
        wilbert.setPower(.85);
    }
    else {
        evangelino.setPower(0);
        wilbert.setPower(0);
    }


    if (gamepad2.y) {
        evangelino.setPower(.85);
        wilbert.setPower(-.85);
    }
    else {

        evangelino.setPower(0);
        wilbert.setPower(0);
    }


    //FOUR BAR WITHOUT ENCODERS//
//    if (gamepad2.a) { //up
//        wilbert.setPower(-0.85);
//        evangelino.setPower(0.85);
//    }
//    if (gamepad2.y) { //down
//        evangelino.setPower(-0.85);
//        wilbert.setPower(0.85);
//    }
//    if (gamepad2.x) {
//        wilbert.setPower(0);
//        evangelino.setPower(0);
//    }

    //RELIC//
//    if (gamepad1.dpad_up) {
//        georgery.setPower(0.75); }
//    else if (gamepad1.dpad_down) {
//        georgery.setPower(-0.75); }
//    else {
//        georgery.setPower(0.0); }
//
//    if (gamepad1.right_bumper) {
//        elbowPos += .01;
//        brandy.setPosition(elbowPos); }
//
//    else if (gamepad1.left_bumper) {
//        elbowPos -= .01;
//        brandy.setPosition(elbowPos); }
//
//    if (gamepad1.right_trigger > .7) {
//        franny.setPosition(0.00);
//        mobert.setPosition(0.00); }
//    else if (gamepad1.left_trigger > .7) {
//        franny.setPosition(1.00);
//        mobert.setPosition(1.00); }

//        //MAKESHIFT RELIC//
//        if (gamepad2.dpad_up) {
//            brandy.setPosition(0.3);
//        }
//        if (gamepad2.dpad_down) {
//            brandy.setPosition(0.9);
//        }
//        if (gamepad2.dpad_left) {
//            franny.setPosition(0.5);
//        }
//        if (gamepad2.dpad_right) {
//            franny.setPosition(1);

        if (gamepad2.right_bumper) {
            billiam.setPower(1);
        } else if (gamepad2.right_trigger > .7) {
            billiam.setPower(-1);
        } else {
            billiam.setPower(0.00);
        }

        if (gamepad2.left_trigger > .7) {
            hamilton.setPosition(0.3);
        } else if (gamepad2.left_bumper) {
            hamilton.setPosition(1);
        } else if (gamepad2.dpad_up) {
            hamilton.setPosition(0.4);
        }

    }
///////////////////////////////////////////////////////////////////////////////////////////////////

    private String composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
        return formatAngle(angles.angleUnit, angles.firstAngle);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatRadians(AngleUnit.RADIANS.fromUnit(angleUnit, angle));
    }

    String formatRadians(double radians) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.RADIANS.normalize(radians));
    } }