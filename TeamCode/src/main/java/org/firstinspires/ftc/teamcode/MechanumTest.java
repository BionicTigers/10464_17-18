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


@TeleOp(name="Mechanum Test", group="Protobot")


public class MechanumTest extends OpMode {

    public Orientation angles;
    public Acceleration gravity;

    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;

    BNO055IMU imu;
    public int calibToggle;


public void init() {

    motorFrontRight = hardwareMap.dcMotor.get("frontRight");
    motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
    motorBackRight = hardwareMap.dcMotor.get("backRight");
    motorBackLeft = hardwareMap.dcMotor.get("backLeft"); }


public void loop() {

    telemetry.update();

    if (gamepad1.a) {
        // Get the calibration data
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
        telemetry.log().add("saved to '%s'", filename); }

    if (gamepad1.x) {
        calibToggle += 1; }

    if ((calibToggle & 1) != 0) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double H = (angles.firstAngle * Math.PI) / 180;
        double rightX = gamepad1.left_stick_x;
        double P = Math.hypot(gamepad1.right_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.right_stick_x, gamepad1.left_stick_y) + Math.PI / 4;

        final double v5 = -P * Math.sin(robotAngle) + rightX - H;
        final double v6 = -P * Math.cos(robotAngle) - rightX - H;
        final double v7 = -P * Math.cos(robotAngle) + rightX - H;
        final double v8 = -P * Math.sin(robotAngle) - rightX - H;

        motorFrontRight.setPower(v5);
        motorFrontLeft.setPower(v6);
        motorBackRight.setPower(v7);
        motorBackLeft.setPower(v8); }

    else {
        double P = Math.hypot(gamepad1.right_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.right_stick_x, gamepad1.left_stick_y) + Math.PI / 4;
        double rightX = gamepad1.left_stick_x;

        final double v1 = -P * Math.sin(robotAngle) + rightX;
        final double v2 = -P * Math.cos(robotAngle) - rightX;
        final double v3 = -P * Math.cos(robotAngle) + rightX;
        final double v4 = -P * Math.sin(robotAngle) - rightX;

        motorFrontRight.setPower(v1);
        motorFrontLeft.setPower(v2);
        motorBackRight.setPower(v3);
        motorBackLeft.setPower(v4); } } }