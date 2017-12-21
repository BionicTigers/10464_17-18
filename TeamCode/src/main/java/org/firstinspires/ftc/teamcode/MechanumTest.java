package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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


@TeleOp(name="Mechanum Test", group="TestyOp")


public class MechanumTest extends OpMode {

    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;

    public int calibToggle;


public void init() {

    motorFrontRight = hardwareMap.dcMotor.get("frontRight");
    motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
    motorBackRight = hardwareMap.dcMotor.get("backRight");
    motorBackLeft = hardwareMap.dcMotor.get("backLeft");

    calibToggle = 1; }


public void loop() {

    telemetry.addData("Calib Toggle", calibToggle);

    telemetry.update();

    if (gamepad1.a) {
        calibToggle = 1; }

    if (gamepad1.x) {
        calibToggle = 2; }

    if (gamepad1.y) {
        calibToggle = 3; }

    if (gamepad1.b) {
        calibToggle = 4; }

    if (calibToggle == 1) {
        double P = -(Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y));
        double robotAngle = -(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x));
        double rightX = -gamepad1.right_stick_x;
        double sinRAngle = Math.sin(robotAngle);
        double cosRAngle = Math.cos(robotAngle);

        final double v11 = (P * sinRAngle) - (P * cosRAngle) - rightX;
        final double v12 = (P * sinRAngle) + (P * cosRAngle) + rightX;
        final double v13 = (P * sinRAngle) + (P * cosRAngle) - rightX;
        final double v14 = (P * sinRAngle) - (P * cosRAngle) + rightX;

        motorFrontRight.setPower(v11);
        motorFrontLeft.setPower(v12);
        motorBackRight.setPower(v13);
        motorBackLeft.setPower(v14); }

    if (calibToggle == 2) {
        double P = -(Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y));
        double robotAngle = -(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x));
        double rightX = -gamepad1.right_stick_x;
        double sinRAngle = Math.sin(robotAngle);
        double cosRAngle = Math.cos(robotAngle);

        final double v21 = (P * sinRAngle) - (P * cosRAngle) - rightX;
        final double v22 = (P * sinRAngle) + (P * cosRAngle) + rightX;
        final double v23 = (P * sinRAngle) + (P * cosRAngle) - rightX;
        final double v24 = (P * sinRAngle) - (P * cosRAngle) + rightX;

        motorFrontRight.setPower(v21);
        motorFrontLeft.setPower(v22);
        motorBackRight.setPower(v23);
        motorBackLeft.setPower(v24); }

    if (calibToggle == 3) {
        double P = -(Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y));
        double robotAngle = -(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x));
        double rightX = -gamepad1.right_stick_x;
        double sinRAngle = Math.sin(robotAngle);
        double cosRAngle = Math.cos(robotAngle);

        final double v31 = (P * sinRAngle) - (P * cosRAngle) - rightX;
        final double v32 = (P * sinRAngle) + (P * cosRAngle) + rightX;
        final double v33 = (P * sinRAngle) + (P * cosRAngle) - rightX;
        final double v34 = (P * sinRAngle) - (P * cosRAngle) + rightX;

        motorFrontRight.setPower(v31);
        motorFrontLeft.setPower(v32);
        motorBackRight.setPower(v33);
        motorBackLeft.setPower(v34); }

    if (calibToggle == 4) {
        double P = -(Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y));
        double robotAngle = -(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x));
        double rightX = -gamepad1.right_stick_x;
        double sinRAngle = Math.sin(robotAngle);
        double cosRAngle = Math.cos(robotAngle);

        final double v41 = (P * sinRAngle) - (P * cosRAngle) - rightX;
        final double v42 = (P * sinRAngle) + (P * cosRAngle) + rightX;
        final double v43 = (P * sinRAngle) + (P * cosRAngle) - rightX;
        final double v44 = (P * sinRAngle) - (P * cosRAngle) + rightX;

        motorFrontRight.setPower(v41);
        motorFrontLeft.setPower(v42);
        motorBackRight.setPower(v43);
        motorBackLeft.setPower(v44); } } }