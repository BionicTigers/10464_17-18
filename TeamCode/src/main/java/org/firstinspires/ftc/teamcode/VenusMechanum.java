package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="Venus Mechanum", group="Protobot")


public class VenusMechanum extends OpMode {

    private Orientation angles;
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    private BNO055IMU imu;


public void init() {

    motorFrontRight = hardwareMap.dcMotor.get("frontRight");
    motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
    motorBackRight = hardwareMap.dcMotor.get("backRight");
    motorBackLeft = hardwareMap.dcMotor.get("backLeft");

    BNO055IMU imu; }


public void loop() {

    ////////////////
    // MAIN DRIVE //
    ////////////////

    double r = Math.hypot(-gamepad1.right_stick_x, -gamepad1.left_stick_y);
    double robotAngle = Math.atan2(-gamepad1.right_stick_x, -gamepad1.left_stick_y) - Math.PI / 4;
    double rightX = gamepad1.left_stick_x;
    final double v1 = r * Math.sin(robotAngle) + rightX;
    final double v2 = r * Math.cos(robotAngle) + rightX;
    final double v3 = r * Math.cos(robotAngle) - rightX;
    final double v4 = r * Math.sin(robotAngle) - rightX;

    motorFrontRight.setPower(v1);
    motorFrontLeft.setPower(v2);
    motorBackRight.setPower(v3);
    motorBackLeft.setPower(v4); }


}