package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Venus Mecanum", group="Venus")


public class onlyMecanum extends OpMode {

    // DRIVETRAIN \\
    private DcMotor motorOne; //fR
    private DcMotor motorTwo; //fL
    private DcMotor motorThree; //BR
    private DcMotor motorFour; //BL

    public void init() {

// DRIVETRAIN \\
        motorOne = hardwareMap.dcMotor.get("frontRight");
        motorTwo = hardwareMap.dcMotor.get("frontLeft");
        motorThree = hardwareMap.dcMotor.get("backRight");
        motorFour = hardwareMap.dcMotor.get("backLeft");

        motorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorThree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFour.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorTwo.setDirection(DcMotor.Direction.REVERSE);
        motorFour.setDirection(DcMotor.Direction.REVERSE); }

    public void loop() {
        telemetry.update();
        ///////////////
        // GAMEPAD 1 //
        ///////////////

        double P = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
        double rightX = -gamepad1.right_stick_x;
        double sinRAngle = Math.sin(robotAngle);
        double cosRAngle = Math.cos(robotAngle);

        final double v1 = (P * sinRAngle) + (P * cosRAngle) + rightX;
        final double v2 = (P * sinRAngle) - (P * cosRAngle) - rightX;
        final double v3 = (P * sinRAngle) - (P * cosRAngle) + rightX;
        final double v4 = (P * sinRAngle) + (P * cosRAngle) - rightX;

        motorOne.setPower(v1);
        motorTwo.setPower(v2);
        motorThree.setPower(v3);
        motorFour.setPower(v4);


 } }