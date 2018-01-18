package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Mercury Kids Code", group="KidsCode")


public class MercuryKidsCode extends OpMode {

    //DRIVETRAIN\\
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;


    public void init() {

        //DRIVETRAIN\\
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
    }


    public void loop() {

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

        motorFrontRight.setPower(v1);
        motorFrontLeft.setPower(v2);
        motorBackRight.setPower(v3);
        motorBackLeft.setPower(v4);
    }
}