package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Oriented ProtoBot", group="Protobot")

public abstract class OrientedProtoBot extends OpMode {

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor top;
    private DcMotor front;
    private Servo franny = null;
    private Servo mobert = null;
    private double left;
    private double right;
    ModernRoboticsI2cGyro gyro;
    HardwareMap hwMap;

    public void HardwareOmniRobot(){

        hwMap = null;
    }

    public void init(HardwareMap ahwMap, boolean rungyro) {
        hwMap = ahwMap;
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backRight");
        franny = hardwareMap.servo.get("franny");
        mobert = hardwareMap.servo.get("mobert");
        top = hardwareMap.dcMotor.get("top");
        front = hardwareMap.dcMotor.get("front");
        left = 0.32;
        right = .60;
        if(rungyro == true) {
            gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
            gyro.calibrate();
        }
    }

    public void loop() {
        /////////////////////////////
        // ORIENTATION CALIBRATION //
        /////////////////////////////


        int a = gyro.getHeading();

        telemetry.addData("heading", a);
        double r = Math.hypot(-gamepad1.right_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.right_stick_x, -gamepad1.left_stick_y) - Math.PI / 4;
        double rightX = gamepad1.left_stick_x;
        final double v5 = r * Math.sin(robotAngle) + rightX + a;
        final double v6 = r * Math.cos(robotAngle) + rightX + a;
        final double v7 = r * Math.cos(robotAngle) - rightX + a;
        final double v8 = r * Math.sin(robotAngle) - rightX + a;

        motorFrontRight.setPower(v5);
        motorFrontLeft.setPower(v6);
        motorBackRight.setPower(v7);
        motorBackLeft.setPower(v8);


        ///////////////////////
        // COLLECTION SERVOS //
        ///////////////////////

        if (gamepad2.x) {
            if (left < 0.3 && right > 0.32) {
                left += .01;
                right -= .01;
            }
            franny.setPosition(left);
            mobert.setPosition(right);
        } else if (gamepad2.b) {
            if (left > 0.00 && right < 1.0) {
                left -= .01;
                right += .01;
            }
            franny.setPosition(left);
            mobert.setPosition(right);
        }

        if (gamepad2.left_bumper) {
            if (left < 0.3) {
                left += .01;
            }
            franny.setPosition(left);
        } else if (gamepad2.left_trigger > .7) {
            if (left > 0.0) {
                left -= .01;
            }
            franny.setPosition(left);
        }

        if (gamepad2.right_bumper) {
            if (right > 0.32) {
                right -= .01;
            }
            mobert.setPosition(right);
        } else if (gamepad2.right_trigger > .7) {
            if (right < 1) {
                right += .01;
            }
            mobert.setPosition(right);
        }

        telemetry.addData("Left", left);
        telemetry.addData("Right", right);
        telemetry.addData("franny", franny);
        telemetry.addData("mobert", mobert);


        ///////////////////
        // BELT CONTROLS //
        ///////////////////

        if (gamepad2.dpad_up) {
            top.setPower(-0.45);
        } else if (gamepad2.dpad_down) {
            top.setPower(0.45);
        } else {
            top.setPower(0);
        }

        if (gamepad2.y) {
            front.setPower(-0.7);
        } else if (gamepad2.a) {
            front.setPower(0.7);
        } else {
            front.setPower(0);
        }
    }
}
///////////////////////////////////////////////////////////////////////////
//Now for the part of the code we edit when commit says theres no changes//
///////////////////////////////////////////////////////////////////////////
//Push Please