package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Mechanum Protobot Tank", group="Protobot")

public class MechanumProtoBot extends OpMode    {

    ElapsedTime runtime = new ElapsedTime();

    private Orientation angles;
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
    private BNO055IMU imu;

    public void init()
    {
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
        runtime.reset();
        BNO055IMU imu;
    }

    public void loop()
    {
        ////////////////
        // MAIN DRIVE //
        ////////////////

        double r = Math.hypot(-gamepad1.right_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.right_stick_x, -gamepad1.left_stick_y) - Math.PI / 4;
        double rightX = gamepad1.left_stick_x;
        //final double v1 = r * Math.sin(robotAngle) + rightX;
        //final double v2 = r * Math.cos(robotAngle) + rightX;
        //final double v3 = r * Math.cos(robotAngle) - rightX;
        //final double v4 = r * Math.sin(robotAngle) - rightX;

        //motorFrontRight.setPower(v1);
        //motorFrontLeft.setPower(v2);
        //motorBackRight.setPower(v3);
        //motorBackLeft.setPower(v4);

        /////////////////////////////
        // ORIENTATION CALIBRATION //
        /////////////////////////////

        //if (gamepad1.left_stick_button) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        final double v5 = r * Math.sin(robotAngle) + rightX + angles.firstAngle;
        final double v6 = r * Math.cos(robotAngle) + rightX + angles.firstAngle;
        final double v7 = r * Math.cos(robotAngle) - rightX + angles.firstAngle;
        final double v8 = r * Math.sin(robotAngle) - rightX + angles.firstAngle;

        motorFrontRight.setPower(v5);
        motorFrontLeft.setPower(v6);
        motorBackRight.setPower(v7);
        motorBackLeft.setPower(v8);

        //}

        ///////////////////////
        // COLLECTION SERVOS //
        ///////////////////////

        if (gamepad2.b) {

            franny.setPosition(.1);
            mobert.setPosition(.9);

        } else if (gamepad2.x) {

            franny.setPosition(1);
            mobert.setPosition(0);
            }

//
//        if (gamepad2.left_bumper) {
//            if (left < 1.0) {
//                left += .01;
//            }
//            franny.setPosition(left);
//        } else if (gamepad2.left_trigger > .7) {
//            if (left > 0.0) {
//                left -= .01;
//            }
//            franny.setPosition(left);
//        }
//
//        if (gamepad2.right_bumper) {
//            if (right > 0) {
//                right -= .01;
//            }
//            mobert.setPosition(right);
//        } else if (gamepad2.right_trigger > .7) {
//            if (right < 1) {
//                right += .01;
//            }
//            mobert.setPosition(right);
//        }

        telemetry.addData("Left", left);
        telemetry.addData("Right", right);

        ///////////////////
        // BELT CONTROLS //
        ///////////////////

        if (gamepad2.dpad_up) {
            top.setPower(-0.3);
        } else if (gamepad2.dpad_down) {
            top.setPower(0.3);
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