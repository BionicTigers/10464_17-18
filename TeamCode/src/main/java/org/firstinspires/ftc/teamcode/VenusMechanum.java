package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Venus Mechanum", group="Venus")


public class VenusMechanum extends OpMode {

// DRIVETRAIN \\
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
// CAR WASHER \\
    private DcMotor billiam;
// GLYPH FLIPPER \\
    private Servo hamilton = null;
// LIFT \\
    private DcMotor evangelino; // Left
    private DcMotor wilbert; // Right
    private Servo donneet; // Gate
// HAMMER \\
    private Servo eddie = null; // Flicker
    private Servo clark = null; // Dropper
// RELIC \\
    //private DcMotor georgery; // Extender
    //private Servo brandy = null; // Elbow
    //private Servo franny = null; // Left Finger
    //private Servo mobert = null; // Right Finger
// VARIABLES \\
    private double elbowPos;


public void init() {

// DRIVETRAIN \\
    motorFrontRight = hardwareMap.dcMotor.get("frontRight");
    motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
    motorBackRight = hardwareMap.dcMotor.get("backRight");
    motorBackLeft = hardwareMap.dcMotor.get("backLeft");

    motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
// CAR WASHER \\
    billiam = hardwareMap.dcMotor.get("billiam");
// GLYPH FLIPPER \\
    hamilton = hardwareMap.servo.get("hamilton");
// LIFT \\
    evangelino = hardwareMap.dcMotor.get("evangelino");
    wilbert = hardwareMap.dcMotor.get("wilbert");
    donneet = hardwareMap.servo.get("donneet");
// HAMMER \\
    eddie = hardwareMap.servo.get("eddie");
    clark = hardwareMap.servo.get("clark");
// RELIC \\
    //georgery = hardwareMap.dcMotor.get("georgery");
    //brandy = hardwareMap.servo.get("brandy");
    //franny = hardwareMap.servo.get("franny");
    //mobert = hardwareMap.servo.get("mobert");
// VARIABLES \\
    motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
    motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
    elbowPos = 0.00; }


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

    motorFrontRight.setPower(v1);
    motorFrontLeft.setPower(v2);
    motorBackRight.setPower(v3);
    motorBackLeft.setPower(v4);

// RELIC //
//    if (gamepad1.dpad_up) { // Extension
//        georgery.setPower(0.75); }
//    else if (gamepad1.dpad_down) {
//        georgery.setPower(-0.75); }
//    else {
//        georgery.setPower(0.0); }
//
//    if (gamepad1.right_bumper) { // Grabbing
//        elbowPos += .01;
//        brandy.setPosition(elbowPos); }
//
//    else if (gamepad1.left_bumper) {
//        elbowPos -= .01;
//        brandy.setPosition(elbowPos); }
//
//    if (gamepad1.right_trigger > .7) { // Grabbing
//        franny.setPosition(0.00);
//        mobert.setPosition(0.00); }
//    else if (gamepad1.left_trigger > .7) {
//        franny.setPosition(1.00);
//        mobert.setPosition(1.00); }
//
//    if (gamepad2.dpad_up) {
//            brandy.setPosition(0.3); } // Elbow
//
//    if (gamepad2.dpad_down) {
//            brandy.setPosition(0.9); }
//
//    if (gamepad2.dpad_left) {
//            franny.setPosition(0.5); }
//
//    if (gamepad2.dpad_right) {
//            franny.setPosition(1); }


    ///////////////
    // GAMEPAD 2 //
    ///////////////

// FOUR BAR WITH ENCODERS //
    if (gamepad2.a) {
        evangelino.setPower(-.90);
        wilbert.setPower(.90);

    } else if (gamepad2.y) {
        evangelino.setPower(.90);
        wilbert.setPower(-.90);

    } else {
        evangelino.setPower(0);
        wilbert.setPower(0);
    }

// GATE //
    if (gamepad2.x) {
        donneet.setPosition(1);
    } else if (gamepad2.b) {
        donneet.setPosition(.5);
    }

// CAR WASHER //
    if (gamepad2.right_bumper) {
        billiam.setPower(.75);
    } else if (gamepad2.right_trigger > .7) {
        billiam.setPower(-.75);
    } else {
        billiam.setPower(0.00);
    }

// GLYPH FLIPPER //
    if (gamepad2.left_trigger > .7) {
        hamilton.setPosition(0.3);
    } else if (gamepad2.left_bumper) {
        hamilton.setPosition(1);
    } else if (gamepad2.dpad_up) {
        hamilton.setPosition(0.4); }
    }
}