package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="Venus Mechanum", group="Venus")


public class VenusMechanum extends OpMode {

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
//    public DcMotor evangelino; //Left
//    public DcMotor wilbert; //Right
    //HAMMER\\
    //public Servo eddie = null; //Flicker
    //public Servo clark = null; //Dropper
    //RELIC\\
//    public DcMotor georgery; //Extender
//    public Servo brandy = null; //Elbow
//    public Servo franny = null; //Left Finger
//    public Servo mobert = null; //Right Finger
    //VARIABLES\\
    public double elbowPos;


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
    //CAR WASHER\\
    billiam = hardwareMap.dcMotor.get("billiam");
    //GLYPH FLIPPER\\
    hamilton = hardwareMap.servo.get("hamilton");
    //LIFT\\
//    evangelino = hardwareMap.dcMotor.get("evangelino");
//    wilbert = hardwareMap.dcMotor.get("wilbert");
//
//    evangelino.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    evangelino.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
//    wilbert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    wilbert.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
    //HAMMER\\
    //eddie = hardwareMap.servo.get("eddie");
    //clark = hardwareMap.servo.get("clark");
    //RELIC\\
//    georgery = hardwareMap.dcMotor.get("georgery");
//    brandy = hardwareMap.servo.get("brandy");
//    franny = hardwareMap.servo.get("franny");
//    mobert = hardwareMap.servo.get("mobert");
    //VARIABLES\\
    elbowPos = 0.00;

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

    if (gamepad1.y) {
        //eddie.setPosition(0.5);
        //clark.setPosition(0.6); }

//    if (gamepad1.dpad_up) {
//        georgery.setPower(0.75); }
//    else if (gamepad1.dpad_down) {
//        georgery.setPower(-0.75); }
//    else {
//        georgery.setPower(0.0); }
//
//    if (gamepad1.right_bumper) {
//            elbowPos += .01;
//        brandy.setPosition(elbowPos); }
//
//    else if (gamepad1.left_bumper) {
//            elbowPos -= .01;
//        brandy.setPosition(elbowPos); }
//
//    if (gamepad1.right_trigger > .7) {
//        franny.setPosition(0.00);
//        mobert.setPosition(0.00); }
//    else if (gamepad1.left_trigger > .7) {
//        franny.setPosition(1.00);
//        mobert.setPosition(1.00); }

    ///////////////
    // GAMEPAD 2 //
    ///////////////
    if (gamepad2.right_bumper) {
        billiam.setPower(0.75); }
    else if (gamepad2.right_trigger >.7) {
        billiam.setPower(-0.75); }
    else {
        billiam.setPower(0.00); }

    if (gamepad2.left_trigger > .7) {
        hamilton.setPosition(0.6); }
    else if (gamepad2.left_bumper) {
        hamilton.setPosition(0.0); } } } }


//    if (gamepad2.a) {
//        evangelino.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        wilbert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        evangelino.setTargetPosition(0);
//        wilbert.setTargetPosition(0);
//        evangelino.setPower(0.75);
//        wilbert.setPower(0.75); }
//
//    if (gamepad2.x) {
//        evangelino.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        wilbert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        evangelino.setTargetPosition(22);
//        wilbert.setTargetPosition(22);
//        evangelino.setPower(0.75);
//        wilbert.setPower(0.75); }
//
//    if (gamepad2.b) {
//        evangelino.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        wilbert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        evangelino.setTargetPosition(22);
//        wilbert.setTargetPosition(22);
//        evangelino.setPower(0.75);
//        wilbert.setPower(0.75); }
//
//    if (gamepad2.y) {
//        evangelino.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        wilbert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        evangelino.setTargetPosition(66);
//        wilbert.setTargetPosition(66);
//        evangelino.setPower(0.75);
//        wilbert.setPower(0.75); } } }

