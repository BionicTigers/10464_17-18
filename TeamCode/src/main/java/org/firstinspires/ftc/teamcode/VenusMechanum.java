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
    //CONVEYOR BELT\\
    public Servo franny = null; //Left
    public Servo mobert = null; //Right
    //LIFT\\
    public DcMotor evangelino; //Left
    public DcMotor wilbert; //Right
    //HAMMER\\
    public Servo eddie = null; //Flicker
    public Servo clark = null; //Dropper
    //RELIC\\
    public DcMotor georgery = null;
    //VARIABLES\\
    public double beltRotation;


public void init() {

    //DRIVETRAIN\\
    motorFrontRight = hardwareMap.dcMotor.get("frontRight");
    motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
    motorBackRight = hardwareMap.dcMotor.get("backRight");
    motorBackLeft = hardwareMap.dcMotor.get("backLeft");
    //CAR WASHER\\
    billiam = hardwareMap.dcMotor.get("billiam");
    //CONVEYOR BELT\\
    franny = hardwareMap.servo.get("franny");
    mobert = hardwareMap.servo.get("mobert");
    //LIFT\\
    evangelino = hardwareMap.dcMotor.get("evangelino");
    wilbert = hardwareMap.dcMotor.get("wilbert");

    evangelino.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    evangelino.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
    wilbert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    wilbert.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
    //HAMMER\\
    eddie = hardwareMap.servo.get("eddie");
    clark = hardwareMap.servo.get("clark");
    //RELIC\\
    georgery = hardwareMap.dcMotor.get("georgery");
    //VARIABLES\\
    beltRotation = 0.0; }


public void loop() {

    ///////////////
    // GAMEPAD 1 //
    ///////////////
    double P = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
    double robotAngle = Math.atan2(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
    double rightX = -gamepad1.right_stick_x;
    double sinRAngle = Math.sin(robotAngle);
    double cosRAngle = Math.cos(robotAngle);

    final double v1 = (P * sinRAngle) - (P * cosRAngle) - rightX;
    final double v2 = (P * sinRAngle) + (P * cosRAngle) + rightX;
    final double v3 = (P * sinRAngle) + (P * cosRAngle) - rightX;
    final double v4 = (P * sinRAngle) - (P * cosRAngle) + rightX;

    motorFrontRight.setPower(v1);
    motorFrontLeft.setPower(v2);
    motorBackRight.setPower(v3);
    motorBackLeft.setPower(v4);

    if (gamepad1.dpad_up) {
        eddie.setPosition(0.5);
        clark.setPosition(0.6); }

    ///////////////
    // GAMEPAD 2 //
    ///////////////
    if (gamepad2.right_bumper) {
        billiam.setPower(0.75); }
    else if (gamepad2.left_bumper) {
        billiam.setPower(-0.75); }
    else {
        billiam.setPower(0.00); }

    if (gamepad2.right_trigger > .7) {
        beltRotation += .01;
        franny.setPosition(beltRotation);
        mobert.setPosition(beltRotation); }
    else if (gamepad2.left_trigger > .7) {
        beltRotation -= .01;
        franny.setPosition(beltRotation);
        mobert.setPosition(beltRotation); }
    else {
        beltRotation += 0;
        franny.setPosition(beltRotation);
        mobert.setPosition(beltRotation); }

    if (gamepad2.a) {
        evangelino.setTargetPosition(0);
        wilbert.setTargetPosition(0);
        evangelino.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wilbert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        evangelino.setPower(0.75);
        wilbert.setPower(0.75); }

    if (gamepad2.x) {
        evangelino.setTargetPosition(22);
        wilbert.setTargetPosition(22);
        evangelino.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wilbert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        evangelino.setPower(0.75);
        wilbert.setPower(0.75); }

    if (gamepad2.b) {
        evangelino.setTargetPosition(44);
        wilbert.setTargetPosition(44);
        evangelino.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wilbert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        evangelino.setPower(0.75);
        wilbert.setPower(0.75); }

    if (gamepad2.y) {
        evangelino.setTargetPosition(66);
        wilbert.setTargetPosition(66);
        evangelino.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wilbert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        evangelino.setPower(0.75);
        wilbert.setPower(0.75); } } }