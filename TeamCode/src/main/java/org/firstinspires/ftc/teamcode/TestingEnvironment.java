package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Testing Environment", group="Venus")


public class TestingEnvironment extends OpMode {

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
    public DcMotor evangelino; //Left
    public DcMotor wilbert; //Right
    //VARIABLES\\
    private int targetPos;


    public void init() {

        ///DRIVETRAIN\\
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        //CAR WASHER\\
        billiam = hardwareMap.dcMotor.get("billiam");
        //GLYPH FLIPPER\\
        hamilton = hardwareMap.servo.get("hamilton");
        //LIFT\\
        evangelino = hardwareMap.dcMotor.get("evangelino");
        wilbert = hardwareMap.dcMotor.get("wilbert");
        //VARIABLES//
        targetPos = 0;
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        evangelino.setDirection(DcMotor.Direction.REVERSE);
        evangelino.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        evangelino.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wilbert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wilbert.setMode(DcMotor.RunMode.RUN_TO_POSITION); }


    public void loop() {
        telemetry.addData("evangelino", evangelino.getCurrentPosition());
        telemetry.addData("wilbert",    wilbert.getCurrentPosition());
        telemetry.addData("targetPos",  targetPos);
        telemetry.update();

        ///////////////
        // GAMEPAD 1 //
        ///////////////

        //MAIN DRIVE//
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

        ///////////////
        // GAMEPAD 2 //
        ///////////////

        //FOUR BAR WITH ENCODERS//
        if (gamepad2.a) {
            evangelino.setTargetPosition(-18);
            evangelino.setPower(.90);
            wilbert.setTargetPosition(-18);
            wilbert.setPower(.90); }

        if (gamepad2.y) {
            evangelino.setTargetPosition(-5);
            evangelino.setPower(.50);
            wilbert.setTargetPosition(-5);
            wilbert.setPower(.50); }

        if (gamepad2.right_bumper) {
            billiam.setPower(-1); }
        else if (gamepad2.right_trigger > .7) {
            billiam.setPower(1); }
        else {
            billiam.setPower(0.00); }

        if (gamepad2.left_trigger > .7) {
            hamilton.setPosition(0.6);
        } else if (gamepad2.left_bumper) {
            hamilton.setPosition(1); } } }