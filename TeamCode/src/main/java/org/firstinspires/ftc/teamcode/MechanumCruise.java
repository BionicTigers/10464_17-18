//package org.firstinspires.ftc.teamcode;
//
///**
// * Created by emilydkiehl on 2/13/2018.
// */
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//
//
//@Autonomous(name="Cruise", group ="Red")
//
//public class MechanumCruise extends LinearOpMode {
//
//    OpenGLMatrix lastLocation = null;
//    VuforiaLocalizer vuforia;
//    public DcMotor motorFrontLeft;
//    public DcMotor motorBackRight;
//    public DcMotor motorFrontRight;
//    public DcMotor motorBackLeft;
//    public DcMotor wilbert; //Four Bar Right
//
//    //public Servo burr; //Glyph Flipper 2
//    public ElapsedTime runtime = new ElapsedTime();
//    int i;
//
//
//    @Override
//    public void runOpMode() {
//        i = 0;
//
//
//        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
//        motorBackRight = hardwareMap.dcMotor.get("backRight");
//        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
//        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
//
//        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
//        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
//
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Status", "Resetting Encoders");    //
//        telemetry.update();
//
//        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        time = getRuntime();
//
//                motorFrontLeft.getCurrentPosition(),
//                motorBackRight.getCurrentPosition(),
//                motorFrontRight.getCurrentPosition(),
//                motorBackLeft.getCurrentPosition());
//
//
//        telemetry.update();
//
//        waitForStart(){
//
//        }

