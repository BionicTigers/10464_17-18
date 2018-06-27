package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;


@Autonomous(name="Cruise", group ="Red")
@Disabled
public class MecanumCruise extends LinearOpMode {

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor wilbert; //Four Bar Right

    //public Servo burr; //Glyph Flipper 2
    public ElapsedTime runtime = new ElapsedTime();
    int i;

    public static double currentDesiredDistance = 0;

    @Override
    public void runOpMode() {
        i = 0;


        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setCruiseControl(int speed, int distance) { //Distance should be in full wheel rotations
        distance = distance * 560 * 2; //560 encoder ticks per revolution of Classic NeveRest 20s, on Mercury it is geared down a 2:1

        final double desiredDistancePerSecond = (((200 / 60) / 100) * speed); //200 is the reasonable RPM of a loaded NeveRest 20

        motorFrontLeft.setTargetPosition(distance);
        motorBackRight.setTargetPosition(distance);
        motorFrontRight.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(distance);

        motorFrontLeft.setPower(speed);
        motorBackRight.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackLeft.setPower(speed);

        Runnable cruiseCheck = new Runnable() {
            public void run() {
                currentDesiredDistance = currentDesiredDistance + desiredDistancePerSecond;

                if(motorFrontLeft.getCurrentPosition() < currentDesiredDistance) {
                    motorFrontLeft.setPower(motorFrontLeft.getPower() + ((currentDesiredDistance - motorFrontLeft.getCurrentPosition()) / 50));
                } else if(motorFrontLeft.getCurrentPosition() > currentDesiredDistance) {
                    motorFrontLeft.setPower(motorFrontLeft.getPower() - ((motorFrontLeft.getCurrentPosition() - currentDesiredDistance) / 50));
                }

                if(motorBackRight.getCurrentPosition() < currentDesiredDistance) {
                    motorBackRight.setPower(motorBackRight.getPower() + ((currentDesiredDistance - motorBackRight.getCurrentPosition()) / 50));
                } else if(motorBackRight.getCurrentPosition() > currentDesiredDistance) {
                    motorBackRight.setPower(motorBackRight.getPower() - ((motorBackRight.getCurrentPosition() - currentDesiredDistance) / 50));
                }

                if(motorFrontRight.getCurrentPosition() < currentDesiredDistance) {
                    motorFrontRight.setPower(motorFrontRight.getPower() + ((currentDesiredDistance - motorFrontRight.getCurrentPosition()) / 50));
                } else if(motorFrontRight.getCurrentPosition() > currentDesiredDistance) {
                    motorFrontRight.setPower(motorFrontRight.getPower() - ((motorFrontRight.getCurrentPosition() - currentDesiredDistance) / 50));
                }

                if(motorBackLeft.getCurrentPosition() < currentDesiredDistance) {
                    motorBackLeft.setPower(motorBackLeft.getPower() + ((currentDesiredDistance - motorBackLeft.getCurrentPosition()) / 50));
                } else if(motorBackLeft.getCurrentPosition() > currentDesiredDistance) {
                    motorBackLeft.setPower(motorBackLeft.getPower() - ((motorBackLeft.getCurrentPosition() - currentDesiredDistance) / 50));
                }
            }
        };

        while (motorFrontLeft.isBusy() && motorBackRight.isBusy() && motorBackLeft.isBusy() && motorFrontRight.isBusy()) {
            ScheduledExecutorService executor = Executors.newScheduledThreadPool(1);
            executor.scheduleAtFixedRate(cruiseCheck, 1, 1, TimeUnit.SECONDS);
        }

        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}