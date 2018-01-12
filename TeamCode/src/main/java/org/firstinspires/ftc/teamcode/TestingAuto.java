package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="TESTINGAUTO", group="Red")
public class TestingAuto extends OpMode{
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    int state = 0, leftPos = 0, rightPos = 0;
    public void init()
    {
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
    }

    public void loop() //Short Autonomous
    {
        telemetry.addData("leftMotor", motorFrontLeft.getCurrentPosition());
        telemetry.addData("leftPos", leftPos);
        telemetry.addData("rightPos", rightPos);
        switch (state)
        {
            //initialize values to run
            case 0:
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                motorBackLeft.setPower(0);
                leftPos = motorFrontLeft.getCurrentPosition();
                rightPos = motorFrontRight.getCurrentPosition();
                state = 1;
                break;
            // Drive forward stop motors when target reached and move to next step
            case 1:
                if (motorFrontLeft.getCurrentPosition() > leftPos - 75) {
                    motorFrontLeft.getCurrentPosition();
                    motorBackLeft.getCurrentPosition();
                    motorFrontLeft.setPower(.35);
                    motorFrontRight.setPower(.35);
                    motorBackLeft.setPower(.35);
                    motorBackRight.setPower(.35);
                    telemetry.update();

                }
                else
                {
                    motorFrontLeft.setPower(0);
                    motorFrontRight.setPower(0);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                    leftPos = motorFrontLeft.getCurrentPosition();
                    state = 2;
                    telemetry.update();
                }
                break;
            case 2:
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                motorBackLeft.setPower(0);
                break;
        }
    }
}
