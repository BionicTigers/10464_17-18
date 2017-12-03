package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Output", group="Protobot")
@Disabled
public class Output extends OpMode    {

    ElapsedTime runtime = new ElapsedTime();
//hello
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
    private ColorSensor sensorColor;

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
        left = 0.0;
        right = 1.0;
        runtime.reset();
        BNO055IMU imu;
        sensorColor = hardwareMap.get(ColorSensor.class, "sensorColor");
    }

    public void loop()
    {
        telemetry.addData("Motor degrees1", motorBackLeft.getCurrentPosition());
        telemetry.addData("Color value blue", sensorColor.blue());
        telemetry.addData("Motor degrees2", motorBackRight.getCurrentPosition());
        telemetry.addData("Motor degrees3", motorFrontLeft.getCurrentPosition());
        telemetry.addData("Motor degrees4", motorFrontRight.getCurrentPosition());

        //telemetry.addData(imu.getAngularOrientation().toAngleUnit());
    }
}



