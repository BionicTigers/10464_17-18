package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;


@TeleOp(name="Tutorial", group="ThatOneGroup")


public class Tutorial extends OpMode {

    public DcMotor motorFrontRight;

    public void init() {

        motorFrontRight = hardwareMap.dcMotor.get("frontRight");

    }


    public void loop() {

        motorFrontRight.setPower(-gamepad1.right_stick_y);

    }
}


//public DcMotor motorFrontRight;
//motorFrontRight = hardwareMap.dcMotor.get("frontRight");
//motorFrontRight.setPower(-gamepad1.right_stick_y);