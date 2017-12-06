package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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

@TeleOp(name="OrientedMechanum", group="Protobot")

public class OrientedMechanum extends OpMode {

    public Orientation angles;
    public Acceleration gravity;
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;
    public DcMotor top;
    public DcMotor front;
    public Servo franny = null; //left servo
    public Servo mobert = null; //right servo
    public double left;
    public double right;
    public int calibToggle;
    BNO055IMU imu;

    public void init() {
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
        calibToggle = 0;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void loop() {

        telemetry.update();


        /////////////////////////////
        // ORIENTATION CALIBRATION //
        /////////////////////////////

        if (gamepad1.a) {
            // Get the calibration data
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu.initialize(parameters);

            BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
            String filename = "BNO055IMUCalibration.json";
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, calibrationData.serialize());
            telemetry.log().add("saved to '%s'", filename);
        }

            //telemetry.addData("imu gyro calib status", imu.getCalibrationStatus());


            if (gamepad1.x) {
                calibToggle += 1;
            }


            if ((calibToggle & 1) != 0) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double H = (angles.firstAngle * Math.PI) / 180;
                double rightX = gamepad1.right_stick_x;
                double rightY = gamepad1.right_stick_y;
                double turn = gamepad1.left_stick_x;

                motorFrontRight.setPower((rightY * Math.cos(H) + rightX * Math.sin(H)) + (rightY * Math.sin(H) + rightX * Math.cos(H)) + turn);
                motorFrontLeft.setPower((rightY * Math.cos(H) + rightX * Math.sin(H)) - (rightY * Math.sin(H) + rightX * Math.cos(H)) + turn);
                motorBackRight.setPower((rightY * Math.cos(H) + rightX * Math.sin(H)) - (rightY * Math.sin(H) + rightX * Math.cos(H)) - turn);
                motorBackLeft.setPower((rightY * Math.cos(H) + rightX * Math.sin(H)) + (rightY * Math.sin(H) + rightX * Math.cos(H)) - turn);
            }
            else {

                double P = Math.hypot(-gamepad1.right_stick_x, -gamepad1.left_stick_y);
                double robotAngle = Math.atan2(-gamepad1.right_stick_x, -gamepad1.left_stick_y) - Math.PI / 4;
                double rightX = gamepad1.left_stick_x;

                final double v1 = P * Math.sin(robotAngle) + rightX;
                final double v2 = P * Math.cos(robotAngle) + rightX;
                final double v3 = P * Math.cos(robotAngle) - rightX;
                final double v4 = P * Math.sin(robotAngle) - rightX;

                motorFrontRight.setPower(v1);
                motorFrontLeft.setPower(v2);
                motorBackRight.setPower(v3);
                motorBackLeft.setPower(v4);
            }

            if (gamepad2.left_bumper) {
                if (left < 0.35) {
                    left += .01;
                }
                franny.setPosition(left);
            } else if (gamepad2.left_trigger > .7) {
                if (left > 0.0) {
                    left -= .01;
                }
                franny.setPosition(left);
            }

            if (gamepad2.right_bumper) {
                if (right > 0.32) {
                    right -= .01;
                }
                mobert.setPosition(right);
            } else if (gamepad2.right_trigger > .7) {
                if (right < 1) {
                    right += .01;
                }
                mobert.setPosition(right);
            }

            telemetry.addData("Left", left);
            telemetry.addData("Right", right);
            telemetry.addData("franny", franny);
            telemetry.addData("mobert", mobert);

            ///////////////////
            // BELT CONTROLS //
            ///////////////////

            if (gamepad2.dpad_up) {
                top.setPower(-0.45);
            } else if (gamepad2.dpad_down) {
                top.setPower(0.45);
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





    private String composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
        return formatAngle(angles.angleUnit, angles.firstAngle);
    }





    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
