package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name="Oriented Mechanum", group="Protobot")

public abstract class OrientedMechanum extends OpMode {


    /* Declare OpMode members. */
    Orientation angles;
    Acceleration gravity;
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor top;
    DcMotor front;
    Servo franny = null; //left servo
    Servo mobert = null; //right servo
    double left;
    double right;
    BNO055IMU imu;
    double initState;
    String formatAngle;


    public class SensorBNO055IMU extends LinearOpMode {
        //----------------------------------------------------------------------------------------------
        // State
        //----------------------------------------------------------------------------------------------

        // The IMU sensor object
        BNO055IMU imu;

        // State used for updating telemetry
        Orientation angles;
        Acceleration gravity;

        //----------------------------------------------------------------------------------------------
        // Main logic
        //----------------------------------------------------------------------------------------------

        @Override
        public void runOpMode() {

            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            // Set up our telemetry dashboard
            composeTelemetry();

            // Wait until we're told to go
            waitForStart();

            // Start the logging of measured acceleration
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

            // Loop and update the dashboard
            while (opModeIsActive()) {
                telemetry.update();
            }
        }

        //----------------------------------------------------------------------------------------------
        // Telemetry Configuration
        //----------------------------------------------------------------------------------------------


        //----------------------------------------------------------------------------------------------
        // Formatting
        //----------------------------------------------------------------------------------------------

        String formatAngle(AngleUnit angleUnit, double angle) {
            return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
        }

        String formatDegrees(double degrees) {
            return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
        }
    }

    public void loop()
    {
        ////////////////
        // MAIN DRIVE //
        ////////////////
        double r = Math.hypot(-gamepad1.right_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.right_stick_x, -gamepad1.left_stick_y) - Math.PI / 4;
        double rightX = gamepad1.left_stick_x;
        final double v1 = r * Math.sin(robotAngle) + rightX;
        final double v2 = r * Math.cos(robotAngle) + rightX;
        final double v3 = r * Math.cos(robotAngle) - rightX;
        final double v4 = r * Math.sin(robotAngle) - rightX;

        motorFrontRight.setPower(v1);
        motorFrontLeft.setPower(v2);
        motorBackRight.setPower(v3);
        motorBackLeft.setPower(v4);

        /////////////////////////////
        // ORIENTATION CALIBRATION //
        /////////////////////////////


        if (gamepad1.right_trigger > .7)
        {
            composeTelemetry();
            telemetry.addData("angles", angles.firstAngle);
            telemetry.addData("imu gyro calib status", imu.getCalibrationStatus());
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double P = -((Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x) / 2));
            double H = (angles.firstAngle * Math.PI) / 180;
            double Ht = (Math.PI + Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y));
            motorBackRight.setPower(P * Math.sin(H - Ht));
            motorFrontLeft.setPower(P * Math.sin(H - Ht));
            motorBackLeft.setPower(P * Math.cos(H - Ht));
            motorFrontRight.setPower(P * Math.cos(H - Ht));

        }
        else {
            telemetry.addData("imu gyro calib status", imu.getCalibrationStatus());
            motorFrontRight.setPower(v1);
            motorFrontLeft.setPower(v2);
            motorBackRight.setPower(v3);
            motorBackLeft.setPower(v4);
        }



        //if (gamepad1.left_stick_button) {
        //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //telemetry.addData("heading", angles);

        //final double v5 = r * Math.sin(robotAngle) + rightX + angles.firstAngle;
        //final double v6 = r * Math.cos(robotAngle) + rightX + angles.firstAngle;
        //final double v7 = r * Math.cos(robotAngle) - rightX + angles.firstAngle;
        //final double v8 = r * Math.sin(robotAngle) - rightX + angles.firstAngle;

        //motorFrontRight.setPower(v5);
        //motorFrontLeft.setPower(v6);
        //motorBackRight.setPower(v7);
        //motorBackLeft.setPower(v8);

        //}

        ///////////////////////
        // COLLECTION SERVOS //
        ///////////////////////

        if (gamepad2.x) {
            if (left < 0.3 && right > 0.32) {
                left += .01;
                right -= .01;
            }
            franny.setPosition(left);
            mobert.setPosition(right);
        }
        else if (gamepad2.b) {
            if (left > 0.00 && right < 1.0) {
                left -= .01;
                right += .01;
            }
            franny.setPosition(left);
            mobert.setPosition(right);
        }

        if (gamepad2.left_bumper) {
            if (left < 0.3) {
                left += .01;
            }
            franny.setPosition(left);
        }
        else if (gamepad2.left_trigger > .7) {
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
        }
        else if (gamepad2.right_trigger > .7) {
            if (right < 1) {
                right += .01;
            }
            mobert.setPosition(right);
        }

        telemetry.addData("Left", left);
        telemetry.addData("Right", right);
        telemetry.addData("franny", franny);
        telemetry.addData("mobert", mobert);
        telemetry.addData("angles", angles.firstAngle);


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

    void composeTelemetry() {

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
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}