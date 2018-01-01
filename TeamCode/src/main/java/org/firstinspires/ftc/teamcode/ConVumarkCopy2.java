package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="Vuforia", group ="Concept")

public abstract class ConVumarkCopy2 extends OpMode {

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    int moveState = 0;
    DcMotor motorBackRight;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorFrontLeft;
    BNO055IMU imu;


    VuforiaTrackable relicTemplate;
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");

    public void init() {

        parameters.vuforiaLicenseKey = "AfBkGLH/////AAAAGUUS7r9Ue00upoglw/0yqTBLwhqYHpjwUK9zxmWMMFGuNGPjo/RjNOTsS8POmdQLHwe3/75saYsyb+mxz6p4O8xFwDT7FEYMmKW2NKaLKCA2078PZgJjnyw+34GV8IBUvi2SOre0m/g0X5eajpAhJ8ZFYNIMbUfavjQX3O7P0UHyXsC3MKxfjMzIqG1AgfRevcR/ONOJlONZw7YIZU3STjODyuPWupm2p7DtSY4TRX5opqFjGQVKWa2IlNoszsN0szgW/xJ1Oz5VZp4oDRS8efG0jOq1QlGw7IJOs4XXZMcsk0RW/70fVeBiT+LMzM8Ih/BUxtVVK4pcLMpb2wlzdKVLkSD8LOpaFWmgOhxtNz2M";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        imu = (BNO055IMU) hardwareMap.get("imu");
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //relicTrackables.activate();
    }
    public void gameState() {

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        Map.setRobot(10, 2);
        //if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
        //telemetry.addData("VuMark", "%s visible", vuMark);
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
        //telemetry.addData("Pose",format(pose));

        // both left motors need to be reversed. Here that is done manually
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            Map.setGoal(11, 5.4);

            int frontRight = motorFrontRight.getCurrentPosition();
            telemetry.addData("frontright", frontRight);

            int backLeft = motorBackLeft.getCurrentPosition();
            telemetry.addData("backleft", backLeft);

            moveState = AutonomousBaseMercury.MoveState.STRAFE_TOWARDS_GOAL;

        }
        else if (vuMark == RelicRecoveryVuMark.CENTER) {
            Map.setGoal(11, 5);

            int frontRight = motorFrontRight.getCurrentPosition();
            telemetry.addData("frontright", frontRight);

            int backLeft = motorBackLeft.getCurrentPosition();
            telemetry.addData("backleft", backLeft);
            moveState = AutonomousBaseMercury.MoveState.STRAFE_TOWARDS_GOAL;


        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {

            Map.setGoal(11,4.6);

            int frontRight = motorFrontRight.getCurrentPosition();
            telemetry.addData("frontright", frontRight);

            int backLeft = motorBackLeft.getCurrentPosition();
            telemetry.addData("backleft", backLeft);

            moveState = AutonomousBaseMercury.MoveState.STRAFE_TOWARDS_GOAL;


        } else {
            Map.setGoal(11, 5);
            int frontRight = motorFrontRight.getCurrentPosition();
            telemetry.addData("frontright", frontRight);

            int backLeft = motorBackLeft.getCurrentPosition();
            telemetry.addData("backleft", backLeft);
            moveState = AutonomousBaseMercury.MoveState.STRAFE_TOWARDS_GOAL;
        }

        if (pose != null) {
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            double tX = trans.get(0);
            double tY = trans.get(1);
            double tZ = trans.get(2);

            // Extract the rotational components of the target relative to the robot
            double rX = rot.firstAngle;
            double rY = rot.secondAngle;
            double rZ = rot.thirdAngle;

            telemetry.addData("pos", imu.getPosition());
        }
        else {
            telemetry.addData("VuMark", "not visible");
        }

        telemetry.update();
    }
            String format(OpenGLMatrix transformationMatrix) {
                return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
