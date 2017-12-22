package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;


@Autonomous(name="Red Front", group="Red")


public class RedFront extends OpMode {

    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;
    public DcMotor top;
    public DcMotor front;
    public int gameState;
    public ColorSensor sensorColor;
    public double waitTime;
    public Servo servo;
    public Servo mobert;
    public Servo franny;
    public VuforiaTrackable relicTemplate;
    public RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
    public int moveState;





    public void init() {

        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        front = hardwareMap.dcMotor.get("front");
        top = hardwareMap.dcMotor.get("top");
        servo = hardwareMap.servo.get("servo");
        gameState = 0;
        moveState = 0;


        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

    }


    public void loop() {
         //super.gameState();


        switch (gameState) {
            case 0: //preset variables
                waitTime = getRuntime(); //get current runTime
                gameState = 1;
                servo.setPosition(.9);
                franny.setPosition(.35);
                mobert.setPosition(.32);

                break;

            case 1://delay to allow servo to drop
                if (getRuntime() > waitTime + 2.0) {
                    gameState = 2;
                }
                break;
            case 2: //detect color sensor and choose direction
                waitTime = getRuntime(); //get current runTime

                if (sensorColor.blue() < 1) { //red
                    motorFrontLeft.setPower(-.5);
                    motorBackRight.setPower(-.55);
                    gameState = 3;

                } else {
                    motorFrontLeft.setPower(.55);
                    motorBackRight.setPower(.5);
                    gameState = 3;
                }

                break;

            case 3://delay to allow turn
                if (getRuntime() > waitTime + 2.0) {
                    gameState = 4;
                }
                break;
            case 4: //stop all motors, pull servo up
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                motorFrontRight.setPower(0);
                servo.setPosition(0.52);

                if(vuMark != RelicRecoveryVuMark.UNKNOWN){
                    telemetry.addData("VuMark", vuMark);
                    gameState = 6;
                }
                else{
                    telemetry.addData("Vumark is not seen", 0);
                }



                break;


//            case 5:
//
//                map.setGoal(11, 5);
//                //moveState =
//                motorFrontRight.setPower(0.25);
//                motorFrontLeft.setPower(-0.25);
//                motorBackRight.setPower(0.25);
//                motorBackLeft.setPower(-0.25);
//                gameState = 6;
//                break;


            case 6:
                    if(vuMark == RelicRecoveryVuMark.LEFT) {

                        Map.setGoal(11, 5.4);
                        moveState = AutonomousBaseMercury.MoveState.RIGHT;
                        gameState = 7;
                    }
                    else if(vuMark == RelicRecoveryVuMark.CENTER) {
                        Map.setGoal(11, 5);
                        moveState = AutonomousBaseMercury.MoveState.RIGHT;
                        gameState = 7;
                    }
                    else if(vuMark == RelicRecoveryVuMark.RIGHT) {
                        Map.setGoal(11, 4.6);
                        moveState = AutonomousBaseMercury.MoveState.RIGHT;
                        gameState = 7;
                    }
                    else {
                        Map.setGoal(11, 5);
                        moveState = AutonomousBaseMercury.MoveState.RIGHT;
                        gameState = 7;
                    }


                break;

            case 7:
                while (waitTime < 25) {
                    top.setPower(.5);
                    front.setPower(.5);

                    break;
                }
                break;
        }
    }
}





