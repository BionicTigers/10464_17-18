package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by emilydkiehl on 2/21/18
 */
@Autonomous(name="TestingAuto")
public class TestingAuto extends Map {

    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {

            setRobot(10, 2);
            telemetry.addData("setRobot", "robot is set");

            setGoal(11, 5);
            telemetry.addData("setGoal", "goal is set");

            updateStrafeDist();

            driveToGoal();
        }

    }

}





