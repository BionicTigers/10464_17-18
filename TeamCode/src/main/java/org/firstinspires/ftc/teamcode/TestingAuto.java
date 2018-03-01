package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by emilydkiehl on 2/21/18
 */
@Autonomous(name="TestingAuto")
public class TestingAuto extends Map {

    public TestingAuto() {
        super(10, 2);
    }

    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {

            Map.setRobot(10, 2);
            telemetry.addData("setRobot", "robot is set");

            Map.setGoal(11, 5);
            telemetry.addData("setGoal", "goal is set");

            Map.updateStrafeDist();

            Map.driveToGoal();
        }

    }

}





