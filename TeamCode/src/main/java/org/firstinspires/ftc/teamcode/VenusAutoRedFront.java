package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Emily on 1/24/18
 */
@Autonomous(name="Red Front", group="Red")
public class VenusAutoRedFront extends AutonomousBaseVenus {

    public void gameState() {
        super.gameState();
        switch(gameState){
            case 0: //Start

                if(getRuntime() > 3) {
                    gameState = 1;
                    map.setRobot(10,2);
                }
                break;
            case 1: //moves to shooter post

                map.setGoal(7, 1);
                moveState = MoveState.STRAFE_TOWARDS_GOAL;
                if(map.distanceToGoal()<=.1){
                    moveState = MoveState.STOP;
                    gameState = 2;
                }
                break;

            case 2:
                moveState = MoveState.FULL_STOP;
                break;
        }
    }
}




