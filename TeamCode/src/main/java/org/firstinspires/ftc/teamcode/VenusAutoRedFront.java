package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Sean O on 11/23/2016.
 */
@Autonomous(name="Red Front", group="Blue")
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

            case 777:
                moveState = MoveState.FULL_STOP;
                break;
        }
    }
}




