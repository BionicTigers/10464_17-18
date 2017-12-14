
//            case 2: // moving robot to correct position in safe zone
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                heading = angles.firstAngle;
//
//                if (vuMark == RelicRecoveryVuMark.RIGHT) {
//                    map.setGoal(11, 5.4);
//                    moveState = MoveState.STRAFE_TOWARDS_GOAL;
//                    if (map.distanceToGoal() < +.1) {
//                        moveState = MoveState.STOP;
//                        gameState = 9;
//                    }
//                }
//                if (vuMark == RelicRecoveryVuMark.CENTER) {
//                    map.setGoal(11, 5);
//                    moveState = MoveState.STRAFE_TOWARDS_GOAL;
//                    if (map.distanceToGoal() < +.1) {
//                        moveState = MoveState.STOP;
//                        gameState = 9;
//                    }
//                }
//                if (vuMark == RelicRecoveryVuMark.LEFT) {
//                    map.setGoal(11, 4.6);
//                    moveState = MoveState.STRAFE_TOWARDS_GOAL;
//                    if (map.distanceToGoal() < +.1) {
//                        moveState = MoveState.STOP;
//                    }
//                }
//
//                break;
//        }
//    }
//}