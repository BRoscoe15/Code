/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


// Autonomous Opmode G2


@Autonomous(name="Auto_BLue_Block_Move", group="Wired")
public class Auto_BLue_Block_Move extends AutoLinearAbstract {

    // Declare OpMode members specific to this Autonomous Opmode variant.


    @Override
    public void runOpMode() {



        // Execute the typical autonomous program elements.
        // super.runOpMode finishes as soon as the Drive Station start/play button is pressed.
        super.runOpMode();


        // Go straight to become paralell with blocks
        driveTrain.StrafeRightToTarget(20, DRIVE_TRAIN_DEFAULT_SPEED);
        scissorLift.goToAbsoluteDistance(SCISSOR_LIFT_POS, .3);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Strafing right to find skystone");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        driveTrain.resetEncoders();



//vision



        driveTrain.goStraightToTarget(15.5, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Moving forward out of the way");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }


        while (!scissorLift.isMoveDone(SCISSOR_LIFT_POSITION_ERROR)) {
            telemetry.addLine("Wait - Making sure the scissor lift is out");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        driveTrain.StrafeRightToTarget(12, DRIVE_TRAIN_DEFAULT_SPEED);
        boxMover.goToAbsoluteDistance(BOX_MOVER_PICK_POS_OUT, .3);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Strafing right while moving the box mover out");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }


        while (!boxMover.isMoveDone(BOX_MOVER_POSITION_ERROR)) {
            telemetry.addLine("Wait - Making sure the Box mover is out all the way");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        scissorLift.goToAbsoluteDistance(SCISSOR_LIFT_PICK_POS, .3);
        while (!scissorLift.isMoveDone(SCISSOR_LIFT_POSITION_ERROR)) {
            telemetry.addLine("Wait - Lowering lift to pick");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        telemetry.addLine("Wait - Closing Gripper");
        driveTrainTelemetry();
        telemetry.update();
        boxGrabber.goToPosition(BOX_GRABBER_CLOSED, BOX_GRABBER_INC);


        scissorLift.goToAbsoluteDistance(SCISSOR_LIFT_CARRY_HEIGHT,.3);
        while (!scissorLift.isMoveDone(SCISSOR_LIFT_POSITION_ERROR)) {
            telemetry.addLine("Wait - Raising scissor lift to carrying height");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }


        driveTrain.StrafeLeftToTarget(28,DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Strafing right while moving the box mover out");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }


        driveTrain.goStraightToTarget(70,DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Moving forward to the foundation");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        scissorLift.goToAbsoluteDistance(1500,.3);
        driveTrain.StrafeRightToTarget(35,DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Strafing into the foundation");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        while (!scissorLift.isMoveDone(SCISSOR_LIFT_POSITION_ERROR)) {
            telemetry.addLine("Wait - Raising scissor lift to carrying height");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        boxGrabber.goToPositionNow(BOX_GRABBER_OPEN);
        clawServoLeft.goToPositionNow(CLAW_SERVO_LEFT_DOWN);
        clawServoRight.goToPosition(CLAW_SERVO_RIGHT_DOWN, .02);

        boxMover.goToAbsoluteDistance(BOX_MOVER_IN,.3);
        driveTrain.StrafeLeftToTarget(45,DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Strafing into the wall with the foundation");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        telemetry.addLine("Wait - Waiting for claws to lift");
        driveTrainTelemetry();
        telemetry.update();
        clawServoLeft.goToPositionNow(CLAW_SERVO_LEFT_UP);
        clawServoRight.goToPosition(CLAW_SERVO_RIGHT_UP,.02);

        while (!boxMover.isMoveDone(BOX_MOVER_POSITION_ERROR)) {
            telemetry.addLine("Wait - Making sure the Box mover is out all the way");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }


        scissorLift.goToAbsoluteDistance(SCISSOR_DOWN_POS, .3);
        while (!scissorLift.isMoveDone(SCISSOR_LIFT_POSITION_ERROR)) {
            telemetry.addLine("Wait - Lowering scissor lift so it does not hit the bridge");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }


        driveTrain.goStraightToTarget(-48, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Moving to park area");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }




    }
}




