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


/**
 * Autonomous Opmode G2
 */

@Autonomous(name="Auto_Red_Grab_Block", group="Wired")
public class Auto_Red_Grab_Block extends AutoLinearAbstract {

    // Declare OpMode members specific to this Autonomous Opmode variant.


    @Override
    public void runOpMode() {



        // Execute the typical autonomous program elements.
        // super.runOpMode finishes as soon as the Drive Station start/play button is pressed.
        super.runOpMode();


        driveTrain.goStraightToTarget(24, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Moving to line up with a block");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        scissorLift.goToAbsoluteDistance(SCISSOR_LIFT_PICK_POS,.5);

        generalTimer.reset();
        while (generalTimer.seconds() < 1) {
            if (Kill(28)) {
                break;
            }
        }

        boxMover.goToAbsoluteDistance(BOX_MOVER_PICK_POS_OUT,.5);

        while (!scissorLift.isMoveDone(SCISSOR_LIFT_POSITION_ERROR)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }


        while (!boxMover.isMoveDone(BOX_MOVER_POSITION_ERROR)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        scissorLift.goToRelativeDistance(100,.5);
        while (!scissorLift.isMoveDone(SCISSOR_LIFT_POSITION_ERROR)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }
        boxGrabber.goToPosition(BOX_GRABBER_CLOSED,.003);

        generalTimer.reset();
        while (generalTimer.seconds() < 2) {
            if (Kill(28)) {
                break;
            }
        }

        driveTrain.goStraightToTarget(-10, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Moving to line up with a block");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        driveTrain.StrafeLeftToTarget(72, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Strafing Left to foundation");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        scissorLift.goToAbsoluteDistance(4000,.5);

        while (!scissorLift.isMoveDone(SCISSOR_LIFT_POSITION_ERROR)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        driveTrain.goStraightToTarget(18, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Moving to line up with a block");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        boxGrabber.goToPosition(BOX_GRABBER_OPEN,.003);

        driveTrain.goStraightToTarget(-10, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Moving to line up with a block");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        driveTrain.StrafeRightToTarget(30, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Strafing Left to foundation");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

    }
}




