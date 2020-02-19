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


@Autonomous(name="Auto_Blue_Block_Grab_Foundation", group="Wired")
public class Auto_Blue_Block_Grab_Foundation extends AutoLinearAbstract {

    // Declare OpMode members specific to this Autonomous Opmode variant.


    @Override
    public void runOpMode() {



        // Execute the typical autonomous program elements.
        // super.runOpMode finishes as soon as the Drive Station start/play button is pressed.
        super.runOpMode();


        // Go straight to become paralell with blocks
        driveTrain.StrafeRightToTarget(29, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Strafing right to line up with foundation");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }


        driveTrain.goStraightToTarget(89, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Moving to foundation");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }


        driveTrain.StrafeRightToTarget(6, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Strafing right and knocking into foundation");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        clawServoLeft.goToPosition(CLAW_SERVO_LEFT_DOWN,.02);
        clawServoRight.goToPosition(CLAW_SERVO_RIGHT_DOWN, .02);

        generalTimer.reset();
        while (generalTimer.seconds() < 2) {
            if (Kill(28)) {
                break;
            }
        }

        driveTrain.StrafeLeftToTarget(45, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Strafing left to park foundation");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }


        clawServoLeft.goToPosition(CLAW_SERVO_LEFT_UP,.02);
        clawServoRight.goToPosition(CLAW_SERVO_RIGHT_UP,.02);


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



