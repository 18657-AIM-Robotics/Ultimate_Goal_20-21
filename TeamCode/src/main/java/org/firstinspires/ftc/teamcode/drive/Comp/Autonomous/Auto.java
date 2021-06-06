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

package org.firstinspires.ftc.teamcode.drive.Comp.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Comp.HardwareBot.Hardware;



@Autonomous(name="CompetitionAuto", group="Pushbot")
public class Auto extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();   // Use a Pushbot's hardware

    private ElapsedTime runtime = new ElapsedTime();
    public static double X_TICKSTOINCHES = 1425;  // TODO: Find ticks to inches
    public static double Y_TICKSTOINCHES = 1725;  // TODO: Find ticks to inches

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            armUpDown(1, 1750);
            sleep(500);
            moveIn("y", -.6, 2);
            moveIn("x", .6, 83);
            sleep(500);
            moveIn("x", -.7, 20);
            sleep(500);
            moveIn("y", .6, 16.25);
            turn(3, .5);
            sleep(100);
            shootRing(10000);
            shootRing(10000);
            shootRing(10000);
            moveIn("y", .4, 48);
            turn(4, -.5);
            moveIn("x", -.7, 71);
            moveIn("y", -.4, 9.5);
            turn(32, -.5);
            moveIn("x", .7, 82);
//            moveIn("x", -.7, 4);
            moveIn("y", .8, 16);
            robot.intakepush.setPosition(0);
            sleep(1000);
            break;
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public double encoderToIn(String encoder) {

        return ticksToInches(encoder);
    }

    // TO MAKE IT MOVE THE INVERTED DIRECTION IT IS OPPOSITE POWER NOT DISTANCE
    public void moveIn(String xy, double power, double distance) {
        robot.xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.xEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.yEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (xy == "x") {
            while (Math.abs(encoderToIn(xy)) < distance) {
                robot.leftFront.setPower(power);
                robot.leftRear.setPower(power);
                robot.rightFront.setPower(power);
                robot.rightRear.setPower(power * 1.18);
            }
        } else if (xy == "y") {
            while (Math.abs(encoderToIn(xy)) < distance) {
                robot.leftFront.setPower(power);
                robot.leftRear.setPower(-power);
                robot.rightFront.setPower(-power);
                robot.rightRear.setPower(power * 1.1);
            }
        }
        robot.leftFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightRear.setPower(0);
        sleep( 100);
    }

    public double ticksToInches(String encoder) {
        if (encoder == "x") {
            return (robot.xEncoder.getCurrentPosition() / X_TICKSTOINCHES);
        } else if (encoder == "y") {
            return (robot.yEncoder.getCurrentPosition() / Y_TICKSTOINCHES);
        }
        return 0;
    }

    public void armUpDown(double power, int time) {
        robot.leftArm.setPower(-power);
        robot.rightArm.setPower(power);
        sleep(time);
        robot.leftArm.setPower(0);
        robot.rightArm.setPower(0);
    }

    public void openHand(int time) {
        robot.hand.setPower(1);
        sleep(time);
        robot.hand.setPower(0);
    }

    public void closeHand(int time) {
        robot.hand.setPower(-1);
        sleep(time);
        robot.hand.setPower(0);
    }

    public double accelerate(double power) {
        for (int i = 0; i < power;) {
            sleep(100);
            return i;
        }
        return power;
    }

    public void shootRing(int velocity) {
        robot.flywheel.setVelocity(velocity);
        sleep(1450);
        shoot();
        robot.flywheel.setVelocity(0);
    }

    public void shoot() {
        robot.pushRing.setPosition(.64);
        sleep(1000);
        robot.pushRing.setPosition(.4);
    }

    public void turn(double degrees, double power) {
        robot.yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.yEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double target =  (degrees * .1396);
        while (Math.abs(encoderToIn("y")) < target) {
            robot.leftFront.setPower(power);
            robot.leftRear.setPower(power);
            robot.rightFront.setPower(-power);
            robot.rightRear.setPower(-power * 1.13);
        }
        robot.leftFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightRear.setPower(0);
    }
}
