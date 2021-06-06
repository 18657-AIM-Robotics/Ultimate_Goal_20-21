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

package org.firstinspires.ftc.teamcode.drive.Comp.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.teamcode.drive.Comp.HardwareBot.Hardware;

@TeleOp(name="NotATeleOp", group="Griffy")
public class NotATeleOp extends OpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware(); // use the class created to define a Pushbot's hardware

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        double speedMultiplier = .8;
//        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        robot.flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.xEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.yEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        // Setup a variable for each drive wheel to save power level for telemetry
        double y = -gamepad1.left_stick_y * 1.2; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.5;
        double rx = gamepad1.right_stick_x * .85;

        frontLeftPower = (y + x + rx);
        backLeftPower = (y - x + rx);
        frontRightPower = (y - x - rx);
        backRightPower = (y + x - rx);
        // Put powers in the range of -1 to 1 only if they aren't already (not
        // checking would cause us to always drive at full speed)

        if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 ||
                Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1 ) {
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(Math.abs(frontRightPower), max);
            max = Math.max(Math.abs(backRightPower), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }
        robot.leftFront.setPower(frontLeftPower * .8);
        robot.leftRear.setPower(backLeftPower * .8);
        robot.rightFront.setPower(frontRightPower * .8);
        robot.rightRear.setPower(backRightPower * .8);

            if (gamepad2.dpad_up) {
                robot.pushRing.setPosition(.63);
            } else if (gamepad2.dpad_down) {
                robot.pushRing.setPosition(.4);
            } else if (gamepad2.dpad_right) {
                robot.angle.setPower(.85);
            } else if (gamepad2.dpad_left) {
                robot.angle.setPower(-.85);
            } else if (gamepad2.left_bumper) {
                robot.intake.setVelocity(3000);
            } else if (gamepad2.right_bumper) {
                robot.intake.setVelocity(0);
            } else if (gamepad2.y) {
                robot.leftArm.setPower(1);
                robot.rightArm.setPower(-1);
            } else if (gamepad2.a) {
                robot.leftArm.setPower(-1);
                robot.rightArm.setPower(1);
            } else if (gamepad2.x) {
                robot.hand.setPower(1);
            } else if (gamepad2.b) {
                robot.hand.setPower(-1);
            } else if (gamepad2.right_trigger > 0) {
                robot.flywheel.setVelocity(3000);
            } else if (gamepad2.left_trigger > 0) {
                robot.flywheel.setPower(2800);
            } else if (gamepad1.a) {
                robot.intakepush.setPosition(0);
            } else if (gamepad1.y) {
                robot.intakepush.setPosition(1);
            } else {
                robot.leftArm.setPower(0);
                robot.rightArm.setPower(0);
                robot.angle.setPower(0);
                robot.hand.setPower(0);
                robot.flywheel.setPower(0);
            }



        // Send telemetry message to signify robot running;
        telemetry.addData("ServoPosition", robot.pushRing.getPosition());
        telemetry.addData("xEncoderPosition", robot.xEncoder.getCurrentPosition());
        telemetry.addData("yEncoderPosition", robot.yEncoder.getCurrentPosition());
        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}