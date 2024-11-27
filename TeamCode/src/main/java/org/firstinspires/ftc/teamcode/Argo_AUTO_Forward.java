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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Argo_AUTO_Forward", group="Linear OpMode")
//@Disabled
public class Argo_AUTO_Forward extends LinearOpMode {

    private DcMotor frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor;

    // Constants
    private static final double WHEEL_DIAMETER = 3.78; // inches
    private static final double COUNTS_PER_REV = 753.2; // Encoder counts per full revolution of the motor
    private static final double DRIVE_GEAR_RATIO = 1.0; // Gear ratio (1:1 for direct drive, adjust if needed)

    // Calculate the distance per encoder tick
    private static final double INCHES_PER_TICK = (Math.PI * WHEEL_DIAMETER) / COUNTS_PER_REV;

    @Override
    public void runOpMode() {

        // Drivetrain
        frontLeftMotor = hardwareMap.dcMotor.get("FrontLeft");//Hub - Port #2
        backLeftMotor = hardwareMap.dcMotor.get("BackLeft");//Hub - Port # 1
        frontRightMotor = hardwareMap.dcMotor.get("FrontRight");//Hub - Port #0
        backRightMotor = hardwareMap.dcMotor.get("BackRight");//Hub - Port #3

        // Set motor modes to run without encoders (using encoders for position control)
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the start button to be pressed
        waitForStart();

        if (opModeIsActive()) {
            // Move robot forward by a specified number of inches (e.g., 24 inches)
            moveForward(24); // Move forward 24 inches (change this value as needed)
        }

    }
    // Function to move the robot forward by a specific distance in inches
    public void moveForward(double inches) {

        // Calculate the number of encoder ticks needed to move the robot the desired distance
        int targetTicks = (int) (inches / INCHES_PER_TICK);

        // Set the target position for each motor
        frontLeftMotor.setTargetPosition(targetTicks);
        frontRightMotor.setTargetPosition(targetTicks);
        backLeftMotor.setTargetPosition(targetTicks);
        backRightMotor.setTargetPosition(targetTicks);

        // Set the motors to run forward (positive power)
        frontLeftMotor.setPower(0.5);  // Adjust power as necessary
        frontRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.5);

        // Wait until all motors reach the target position
        while (opModeIsActive() && (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy())) {
            // Add any additional telemetry or logic if needed
            telemetry.addData("Moving", "Inches: " + inches);
            telemetry.addData("Target Position", targetTicks);
            telemetry.update();
        }

        // Stop motors once the target is reached
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
