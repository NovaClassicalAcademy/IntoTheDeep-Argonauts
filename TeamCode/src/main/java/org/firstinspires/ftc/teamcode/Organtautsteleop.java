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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="BasicTankControl", group="TeleOpMode")
public class Organtautsteleop extends OpMode
{
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;

    CRServo ServoLeft;

    CRServo ServoRight;
    double color_threshold = 0.01;

    double ServoPower = 0.5;


    // Declare OpMode members.


    NormalizedColorSensor colorSensor;
    double rejecttime = 0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        telemetry.addData("Status", "Initialized");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);


        ServoLeft = hardwareMap.get(CRServo.class, "servoLeft");
        ServoRight = hardwareMap.get(CRServo.class, "servoRight");
        ServoLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        ServoRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        double fl_power = (gamepad1.left_stick_y - gamepad1.left_stick_x / 2);
        double fr_power = (gamepad1.right_stick_y + gamepad1.right_stick_x / 2);
        double bl_power = (gamepad1.left_stick_y + gamepad1.left_stick_x / 2);
        double br_power = (gamepad1.right_stick_y - gamepad1.right_stick_x / 2);
        FrontLeft.setPower(fl_power);
        FrontRight.setPower(fr_power);
        BackLeft.setPower(bl_power);
        BackRight.setPower(br_power);

        if (gamepad1.right_bumper) {
            run_intake();
        }
       }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);

    }

    private void run_intake(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        if (rejecttime!=0) {
            if ((time-rejecttime)<2){
                ServoLeft.setPower(-ServoPower);
                ServoRight.setPower(-ServoPower);
            } else{
                rejecttime = 0;
            }
        }
        //no block
        else if (colors.red<color_threshold && colors.blue<color_threshold && colors.green<color_threshold) {
            ServoLeft.setPower(ServoPower);
            ServoRight.setPower(ServoPower);
        }
        //Yellow or Red
        else if ((colors.red<color_threshold && colors.blue>color_threshold && colors.green>color_threshold) ||
                (colors.red>color_threshold && colors.blue<color_threshold && colors.green<color_threshold)){
            ServoLeft.setPower(0);
            ServoRight.setPower(0);
        }
        //Blue
        else {
            rejecttime = time;

            }

        }

}
