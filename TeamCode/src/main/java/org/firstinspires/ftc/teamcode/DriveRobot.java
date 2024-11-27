package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//*TELEMETRY- Set power so that it will move but do sleep 2 sec so the servo moves*//

@TeleOp(name="Robot Centric Drive", group="Teleop")
//@Disabled
public class DriveRobot extends OpMode
{
    // Declare OpMode members.

    private DcMotorSimple frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor;

    //This is the Gyro (actually the Inertial Measurement Unit)
    IMU imu;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Drivetrain
        frontLeftMotor = hardwareMap.dcMotor.get("FrontLeft");//Hub - Port #2
        backLeftMotor = hardwareMap.dcMotor.get("BackLeft");//Hub - Port # 1
        frontRightMotor = hardwareMap.dcMotor.get("FrontRight");//Hub - Port #0
        backRightMotor = hardwareMap.dcMotor.get("BackRight");//Hub - Port #3

        frontLeftMotor.resetDeviceConfigurationForOpMode();
        backLeftMotor.resetDeviceConfigurationForOpMode();
        frontRightMotor.resetDeviceConfigurationForOpMode();
        backRightMotor.resetDeviceConfigurationForOpMode();

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Define the IMU (gyro sensor)
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        //Field-centric initialization - end
        imu.resetYaw();  //reset the gyro
    }


    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //*************************
        //* Field-centric driving *
        //*************************

        // Rotate the movement direction counter to the bot's rotation
        double driveY = -gamepad1.left_stick_y; // Forward/Backward
        double driveX = gamepad1.left_stick_x;  // Left/Right (Strafe)
        double turn = gamepad1.right_stick_x;   // Rotation

        double radians = Math.toRadians(robotHeading);

        // Calculate the field-centric drive values
        double tempX = driveX * Math.cos(radians) - driveY * Math.sin(radians);
        double tempY = driveX * Math.sin(radians) + driveY * Math.cos(radians);

        double frontLeftPower = tempY + tempX + turn;
        double frontRightPower = tempY - tempX - turn;
        double backLeftPower = tempY - tempX + turn;
        double backRightPower = tempY + tempX - turn;

        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }
        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);

        // Update telemetry
        telemetry.addData("Heading", robotHeading);
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Back Right Power", backRightPower);
        telemetry.update();
    }

     /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
