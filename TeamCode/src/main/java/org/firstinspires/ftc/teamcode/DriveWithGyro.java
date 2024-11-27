package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//*TELEMETRY- Set power so that it will move but do sleep 2 sec so the servo moves*//

@TeleOp(name="Drive With Gyro-Field centric", group="Teleop")
//@Disabled
public class DriveWithGyro extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor sliderMotor = null;
    public double triggerSensitivityDeposit;
    public double triggerSensitivityIntake;
    private Servo servoMain;
    private CRServo servoGrab= null;
    //This is the Gyro (actually the Inertial Measurement Unit)
    private IMU imu;

    private double motorPower = 1;
    // Target positions for the slider (in encoder ticks)
    private int positionUp = 5800;   // Example target position for slider up
    private int positionDown = 0;     // Example target position for slider down



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

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //INTAKE / deposit
        sliderMotor = hardwareMap.dcMotor.get("sliderMotor");//EHub- Port #1
        servoMain = hardwareMap.servo.get("servoMain");//Hub - Port #0
        servoGrab = hardwareMap.crservo.get("servoGrab");//Hub - Port #2

        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Reset encoders and set to RUN_USING_ENCODER mode
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Hold the intake in place
        servoMain.setPosition(.3);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     *
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        //drvetrain
        double leftStickY = -gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double speedMax = 1;

        telemetry.addData("left Stick X: ", leftStickX);
        telemetry.addData("left Stick Y: ", leftStickY);
        telemetry.addData("Heading: ", botHeading);

        //*************************
        //* Field-centric driving *
        //*************************

        // Rotate the movement direction counter to the bot's rotation
        double rotX = leftStickX * Math.cos(-botHeading) - leftStickY * Math.sin(-botHeading);
        double rotY = leftStickX * Math.sin(-botHeading) + leftStickY * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightStickX), 1);

        double frontLeftPower = ((rotY + rotX + rightStickX) / denominator) * speedMax;
        double backLeftPower = ((rotY - rotX + rightStickX) / denominator) * speedMax;
        double frontRightPower = ((rotY - rotX - rightStickX) / denominator) * speedMax;
        double backRightPower = ((rotY + rotX - rightStickX) / denominator) * speedMax;

        frontLeftMotor.setPower(-frontLeftPower);
        backLeftMotor.setPower(-backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);


        //intake
        double Servoposition =0.65;

        if ( gamepad2.a)// Brings the intake down
        {
            servoMain.setPosition(Servoposition);
            telemetry.addData("Servo x position", servoMain.getPosition());
        }

        if ( gamepad2.y) //Brings intake up
        {
            servoMain.setPosition(0.4);
            telemetry.addData("Servo b position", servoMain.getPosition());
        }

        if (gamepad2.b) //Intake butter blocks
        {
            servoGrab.setPower(.5);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            servoGrab.setPower(0);
        }
        if (gamepad2.x) //Outake
        {
            servoGrab.setPower(-.5);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            servoGrab.setPower(0);
        }
        //slider
        boolean dpadUp = gamepad2.dpad_up;
        boolean dpadDown = gamepad2.dpad_down;

        if (dpadUp) {
            // Move slider up
            //moveSliderToPosition(positionUp);
            if (sliderMotor.getCurrentPosition()<= positionUp) {
                sliderMotor.setTargetPosition(positionUp);
                sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderMotor.setPower(motorPower);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                sliderMotor.setPower(0.0);
                sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        } else if (dpadDown) {
            // Move slider down
            //moveSliderToPosition(positionDown);
            if (sliderMotor.getCurrentPosition()>= positionDown)
            {
                sliderMotor.setTargetPosition(positionDown);
                sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                sliderMotor.setPower(motorPower);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                sliderMotor.setPower(0.0);
                sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

     /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
