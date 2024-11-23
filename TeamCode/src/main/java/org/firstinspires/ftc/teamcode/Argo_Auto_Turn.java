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

@TeleOp(name="Argo_Auto_Turn", group="Teleop")
//@Disabled
public class Argo_Auto_Turn extends OpMode
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
    private double motorPowerAuto = 0.5;
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
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        //Field-centric initialization - end
        imu.resetYaw();  //reset the gyro

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);


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



        // Autonomous
        // move forward

        int target = 200;
        frontLeftMotor.setTargetPosition(target);
        frontRightMotor.setTargetPosition(target);
        backLeftMotor.setTargetPosition(target);
        backRightMotor.setTargetPosition(target);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setPower(motorPowerAuto);
        frontRightMotor.setPower(motorPowerAuto);
        backLeftMotor.setPower(motorPowerAuto);
        backRightMotor.setPower(motorPowerAuto);
//turn left
        target = 100;
        //frontLeftMotor.setTargetPosition(target);
        frontRightMotor.setTargetPosition(target);
        //backLeftMotor.setTargetPosition(target);
        backRightMotor.setTargetPosition(target);
        //frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //frontLeftMotor.setPower(motorPowerAuto);
        frontRightMotor.setPower(motorPowerAuto);
        //backLeftMotor.setPower(motorPowerAuto);
        backRightMotor.setPower(motorPowerAuto);
//move forward
        target = 1500;
        frontLeftMotor.setTargetPosition(target);
        frontRightMotor.setTargetPosition(target);
        backLeftMotor.setTargetPosition(target);
        backRightMotor.setTargetPosition(target);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setPower(motorPowerAuto);
        frontRightMotor.setPower(motorPowerAuto);
        backLeftMotor.setPower(motorPowerAuto);
        backRightMotor.setPower(motorPowerAuto);

        //move hand forward

        servoMain.setPosition(0.4);

        //lift slider up
        if (sliderMotor.getCurrentPosition()<= positionUp) {
            sliderMotor.setTargetPosition(positionUp);
            sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sliderMotor.setPower(motorPower);

            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sliderMotor.setPower(0.0);
            sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //drop specimen;
        servoGrab.setDirection(DcMotor.Direction.FORWARD);
        servoGrab.setPower(.5);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        servoGrab.setPower(0);

        //bring hand back
        servoMain.setPosition(.2);

        //move slider down
        // if (sliderMotor.getCurrentPosition()>= positionDown)
        {
            sliderMotor.setTargetPosition(positionDown);
            sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            sliderMotor.setPower(motorPower);
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sliderMotor.setPower(0.0);
            sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //park robot
        target = -100;
        frontLeftMotor.setTargetPosition(target);
        frontRightMotor.setTargetPosition(target);
        backLeftMotor.setTargetPosition(target);
        backRightMotor.setTargetPosition(target);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setPower(motorPowerAuto);
        frontRightMotor.setPower(motorPowerAuto);
        backLeftMotor.setPower(motorPowerAuto);
        backRightMotor.setPower(motorPowerAuto);

    }

     /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
