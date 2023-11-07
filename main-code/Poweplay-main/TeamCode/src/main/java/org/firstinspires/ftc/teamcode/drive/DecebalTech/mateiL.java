package org.firstinspires.ftc.teamcode.drive.DecebalTech;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RobotUtils;

@TeleOp(name="DRIVE_TEST", group="Linear Opmode")

@Config


public class mateiL extends LinearOpMode {

    enum PoleType
    {
        HIGH,
        MEDIUM,
        LOW,
        GROUND
    }
    enum ControlMode
    {
        AUTO,
        MANUAL
    }
    private ControlMode currentControlMode =ControlMode.AUTO;
    private ColorSensor color;
    private int CLAW_DELAY_TIMER = 0;
    private int CLAW_RESET_TIMER = 0;
    private boolean SCORED_CONE = false;
    private boolean CLAW_ACTIVE=false;
    private Pose2d SCORED_CONE_POS = new Pose2d(0,0);
    private double lastLeftStickX = 0;
    private boolean stopObjectDetection = false;
    private SampleMecanumDrive drive;
    private RobotUtils robot;
    private boolean HAS_CLOSED_CLAW = false;
    private boolean RUMBLE_CONTROLLER = false;
    private boolean hasDetectedObject = false;
    private boolean movedSlider = false;
    private boolean ghearaDeschisa = false;
    private boolean movedArm = false;
    private boolean manualMode = false;
    private int intakeCounter = 1;
    private boolean ready = false;
    private boolean intakeIsLifted = false;
    private double distance_threshold = 5;
    ElapsedTime lastManualMode = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime lastIntakeTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime lastTriangleTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private PoleType pole = PoleType.HIGH;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.experimental.setSinglethreadedOptimized(false);
        PhotonCore.enable();

        robot = new RobotUtils(hardwareMap);


        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();


        if (isStopRequested()) return;

        robot.slider1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.slider2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rotire.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && !isStopRequested()) {

            robot.rotire.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.slider1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.slider2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("READY",ready);
            telemetry.addData("Curent MODE",currentControlMode);
            double heading = 0;

            if(gamepad1.left_bumper)
                heading = -0.1;
            else if(gamepad1.right_bumper)
                heading = 0.1;

            if(pole== PoleType.MEDIUM)
            {
                heading *= 4.5;
            }
            else if(pole== PoleType.LOW)
            {
                heading *= 7;
            }

            double speed = gamepad1.left_stick_y;

            if(gamepad1.left_stick_y - lastLeftStickX > 0.5)
                speed = lastLeftStickX + 0.5;

            if(gamepad1.left_stick_y <= 0.05 && gamepad1.left_stick_y >= -0.05)
                speed = 0;

            if(gamepad1.right_trigger != 0 || gamepad1.left_trigger != 0)
            {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -speed,
                                -gamepad1.right_stick_x,
                                -heading * 7

                        )
                );
            }
            else
            {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -speed / 3,
                                -gamepad1.right_stick_x / 3,
                                -heading

                        )
                );
            }

            lastLeftStickX = speed;
            drive.update();


            if(gamepad2.circle)
                robot.deschideGheara();
            if(gamepad2.square)
                robot.inchideGheara();

            robot.slider1.setPower(gamepad2.left_stick_y / 2);
            robot.slider2.setPower(-gamepad2.left_stick_y / 2);
            robot.rotire.setPower(-gamepad2.right_stick_y);



            drive.update();

            telemetry.addData("READY",ready);
            telemetry.addData("Manual Mode", manualMode ? "ON" : "OFF");
            telemetry.addData("intakeCounter",intakeCounter);
            telemetry.addLine();

            telemetry.update();
        }
    }




}