package org.firstinspires.ftc.teamcode.drive.DecebalTech;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RobotUtils;

@TeleOp(name="DRIVE_NORMAL", group="Linear Opmode")
@Disabled
@Config


public class DRIVE_NORMAL extends LinearOpMode {

    enum PoleType
    {
        HIGH,
        MEDIUM,
        LOW,
        GROUND
    }
    enum ModeChassis
    {
        DRIVER_CONTROL,
        TURBO_CONTROL,
        PRECISION_CONTROL
    }
    ModeChassis currentModeChassis = ModeChassis.DRIVER_CONTROL;
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
    ElapsedTime opModeTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public double averageLoopTime = 0;
    public int counterLoop = 0;
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

        robot.rotire.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive() && !isStopRequested()) {

            switch (currentModeChassis) {
                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    gamepad1.left_stick_y/2,
                                    gamepad1.left_stick_x/2,
                                    -gamepad1.right_stick_x/2
                            )
                    );
                    if (gamepad1.right_trigger>0.3) {

                        currentModeChassis = ModeChassis.TURBO_CONTROL;
                    }
                    if (gamepad1.left_trigger>0.3) {

                        currentModeChassis = ModeChassis.PRECISION_CONTROL;
                    }
                    break;
                case TURBO_CONTROL:

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    gamepad1.left_stick_y,
                                    gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );
                    if (gamepad1.right_trigger==0) {

                        currentModeChassis = ModeChassis.DRIVER_CONTROL;
                    }
                    break;
                case PRECISION_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    gamepad1.left_stick_y/4,
                                    gamepad1.left_stick_x/4,
                                    -gamepad1.right_stick_x/4
                            )
                    );
                    if (gamepad1.left_trigger==0) {

                        currentModeChassis = ModeChassis.DRIVER_CONTROL;
                    }
                    break;
            }

            if(robot.hasDetectedObject() && !hasDetectedObject && ready)
            {
                hasDetectedObject = true;

                robot.inchideGheara();
                timer.reset();
            }

            if(timer.time() > 300 && hasDetectedObject && !movedSlider && ready)
            {
                robot.goUpSlider();

                if (pole == PoleType.HIGH)
                    robot.goUpSlider();
                else if (pole == PoleType.MEDIUM) {
                    robot.goMediumSlider();

                }
                else if (pole == PoleType.LOW) {
                    robot.goLowSlider();
                    robot.goLowUp();
                }
                movedSlider = true;
            }

            if(((timer.time() > 700 && pole == PoleType.HIGH) || (timer.time() > 700 && pole == PoleType.MEDIUM)) && hasDetectedObject && movedSlider && !movedArm)
            {

                if(pole == PoleType.HIGH)
                    robot.goUp();
                else
                    robot.goMediumUp();

                movedArm = true;
            }

            if(gamepad1.square)
            {

                robot.deschideGheara();

                if((robot.slider1.getCurrentPosition() > -100 && robot.slider1.getCurrentPosition() < 200) || intakeIsLifted)
                {
                    ready = true;
                }

                if(movedSlider)
                {
                    ghearaDeschisa = true;
                    SCORED_CONE_POS = drive.getPoseEstimate();
                    timer.reset();
                }
            }

            if(intakeCounter > 5) {
                manualMode = false;
                intakeCounter = 1;
            }

            double goDownDistance = 11;

            if(pole == PoleType.LOW)
                goDownDistance = 5;

            if(ghearaDeschisa && (robot.getDistance(drive.getPoseEstimate(),SCORED_CONE_POS) > goDownDistance || (gamepad2.square)))
            {

                ghearaDeschisa = false;
                hasDetectedObject = false;
                movedSlider = false;
                movedArm = false;
                RUMBLE_CONTROLLER = false;
                ready = false;
                intakeIsLifted = false;

                robot.inchideGheara();

                if(manualMode)
                {
                    if(intakeCounter == 1)
                        robot.goSliderCon1dinStack();
                    else
                        robot.goSliderCondinStack(intakeCounter);

                    intakeIsLifted = true;
                    robot.goArmDown();

                    intakeCounter++;
                }
                else if(pole == PoleType.HIGH)
                {
                    robot.goArmDown();
                    robot.goDownSlider();
                }
                else if(pole == PoleType.MEDIUM)
                {
                    robot.goMediumDown();
                    robot.goDownMediumSlider();
                }
                else if(pole == PoleType.LOW)
                {
                    robot.goLowDown();
                    robot.goDownLowSlider();
                }
            }



            if(gamepad1.dpad_up)
                pole=PoleType.HIGH;
            else if(gamepad1.dpad_down) {
                pole = PoleType.MEDIUM;
            }
            else if(gamepad1.dpad_left)
                pole=PoleType.LOW;

            if(pole == PoleType.MEDIUM && robot.rotire.getCurrentPosition()<100)
            {
                robot.slider1.setPower(0.9);
                robot.slider2.setPower(-0.9);
            }

            if(pole == PoleType.MEDIUM)
                distance_threshold = 5;

//            if((robot.dist.getDistance(DistanceUnit.CM) < distance_threshold || robot.dist2.getDistance(DistanceUnit.CM) < distance_threshold) && !RUMBLE_CONTROLLER)
//            {
//                RUMBLE_CONTROLLER = true;
//                gamepad1.rumble(1000);
//                gamepad2.rumble(1000);
//                distance_threshold = 2;
//            }

            if(gamepad2.left_bumper && lastManualMode.time() > 1000)
            {
                manualMode = !manualMode;
                lastManualMode.reset();
                lastIntakeTime.reset();
            }

            if(manualMode)
            {
                if(gamepad2.right_bumper) // manuela check
                    gamepad2.rumble(1000);

                if(gamepad2.dpad_up && lastIntakeTime.time() > 1000)
                {
                    intakeCounter--;
                    lastIntakeTime.reset();
                }
                else if(gamepad2.dpad_down  && lastIntakeTime.time() > 1000)
                {
                    intakeCounter++;
                    lastIntakeTime.reset();
                }

                if(gamepad2.triangle && lastTriangleTime.time() > 1000)
                {
                    if(intakeCounter == 1)
                        robot.goSliderCon1dinStack();
                    else
                        robot.goSliderCondinStack(intakeCounter);

                    if(intakeCounter != 5)
                        intakeIsLifted = true;

                    robot.slider1.setTargetPosition(robot.slider1.getTargetPosition() - 20);
                    robot.slider2.setTargetPosition(robot.slider2.getTargetPosition() + 20);

                    lastTriangleTime.reset();
                    intakeCounter++;
                }
            }

            robot.rotire.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.slider1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.slider2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            drive.update();

            telemetry.addData("READY",ready);
            telemetry.addData("Manual Mode", manualMode ? "ON" : "OFF");
            telemetry.addData("intakeCounter",intakeCounter);
            telemetry.addLine();
//
//            telemetry.addData("Object", robot.hasDetectedObject());
//            telemetry.addData("Servo",robot.gheara.getPosition());
//
//            telemetry.addLine();
//
//            telemetry.addData("slider1",robot.slider1.getCurrentPosition());
//            telemetry.addData("slider2",robot.slider2.getCurrentPosition());
//            telemetry.addData("rotire",robot.rotire.getCurrentPosition());
//
//            telemetry.addLine();
//
//            telemetry.addData("pole",pole);
//            telemetry.addData("distance",Math.min(robot.dist.getDistance(DistanceUnit.CM),robot.dist2.getDistance(DistanceUnit.CM)));
//            telemetry.addData("pose",drive.getPoseEstimate());

            telemetry.update();
        }
    }




}
