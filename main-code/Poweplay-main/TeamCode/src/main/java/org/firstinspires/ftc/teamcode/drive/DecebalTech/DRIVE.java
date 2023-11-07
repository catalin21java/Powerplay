package org.firstinspires.ftc.teamcode.drive.DecebalTech;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.RobotUtils;

@TeleOp(name="DRIVE", group="Linear Opmode")
@Config


public class DRIVE extends LinearOpMode {

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
        MANUAL,
        STACK
    }
    private ControlMode currentControlMode = ControlMode.AUTO;
    private Pose2d SCORED_CONE_POS = new Pose2d(0,0);
    private double lastLeftStickX = 0;

    private RobotUtils robot;
    private boolean wasManualBefore = false;
    private boolean RUMBLE_CONTROLLER = false;
    private boolean hasDetectedObject = false;
    private boolean movedSlider = false;
    private boolean ghearaDeschisa = false;
    private boolean movedArm = false;
    private boolean ready = false;
    private boolean manualMode = true;
    private boolean intakeIsLifted = false;
    private double distance_threshold = 5;
    private boolean cruiseControl = false;
    private int counter =0;
    private boolean lowDown = false;
    private PoleType pole = PoleType.HIGH;
    private Pose2d[] poleLocations = new Pose2d[25];
    private int intakeCounter=1;
    ElapsedTime lowDownTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime lastRumbleReset = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime lastIntakeTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime lastCircleTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime lastCruiseSwitch = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.experimental.setSinglethreadedOptimized(false);
        PhotonCore.enable();

        robot = new RobotUtils(hardwareMap);


       ;

        robot.rotire.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            robot.rotire.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.slider1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.slider2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("READY",ready);
            telemetry.addData("Curent MODE",currentControlMode);
            double heading = 0;

            if(gamepad2.left_bumper)
            {
                currentControlMode = ControlMode.MANUAL;
            }
            else if(gamepad2.right_bumper)
            {
                counter = 0;
                intakeCounter = 1;
                currentControlMode = ControlMode.STACK;
            }
            else if(gamepad2.triangle) {
                currentControlMode = ControlMode.AUTO;
            }
            if(gamepad2.cross && lastRumbleReset.time() > 1000)
            {
                RUMBLE_CONTROLLER = false;
                lastRumbleReset.reset();
            }
            if(gamepad1.left_bumper)
                heading = -0.1;
            else if(gamepad1.right_bumper)
                heading = 0.1;

            if(pole==PoleType.MEDIUM)
            {
                heading *= 4.5;
            }
            else if(pole==PoleType.LOW)
            {
                heading *= 7;
            }

            double speed = gamepad1.left_stick_y;

            if(gamepad1.left_stick_y - lastLeftStickX > 0.5)
                speed = lastLeftStickX + 0.5;

            if(gamepad1.left_stick_y <= 0.05 && gamepad1.left_stick_y >= -0.05)
                speed = 0;


            if(gamepad1.triangle && lastCruiseSwitch.time() > 1000) {
                cruiseControl = !cruiseControl;
                lastCruiseSwitch.reset();
            }




//                if (timer.time() > 800 && hasDetectedObject && movedSlider && !movedArm) {
//
//                    if (pole == PoleType.HIGH)
//                        robot.goUp();
//                    else if (pole == PoleType.MEDIUM)
//                        robot.goMediumUp();
//                    else if (pole == PoleType.LOW)
//                        robot.goLowUp();
//
//                    movedArm = true;
//                }
//
//                if (gamepad1.square) {
//
//                    robot.deschideGheara();
//
//                    if (movedSlider) {
//                        ghearaDeschisa = true;
//                        SCORED_CONE_POS = drive.getPoseEstimate();
//                        timer.reset();
//                    }
//                }
//
//                double goDownDistance = 11;
//
//                if (pole == PoleType.LOW)
//                    goDownDistance = 5;
//
//                if (currentControlMode == ControlMode.STACK) {
//                    if (gamepad2.right_bumper) // manuela check
//                        gamepad2.rumble(1000);
//
//                    if (gamepad2.dpad_up && lastIntakeTime.time() > 1000) {
//                        intakeCounter--;
//                        lastIntakeTime.reset();
//                    } else if (gamepad2.dpad_down && lastIntakeTime.time() > 1000) {
//                        intakeCounter++;
//                        lastIntakeTime.reset();
//                    }
//
//                    if(gamepad2.circle && lastCircleTime.time() > 1000)
//                    {
//                        if (intakeCounter == 1)
//                            robot.goSliderCon1dinStack();
//                        else
//                            robot.goSliderCondinStack(intakeCounter);
//
//                        robot.goDownStack();
//                        lastCircleTime.reset();
//                        intakeCounter++;
//                    }
//                }
//
//                if (ghearaDeschisa && (robot.getDistance(drive.getPoseEstimate(), SCORED_CONE_POS) > goDownDistance || gamepad2.square)) {
//
//                    ghearaDeschisa = false;
//                    hasDetectedObject = false;
//                    movedSlider = false;
//                    movedArm = false;
//                    RUMBLE_CONTROLLER = false;
//
//                    intakeIsLifted = false;
//
//                    // robot.inchideGheara();
//
//
//
//                    if(currentControlMode == ControlMode.STACK) {
//                        if(pole == PoleType.HIGH) {
//                            if (intakeCounter == 1)
//                                robot.goSliderCon1dinStack();
//                            else
//                                robot.goSliderCondinStack(intakeCounter);
//
//                            intakeCounter++;
//                        }
//
//                        else
//                        {
//                            lowDownTimer.reset();
//                        }
//
//                        if(intakeCounter != 5)
//                            robot.goDownStack();
//                        else
//                        {
//                            robot.goDownStackLast();
//                        }
//
//                        if(pole == PoleType.MEDIUM || pole == PoleType.LOW) {
//                            robot.goDownStack();
//                            if(!lowDown) {
//                                lowDown = true;
//                                lowDownTimer.reset();
//                            }
//                        }
//
//                        robot.slider1.setTargetPosition(robot.slider1.getTargetPosition() - 20);
//                        robot.slider2.setTargetPosition(robot.slider2.getTargetPosition() + 20);
//
//                    } else if (pole == PoleType.HIGH) {
//                        robot.goArmDown();
//                        robot.goDownSlider();
//                    } else if (pole == PoleType.MEDIUM) {
//                        robot.goMediumDown();
//                        robot.goDownMediumSlider();
//                    } else if (pole == PoleType.LOW) {
//                        robot.goLowDown();
//                        if (!lowDown) {
//                            lowDown = true;
//                            lowDownTimer.reset();
//                        }
//                    } else if (pole == PoleType.GROUND)
//                        robot.goDownLowSlider();
//                }
//
//                if (lowDown && lowDownTimer.time() > 600) {
//                    if(currentControlMode == ControlMode.STACK)
//                    {
//                        if (intakeCounter == 1)
//                            robot.goSliderCon1dinStack();
//                        else
//                            robot.goSliderCondinStack(intakeCounter);
//
//                        intakeCounter++;
//                    }
//                    else
//                        robot.goDownLowSlider();
//
//                    lowDown = false;
//                }
//
//                boolean shouldChange = movedArm && movedSlider;
//                PoleType lastPole = pole;
//
//                if (gamepad1.dpad_up)
//                    pole = PoleType.HIGH;
//                else if (gamepad1.dpad_down) {
//                    pole = PoleType.MEDIUM;
//                } else if (gamepad1.dpad_left)
//                    pole = PoleType.LOW;
//                else if (gamepad1.dpad_right)
//                    pole = PoleType.GROUND;
//                else {
//                    shouldChange = false;
//                }
//
//                if (shouldChange) {
//                    RUMBLE_CONTROLLER = false;
//                    if (pole == PoleType.HIGH && (lastPole == PoleType.MEDIUM || lastPole == PoleType.LOW)) {
//                        robot.goUpSlider();
//                        robot.goUp();
//                    } else if (pole == PoleType.MEDIUM && lastPole == PoleType.HIGH) {
//                        robot.goSliderMediumFromHigh();
//                        robot.goMediumFromHigh();
//                    } else if (pole == PoleType.LOW && (lastPole == PoleType.HIGH || lastPole == PoleType.MEDIUM)) {
//                        robot.goSliderLowFromHigh();
//                        robot.goLowFromHigh();
//                    } else if (pole == PoleType.MEDIUM && lastPole == PoleType.LOW) {
//                        robot.goMediumUp();
//
//                    }
//
//                }
//
//                if (pole == PoleType.MEDIUM && robot.rotire.getCurrentPosition() < 100) {
//                    robot.slider1.setPower(0.9);
//                    robot.slider2.setPower(-0.9);
//                }
//
////                if ((robot.dist.getDistance(DistanceUnit.CM) < distance_threshold || robot.dist2.getDistance(DistanceUnit.CM) < distance_threshold) && !RUMBLE_CONTROLLER) {
////                    RUMBLE_CONTROLLER = true;
////                    gamepad1.rumble(1000);
////                    gamepad2.rumble(1000);
////                    distance_threshold = 3;
////                }
//                if ((robot.dist.getDistance(DistanceUnit.CM) < distance_threshold && !RUMBLE_CONTROLLER)) {
//                    RUMBLE_CONTROLLER = true;
//                    gamepad1.rumble(1000);
//                    gamepad2.rumble(1000);
//                    distance_threshold = 3;
//                }
//            }
//            else
//            {
//                wasManualBefore = true;
//                robot.rotire.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//                robot.slider1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.slider2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//                robot.rotire.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                robot.slider1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                robot.slider2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                if(robot.slider1.getCurrentPosition() >= 0 && gamepad2.left_stick_y > 0)
//                {
//                    robot.slider1.setPower(0);
//                    robot.slider2.setPower(0);
//                }
//                else
//                {
//                    robot.slider1.setPower(gamepad2.left_stick_y / 2);
//                    robot.slider2.setPower(-gamepad2.left_stick_y / 2);
//                }
//                robot.rotire.setPower(-gamepad2.right_stick_y);
//
//                if(gamepad2.triangle) // catalin daca vezi asta tati i mnandru de tine
//                {
//                    robot.rotire.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//                    robot.rotire.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////                    robot.slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////                    robot.slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//                    currentControlMode = ControlMode.AUTO;
//                    telemetry.update();
//
//                }
//                if(gamepad2.circle)
//                    robot.deschideGheara();
//                else if(gamepad2.cross)
//                    robot.inchideGheara();
//
//            }
//            if(intakeCounter > 5)
//                currentControlMode = ControlMode.AUTO;
//
//
//            telemetry.addData("COUNTER", counter);
//            telemetry.addData("Manual Mode", manualMode ? "ON" : "OFF");
//            telemetry.addData("intakeCounter",intakeCounter);
//            telemetry.addData("distance",robot.dist.getDistance(DistanceUnit.CM));
//            telemetry.addData("slider1",robot.slider1.getCurrentPosition());
//            telemetry.addData("red",robot.color.red());
//            telemetry.addData("blue",robot.color.blue());
//            telemetry.addData("green",robot.color.green());
//            telemetry.addData("slider2",robot.slider2.getCurrentPosition());
//            telemetry.update();
//
////    //
////    //            telemetry.addData("Object", robot.hasDetectedObject());
////    //            telemetry.addData("Servo",robot.gheara.getPosition());
////    //
////    //            telemetry.addLine();
////    //
////    //            telemetry.addData("slider1",robot.slider1.getCurrentPosition());
////    //            telemetry.addData("slider2",robot.slider2.getCurrentPosition());
////    //            telemetry.addData("rotire",robot.rotire.getCurrentPosition());
////    //
////    //            telemetry.addLine();
////    //
////    //            telemetry.addData("pole",pole);D
////             telemetry.addData("pose",drive.getPoseEstimate());
////

        }
    }




}