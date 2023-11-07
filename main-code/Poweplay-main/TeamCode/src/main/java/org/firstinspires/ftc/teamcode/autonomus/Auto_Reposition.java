package org.firstinspires.ftc.teamcode.autonomus;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.broadcom.BroadcomColorSensorImpl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.detection.DetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.RobotUtils;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Auto_reposition", group="Linear Opmode")
@Config

/**
 * se aliniaza cu al treilea pin de la stanga la dreapta al teilului din fata teilului in care este pus!
 */



// TODO: line 70-73 . first rotate down , after slider down and auto fix





public class Auto_Reposition extends LinearOpMode {
    private RobotUtils robot;
    OpenCvCamera webcam;

    DetectionPipeline detectionPipeline;

    boolean bCameraOpened = false;
    private SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        detectionPipeline = new DetectionPipeline();
        webcam.setPipeline(detectionPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                bCameraOpened = true;
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        robot = new RobotUtils(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.rotire.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DetectionPipeline pipeline = new DetectionPipeline();
        int PosSlider=-540;

        Pose2d startPose = new Pose2d(-60.5,-35,0);
        drive.setPoseEstimate(startPose);

        while(!detectionPipeline.hasStarted() && !isStopRequested())
            sleep(100);

        sleep(2000);
        detectionPipeline.setGridSize(10);

        String zoneColor = "red";
        telemetry.addData("zoneColor",zoneColor);

        telemetry.update();
        Point p1 = new Point(375,230);
        boolean bt = false;
        Point p2 = new Point(405, 260);
        int col = 0;
        int columnStart = -1;
        int columnEnd = -1;
        while (!opModeIsActive() && !isStopRequested()) {
            for(int i = 1; i <= 100;i++)
            {
                double r = detectionPipeline.getZoneRGB(i,0);
                double g = detectionPipeline.getZoneRGB(i,1);
                double b = detectionPipeline.getZoneRGB(i,2);

                if(r > g * 1.25 && r > b * 1.25)
                    zoneColor = "red";
                else if(b > g * 1.5 && b > r * 1.5)
                    zoneColor = "blue";
                else if(Math.abs(r-g) < 40 && r > b * 1.5 && g > b * 1.5)
                    zoneColor = "yellow";
                else
                    zoneColor = "green";

                if(zoneColor == "red" && detectionPipeline.getRow(i) >= 7) {
                    columnStart = detectionPipeline.getColumn(i);
                    break;
                }

            }
            for(int i = 1; i <= 100;i++)
            {
                double r = detectionPipeline.getZoneRGB(i,0);
                double g = detectionPipeline.getZoneRGB(i,1);
                double b = detectionPipeline.getZoneRGB(i,2);

                if(r > g * 1.25 && r > b * 1.25)
                    zoneColor = "red";
                else if(b > g * 1.5 && b > r * 1.5)
                    zoneColor = "blue";
                else if(Math.abs(r-g) < 40 && r > b * 1.5 && g > b * 1.5)
                    zoneColor = "yellow";
                else
                    zoneColor = "green";

                if(zoneColor == "red"  && detectionPipeline.getRow(i) >= 7) {
                    columnEnd = detectionPipeline.getColumn(i);

                }

            }
           //  5 6 7 8 9 // rau

            // 2 3 4 5 6 7

            // start column 2  end column 8 binee
//            2 3 4 5 6 7 8 / 5
//                25 / 5 = 5
            // start column 1 end column 4
//            telemetry.addData("red",red);
//            telemetry.addData("green",green);
//            telemetry.addData("blue",blue);
            // average 5

            // 10 9 8 7 / 4 = 19 + 15 = 34 /4 = 8.5 ii 8
            // 1 2 3 / 3  ii 2

            sleep(1000);
            telemetry.addData("column",detectionPipeline.getColumnDetected("blue"));
            for(int i = 1; i <= 10;i++)
                telemetry.addData("column " + Integer.toString(i), detectionPipeline.isColumnDetected("blue",i));
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        col = detectionPipeline.getColumnDetected("blue");
        double moveAmount = 0;
        boolean ok = false;

        if(col < 3)
        {
            moveAmount = (3 - col);
            //moveAmount = Math.max(moveAmount,1);
        }
        else if(col > 6)
        {
            moveAmount = (6 - col);
        }

        if(Math.abs(moveAmount) > 0.5)
        {
            TrajectorySequence reposition = drive.trajectorySequenceBuilder(startPose)
                    .strafeLeft(moveAmount)
                    .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(220), 13.62),
                            SampleMecanumDrive.getAccelerationConstraint(30))
                    .build();

            drive.followTrajectorySequence(reposition);
        }

        sleep(1000);

        robot.color.close();

        telemetry.addData("Object", robot.hasDetectedObject());
        telemetry.addData("Servo",robot.gheara.getPosition());
        telemetry.addData("slider1",robot.slider1.getCurrentPosition());
        telemetry.addData("slider2",robot.slider2.getCurrentPosition());
        telemetry.addData("rotire",robot.rotire.getCurrentPosition());


        telemetry.update();

    }




}