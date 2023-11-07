package org.firstinspires.ftc.teamcode.AxonServoTest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name="ServoTest", group="Linear Opmode")
@Config
public class ServoTest extends LinearOpMode {


    public static double servo1_deschis = 1;
    public static double servo1_inchis =0;

    public static double servo2_deschis = 0;
    public static double servo2_inchis =1;

    private DcMotorEx misumi;
    @Override
    public void runOpMode() {
        ServoImplEx axon1 = hardwareMap.get(ServoImplEx.class, "axon1");
        ServoImplEx axon2 = hardwareMap.get(ServoImplEx.class, "axon2");
        DcMotorEx misumi = hardwareMap.get(DcMotorEx.class,"misumi");
        axon1.setPosition(servo1_inchis);
        axon2.setPosition(servo2_inchis);
        waitForStart();
        while (opModeIsActive()) {


            axon1.setPwmEnable();
            axon1.setPwmRange(new PwmControl.PwmRange(500, 2500));


            axon2.setPwmRange(new PwmControl.PwmRange(500, 2500));
//            axon1.setPwmRange(new PwmControl.PwmRange(500, 3000));

            if(gamepad1.right_stick_x>0) misumi.setPower(1);
            else if(gamepad1.right_stick_x<0) misumi.setPower(-1);
            else misumi.setPower(0);

            misumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if(gamepad1.square){
                axon1.setPosition(servo1_deschis);
                axon2.setPosition(servo2_deschis);

            }
            if(gamepad1.circle){
                axon1.setPosition(servo1_inchis);
                axon2.setPosition(servo2_inchis);
            }
            telemetry.addData("pos misumi",misumi.getCurrentPosition());
            telemetry.addData("isPwmEnabled",axon1.isPwmEnabled());
            telemetry.update();
    }
}}

