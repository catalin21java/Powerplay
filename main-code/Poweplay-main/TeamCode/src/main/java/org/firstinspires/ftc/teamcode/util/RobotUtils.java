package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class RobotUtils {

    public static double SERVO_CLOSED = 0.83;
    public static double SERVO_OPEN = 0.71;
    private final double SLIDER_POWER = -0.95;
    private final double SLIDER_POWER_AUTO = -0.5;
    private final double ARM_POWER_UP = 1;
    private final double ARM_POWER_DOWN = -1;
    private final int ARM_POSITION_UP = 1215;

    private final int ARM_POSITION_LOW = 730;
    private final int ARM_POSITION_MEDIUM = 1175;
    private final int ARM_POSITION_UP_AUTO = 1200;
    
    private final int ARM_POSITION_DOWN = -5;


    private final int SLIDER_POSITION_UP = -1985;

    private final int SLIDER_POSITION_LOW = -1100;
    private final int SLIDER_POSITION_DOWN = -5;
    private final int SLIDER_POSITION_MEDIUM = -1100;

    ///TODO: TREBUIE PUSE MAI SUS!!!!!!!
    public final int AUTO_SLIDER1_CON1_POZ=-560;
    public final int AUTO_SLIDER1_CON2_POZ=-420;
    public final int AUTO_SLIDER1_CON3_POZ=-260;
    public final int AUTO_SLIDER1_CON4_POZ=-140;
    public final int AUTO_SLIDER1_CON5_POZ=5;



    public DcMotor slider1;
    public DcMotor slider2;
    public DcMotor rotire;
    public Servo gheara;
    public RevColorSensorV3 color;
    public RevColorSensorV3 dist;


//    public RevColorSensorV3 dist2;
    public SampleMecanumDrive drive;

    public RobotUtils(HardwareMap hardwareMap)
    {

        ///slider1  = hardwareMap.get(DcMotor.class, "slider1");
        //slider2  = hardwareMap.get(DcMotor.class, "slider2");
       // rotire  = hardwareMap.get(DcMotor.class, "rotire");
      //  gheara = hardwareMap.get(Servo.class, "gheara");
        color = hardwareMap.get(RevColorSensorV3.class, "color");
      //  dist = hardwareMap.get(RevColorSensorV3.class, "dist");
//        dist2 = hardwareMap.get(RevColorSensorV3.class, "dist2");
    }

    public boolean hasDetectedObject()
    {
        return color.red() >= 150 || color.blue() >= 150  ; // 90 110 mergea inainte
    }
    public double getDistance(Pose2d a, Pose2d b)
    {
        return Math.sqrt((a.getX() - b.getX()) * (a.getX() - b.getX()) + (a.getY() - b.getY()) * (a.getY() - b.getY()));
    }
    public void setSliderPositions(int position)
    {
        slider1.setTargetPosition(position);
        slider2.setTargetPosition(-position);
    }

    /**
     * @param position Pozitia la care sa ajunga
     * @param power Putere este transformata in putere absoluta (intre 0 si 1)
     */
    public void goArmToPosition(int position, double power)
    {
        // just in case
        double absPower = Math.abs(power);

        int currentPos = rotire.getCurrentPosition();

        rotire.setTargetPosition(position);
        rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(position > currentPos)
        {
            rotire.setPower(absPower);
        }
        else if(position < currentPos)
        {
            rotire.setPower(-absPower);
        }

    }

    /**
     * @param position Pozitia la care sa ajunga
     *
     * @param power Putere este transformata in putere absoluta (intre 0 si 1)
     */
    public void goSliderToPosition(int position, double power) {
        // just in case
        double absPower = Math.abs(power);

        int currentPos = slider1.getCurrentPosition();

        slider1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setSliderPositions(position);


        if (currentPos > position) // sliderele sunt jos si merg in sus
        {
            slider1.setPower(-absPower);
            slider2.setPower(absPower);
        }
        else if(currentPos < position) // slidere sunt sus si merg in jos
        {
            slider1.setPower(absPower);
            slider2.setPower(-absPower);
        }

    }
   
    public void goUp()
    {
        goArmToPosition(ARM_POSITION_UP, ARM_POWER_UP);
    }
    public void goUpAuto()
    {
        goArmToPosition(ARM_POSITION_UP_AUTO, ARM_POWER_UP);
    }
    public void goUpSlider()
    {
        goSliderToPosition(SLIDER_POSITION_UP,SLIDER_POWER);
    }

    public void goMediumSlider()
    {
        goSliderToPosition(SLIDER_POSITION_MEDIUM,SLIDER_POWER);
    }

    public void goMediumUp()
    {
        goArmToPosition(ARM_POSITION_MEDIUM,ARM_POWER_UP);
    }

    public void goMediumFromHigh()
    {
        goArmToPosition(ARM_POSITION_MEDIUM,ARM_POWER_UP / 2);
    }

    public void goSliderMediumFromHigh()
    {
        goSliderToPosition(SLIDER_POSITION_MEDIUM,SLIDER_POWER);
    }

    public void goLowFromHigh()
    {
        goArmToPosition(ARM_POSITION_LOW, ARM_POWER_UP / 2);
    }

    public void goSliderLowFromHigh()
    {
        goSliderToPosition(ARM_POSITION_MEDIUM,SLIDER_POWER);
    }

    public void goMediumUpAuto()
    {
        goArmToPosition(ARM_POSITION_MEDIUM,ARM_POWER_UP);
    }

    public void goMediumDown()
    {
        goArmToPosition(ARM_POSITION_DOWN, 1);
    }

    public void goDownMediumSlider()
    {
        goSliderToPosition(SLIDER_POSITION_DOWN,0.1);
    }

    public void goLowSlider()
    {
        goSliderToPosition(SLIDER_POSITION_LOW,SLIDER_POWER);
    }

    public void goLowUp()
    {
        goArmToPosition(ARM_POSITION_LOW,ARM_POWER_UP);
    }

    public void goLowDown()
    {
        goArmToPosition(ARM_POSITION_DOWN,0.8);
    }

    public void goDownLowSlider()
    {
        goSliderToPosition(SLIDER_POSITION_DOWN,SLIDER_POWER);
    }
    public void goSliderCondinStack(int counter)
    {
        if(counter==3)
            goSliderCon2dinStack();
        else if(counter==4)
            goSliderCon3dinStack();
        else if(counter==5)
            goSliderCon4dinStack();
        else if(counter==6)
            goSliderCon5dinStack();
    }
    public void goSliderCon1dinStack()
    {
        goSliderToPosition(AUTO_SLIDER1_CON2_POZ,SLIDER_POWER_AUTO);

    }
    public void goSliderCon2dinStack()
    {
        goSliderToPosition(AUTO_SLIDER1_CON2_POZ,SLIDER_POWER_AUTO);
    }
    public void goSliderCon3dinStack()
    {
        goSliderToPosition(AUTO_SLIDER1_CON3_POZ,SLIDER_POWER_AUTO);
    }
    public void goSliderCon4dinStack()
    {
        goSliderToPosition(AUTO_SLIDER1_CON4_POZ,SLIDER_POWER_AUTO);
    }
    public void goSliderCon5dinStack()
    {
        goSliderToPosition(AUTO_SLIDER1_CON5_POZ,SLIDER_POWER_AUTO);
    }

    public void inchideGheara()
    {
        gheara.setPosition(SERVO_CLOSED);
    }
    public void deschideGheara()
    {
        gheara.setPosition(SERVO_OPEN);
    }
    
    public void goArmDown()
    {
        goArmToPosition(ARM_POSITION_DOWN, ARM_POWER_DOWN);
    }
    public void goDownAuto_Further()
    {
        goArmToPosition(-90,ARM_POWER_DOWN);
    }
    public void goDownAuto_New()
    {
        goArmToPosition(-65,ARM_POWER_DOWN);
    }
    public void goDownStack()
    {
        goArmToPosition(-110,ARM_POWER_DOWN);

    }
    public void goDownStackLast()
    {
        goArmToPosition(-20,ARM_POWER_DOWN);
    }
    public void goDownAuto_Cone5()
    {
        goArmToPosition(-50,ARM_POWER_DOWN);
    }
    public void goDownAuto()
    {
        goArmToPosition(-50,ARM_POWER_DOWN);
    }
    public void goDownAutoFinal()
    {
        goArmToPosition(-10,ARM_POWER_DOWN);
    }

    public void goDownSlider()
    {
        goSliderToPosition(SLIDER_POSITION_DOWN,0.55);
    }

    public void hoverSlider()
    {
        goSliderToPosition(-150,0.8);
    }
    public void hoverSliderdrive()
    {
        goSliderToPosition(-200,0.8);
    }

}