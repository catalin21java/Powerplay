package org.firstinspires.ftc.teamcode.drive.detection;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class DetectionPipeline extends OpenCvPipeline {
    private static final Scalar BLUE = new Scalar(0, 0, 255);
    private static final Scalar RED = new Scalar(255, 0, 0);

    private static final int THRESHOLD = 120;

    Mat luminosityMat = new Mat();
    Mat extractionMat = new Mat();
    Mat rgbMat = new Mat();
    Mat mask = new Mat(rows,cols, CvType.CV_8UC1);
    Mat ret = new Mat(rows,cols, CvType.CV_8UC1);

    private static Scalar lowerOrange = new Scalar(60,135,10);
    private static Scalar upperOrange = new Scalar(190,180,105);

//    private static Scalar lowerRed = new Scalar(0,170,60);
//    private static Scalar upperRed = new Scalar(80,255,255);
    private static Scalar lowerRed = new Scalar(0,130,50);
    private static Scalar upperRed = new Scalar(120,255,255);
    private static ArrayList<MatOfPoint> contur = new ArrayList<MatOfPoint>();
    private static int columnDetected = 0;

    private int init = 0;
    private static int gridSize = 3;
    private static int cols,rows;
    private static int numberOfElements = 0;

    public enum ZoneType
    {
        E_NONE,E_LEFT,E_RIGHT,E_CENTER
    }
    private void inputToLuminosity(Mat input) {
        Imgproc.cvtColor(input, extractionMat, Imgproc.COLOR_RGB2YCrCb); // convert rgb to chroma and luminosity
        Imgproc.cvtColor(extractionMat, rgbMat,Imgproc.COLOR_YCrCb2RGB); // convert rgb to chroma and luminosity

        Core.extractChannel(extractionMat, luminosityMat, 2);

    }

    private Mat[]   allMats = new Mat[256];
    private boolean[] validZones = new boolean[256];
    private Point[] centerZones = new Point[256];

    @Override
    public void init(Mat input)
    {
        if(input == null)
            return;

        init = 1;
        inputToLuminosity(input);

        cols = input.cols();
        rows = input.rows();

        for(int i = 0; i < gridSize * gridSize;i++)
            validZones[i] = false;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        if(input == null)
            return input;

        init = 2;

        inputToLuminosity(input);
        ret.release();
        mask.release();

        numberOfElements = 0;

        for(int i = 0; i < cols;i+= cols / gridSize)
        {
            for(int j = 0; j < rows; j += rows / gridSize)
            {
                int topLeftX = i;
                int topLeftY = j;

                int botRightX = i + cols / gridSize - 1;
                int botRightY = j + rows / gridSize - 1;

                if(botRightX >= cols || botRightY >= rows) continue;

                Point p1 = new Point(topLeftX,topLeftY);

                Point p2 = new Point(botRightX, botRightY);

                Point center = new Point((topLeftX + botRightX) / 2 - 25, (topLeftY + botRightY) / 2);

                allMats[numberOfElements] = rgbMat.submat(new Rect(p1,p2));//luminosityMat.submat(new Rect(p1,p2));

                String zoneName = Integer.toString((numberOfElements + 1));

            //  Imgproc.putText(input, zoneName, center , Imgproc.FONT_ITALIC , 1, BLUE,1);

                Imgproc.rectangle(input, p1, p2, BLUE, 2);
                centerZones[numberOfElements] = center;
                validZones[numberOfElements] = Core.mean(allMats[numberOfElements]).val[0] < THRESHOLD;

                numberOfElements++;
            }

        }
        return input;
    }
    public boolean hasStarted() { return init == 2;}
    // public boolean[] getValidZones() { return validZones; }
    public void setGridSize(int gridSize) { this.gridSize = gridSize;}

    public int getGridSize() { return this.gridSize;}

    public int isColumnDetected(String type,int j) // zone parameter IS index 1 based
    {

        int[] a = new int[256];
        for (int i = 1; i <= 100; i++) {
            String zoneColor = "";
            double r = getZoneRGB(i, 0);
            double g = getZoneRGB(i, 1);
            double b = getZoneRGB(i, 2);

            if (r > g * 1.25 && r > b * 1.25)
                zoneColor = "red";
            else if (b > g * 1.25 && b > r * 1.25)
                zoneColor = "blue";
            else if (Math.abs(r - g) < 40 && r > b * 1.5 && g > b * 1.5)
                zoneColor = "yellow";
            else
                zoneColor = "green";

            if (zoneColor == type && getRow(i) >= 4 && getRow(i) <= 7)
                a[getColumn(i)]++;

        }
        return a[j];
    }
    public int getColumnDetected(String type) // zone parameter IS index 1 based
    {
        int col = 0;
        int c = 0;

        int biggest_col = 0;
        int biggest_c = 0;
        for(int i = 1; i <= 10;i++)
        {
            if(isColumnDetected(type,i) == 4)
            {
                col += i;
                c++;
            }
            else
            {
                if(c >= biggest_c)
                {
                    biggest_c = c;
                    biggest_col = col;
                }

                col = 0;
                c = 0;
            }



        }
        if(c >= biggest_c)
        {
            biggest_c = c;
            biggest_col = col;
        }

        if(biggest_c == 0)
            return biggest_c;

        return biggest_col / biggest_c;
    }

    public double getProperMoveAmount(String type)
    {
        return (5 - getColumnDetected(type)) * 0.6;
    }

    public int getStartColumn(String type)
    {
        int columnStart = -1;
        String zoneColor;

        for(int i = 1; i <= 100;i++)
        {
            double r = getZoneRGB(i,0);
            double g = getZoneRGB(i,1);
            double b = getZoneRGB(i,2);

            if(r > g * 1.25 && r > b * 1.25)
                zoneColor = "red";
            else if(b > g * 1.5 && b > r * 1.5)
                zoneColor = "blue";
            else if(Math.abs(r-g) < 40 && r > b * 1.5 && g > b * 1.5)
                zoneColor = "yellow";
            else
                zoneColor = "green";

            if(zoneColor == "red" && getRow(i) >= 7) {
                columnStart = getColumn(i);
                break;
            }

        }
        return columnStart;
    }

    public int getEndColumn(String type)
    {
        int columnEnd = -1;
        String zoneColor;

        for(int i = 1; i <= 100;i++)
        {
            double r = getZoneRGB(i,0);
            double g = getZoneRGB(i,1);
            double b = getZoneRGB(i,2);

            if(r > g * 1.25 && r > b * 1.25)
                zoneColor = "red";
            else if(b > g * 1.5 && b > r * 1.5)
                zoneColor = "blue";
            else if(Math.abs(r-g) < 40 && r > b * 1.5 && g > b * 1.5)
                zoneColor = "yellow";
            else
                zoneColor = "green";

            if(zoneColor == type && getRow(i) >= 7) {
                columnEnd = getColumn(i);

            }

        }
        return columnEnd;
    }
    public double getMoveAmountMedium(String type)
    {
//        int columnStart = getStartColumn(type);
//        int columnEnd = getEndColumn(type);
//        double moveValue = 0;
//
//        if(columnStart == 1)
//        {
//            if(columnEnd == 8 || columnEnd == 9)
//            {
//                moveValue = 0;
//            }
//            else
//            {
//
//                moveValue = (8 - columnEnd) / 2;
//            }
//        }
//        else
//            moveValue = -(columnStart - 1) / 2;
//
//        return moveValue;

        int col = getColumnDetected(type);
        double moveAmount = 0;
        boolean ok = false;

        if(col < 3)
        {
            moveAmount = (3 - col)+0.5;
            //moveAmount = Math.max(moveAmount,1);
        }
        else if(col > 6)
        {
            moveAmount = (6 - col)-0.5;
        }
        if(col == 3)
            moveAmount = 0.8; //0.6
        else if(col == 6)
            moveAmount = -0.8; //-0.6

        return moveAmount;

    }
    public double getMoveAmount(String type) // zone parameter IS index 1 based
    {
        for(int i = 1; i <= 100;i++)
        {
            String zoneColor ="";
            double r = getZoneRGB(i,0);
            double g = getZoneRGB(i,1);
            double b = getZoneRGB(i,2);

            if(r > g * 1.25 && r > b * 1.25)
                zoneColor = "red";
            else if(b > g * 1.5 && b > r * 1.5)
                zoneColor = "blue";
            else if(Math.abs(r-g) < 40 && r > b * 1.5 && g > b * 1.5)
                zoneColor = "yellow";
            else
                zoneColor = "green";

            if(zoneColor == type && getRow(i) == 6)
                return (6 - getColumn(i)) * 0.6;

        }

        return 0;

    }

    public int getRow(int zone) // zone parameter IS index 1 based
    {
        if (zone == gridSize || zone % gridSize == 0) return gridSize;
        else
            return zone % gridSize;
    }

    public int getColumn(int zone) // zone parameter IS index 1 based
    {
        if(zone % gridSize == 0)
            return zone / gridSize;
        else
            return zone / gridSize + 1;
    }

    public boolean isZoneValid(int zone) { return validZones[zone - 1]; }; // index 1 based

    public double getZoneLuminosity(int zone)
    {
        if(allMats[zone - 1] == null) return -1;

        return Core.mean(allMats[zone - 1]).val[0];
    } // index 1 based

    public double getZoneRGB(int zone, int i)
    {
        if(allMats[zone - 1] == null) return -1;

        return Core.mean(allMats[zone - 1]).val[i];
    } // index 1 based

    public double getZoneLuminosityRGB(int zone, int i)
    {

        return Core.mean(allMats[zone - 1]).val[i];
    } // index 1 based

    public double getAverageLuminosity(int[] zones)
    {
        if(zones.length == 0) return -1;

        double average = 0;

        for(int zone : zones)
            average += getZoneLuminosity(zone);

        return average / zones.length;
    }

    public double getAreaLuminosity(Point p1 , Point p2) { Mat area = luminosityMat.submat(new Rect(p1,p2)); return Core.mean(area).val[0]; }


    public double getAreaRGB(Point p1 , Point p2, int i) { Mat area = rgbMat.submat(new Rect(p1,p2)); return Core.mean(area).val[i]; }




}