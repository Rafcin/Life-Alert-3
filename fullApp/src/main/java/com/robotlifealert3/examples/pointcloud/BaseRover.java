package com.robotlifealert3.examples.pointcloud;

import android.util.Log;

import org.rajawali3d.math.Quaternion;

import static org.rajawali3d.util.RajLog.TAG;

/**
 * Created by rafaelszuminski on 1/3/17.
 */

public class BaseRover {

    //Motor Values
    public static final int RIGHT_STOP = 192;
    public static final int LEFT_STOP = 64;

    //Map Values
    public static final int MAP_DIM = 50;
    private int[][] map;
    private int[] location;

    //Tango Position
    protected double[] translationTango;
    protected Quaternion rotationTango;

    public BaseRover() {
        map = new int[MAP_DIM][MAP_DIM];
        location = new int[] {MAP_DIM/2, MAP_DIM/2};
        mapInit();
    }



 /*
 **********************************************************************************************
 * Functions for Position and Map
 * @Todo SLAM Operations June 2, 2017
 **********************************************************************************************
 */

    private void mapInit() {
        for(int i = 0; i < map.length; i++)
            for(int j = 0; j < map[i].length; j++)
                map[i][j] = 0;
    }

    public int[][] getMap() {
        return map;
    }

    //location[0] is x, location[1] is y
    public int[] getLocation() {
        return location;
    }
    public double[] getTangoLocation() {
        return new double[]{MAP_DIM/2 + translationTango[0], MAP_DIM/2 + translationTango[1]};
    }

    //polarLocation[0] is r, polarLocation[1] is theta
    public double[] getPolarLocation() {
        double[] loc = new double[2];
        loc[0] = distance(translationTango, new double[]{0,0});
        loc[1] = getYaw();
        return loc;
    }

    public void setTranslationTango(double[] translationTango) {
        this.translationTango = translationTango;

        location[0] = (int)Math.round(getTangoLocation()[0]);
        location[1] = (int)Math.round(getTangoLocation()[1]);
        try {
            if(map[location[0]][location[1]]%2 == 0)
                map[location[0]][location[1]] = 1;
            //Log.d("Rover", "X: " + location[0] + ", Y: " + location[1]);
            //Log.d("Rover", "Yaw: " + getYaw());
        } catch(ArrayIndexOutOfBoundsException e) {
            Log.d(TAG, "You're outside the map!");

        }
    }
    public void setRotationTango(double[] quaternion) {
        rotationTango.setAll(
                quaternion[3],
                quaternion[0],
                quaternion[1],
                quaternion[2]
        );
    }




 /*
 **********************************************************************************************
 * Sub Functions
 * @Todo Rajawali Map
 **********************************************************************************************
 */

    public double getYaw() {
        return Math.atan2(
                2.0 * (rotationTango.x * rotationTango.y + rotationTango.w * rotationTango.z),
                Math.pow(rotationTango.w, 2) + Math.pow(rotationTango.x, 2) -
                        Math.pow(rotationTango.y, 2) - Math.pow(rotationTango.z, 2));
    }


 /*
     *signed angle between two radian angles, ignores multiples of 2pi
 */
    private double signedDeltaAngle(double source, double target) {
        return Math.atan2(Math.sin(target-source), Math.cos(target-source));
    }

    //distance formulas
    protected double distance(double[] coord1, double[] coord2) {
        int dim = coord1.length < coord2.length ? coord1.length : coord2.length;
        double sum = 0;
        for(int i = 0; i < dim; i++) {
            sum += Math.pow(coord1[i] - coord2[i], 2);
        }
        return Math.sqrt(sum);
    }

    /***
     * Calculates and translates direction for a set of x and y based current positions.
     * Adjusts rads to +/- based on the quadrant of destination
     *
     * Tango YAW:
     *       0
     *       |
     *1.57  -+-    -1.57
     *       |
     *   PI or -PI
     *
     * @param startX Rover current x position
     * @param startY Rover current Y position
     * @param destinationX
     * @param destinationY
     * @return
     */
    protected double getTangoYawForCoordinates(double startX, double startY, double destinationX, double destinationY){
        return getTangoYawForCoordinates((int)startX, (int)startY, (int)destinationX, (int)destinationY);
    }

    protected double getTangoYawForCoordinates(int startX, int startY, int destinationX, int destinationY){
        if (startX == destinationX && startY == destinationY){
            return 0;
        }

        //before going into the triangle calculations, check if destinationX or destinationY
        //might be on one of the current x or y axes. In this case we wont have a triangle and
        //need to return one of the predefined directions
        if (startX == destinationX){
            //moving on the Y axis
            if (startY < destinationY){
                //moving up
                return 0;
            } else {
                //moving down on Y
                return Math.PI;
            }
        }

        if (startY == destinationY){
            //moving on the X axis
            if (startX < destinationX){
                return Math.PI / 2 * -1;
            } else {
                return Math.PI / 2;
            }
        }


        //if we are still here, it means we need to calculate the angles using trig
        if (destinationX > startX && destinationY > startY) {
            //upper right quadrant
            //use cos to calc angle
            double adj = destinationY - startY;
            double opp = destinationX - startX;
            double hyp = Math.sqrt((adj * adj) + (opp * opp));
            double angle = Math.acos(adj / hyp);
            double tangoYaw = angle * -1;
            //print "%sx%s in upper right quadrant %s %s " % (x, y, degrees(angle), tango)
            return tangoYaw;

        } else if (destinationX > startX && destinationY < startY){
            //lower right quadrant
            double adj = destinationX - startX;
            double opp = startY - destinationY;
            double hyp = Math.sqrt((adj*adj) + (opp*opp));
            double angle = Math.acos(adj/hyp) + 1.57;
            double tangoYaw = angle * -1;
            return tangoYaw;

        } else if (destinationX < startX && destinationY > startY) {
            // upper right quadrant
            double adj = destinationY - startY;
            double opp = startX - destinationX;
            double hyp = Math.sqrt((adj*adj) + (opp*opp));
            double angle = Math.acos(adj/hyp);
            return angle;

        } else if (destinationX < startX && destinationY < startY) {
            //lower right quadrant
            double adj = startX - destinationX;
            double opp = startY - destinationY;
            double hyp = Math.sqrt((adj * adj) + (opp * opp));
            double angle = Math.acos(adj / hyp) + 1.57;
            return angle;
        }
        return 0;
    }

}
