package com.robotlifealert3.examples.pointcloud;

/**
 * Created by rafaelszuminski on 12/22/16.
 */

public class Rover {
    public static final int RIGHT_STOP = 192;
    public static final int LEFT_STOP = 64;

    private int rightMotor, leftMotor;


    public static final int NUM_OF_BUCKETS = 15;

    private Bucket[] buckets;

    public Rover() {
        buckets = new Bucket[NUM_OF_BUCKETS];
        rightMotor = RIGHT_STOP;
        leftMotor = LEFT_STOP;
        leftMotor = LEFT_STOP;
    }








    /*
     **********************************************************************************************
     * Computational/semi-autonomous methods
     **********************************************************************************************
     */

    /*
     * value between -1 and 1 depending on how fast and forward/backward
     * forward is positive
     */
    private void move(double value) {
        rightMotor = RIGHT_STOP - (int)Math.round(value*55);
        leftMotor = LEFT_STOP - (int)Math.round(value*55);

    }
    /*
     * value is between -1 and 1, depending on which direction and how fast you want to turn
     * counterclockwise is positive
     */
    private void turn(double value) {
        rightMotor = RIGHT_STOP + (int)Math.round(value*55);
        leftMotor = LEFT_STOP - (int)Math.round(value*55);

    }
    /*
     * stop motors
     */
    private void stop() {
        rightMotor = RIGHT_STOP;
        leftMotor = LEFT_STOP;

    }

}
