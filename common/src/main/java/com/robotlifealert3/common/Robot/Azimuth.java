package com.robotlifealert3.common.Robot;

/**
 * Created by dkfrayne on 1/29/16.
 */

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.location.Location;
import android.location.LocationListener;
import android.os.Bundle;
import android.util.Log;

import java.util.ArrayList;
//test
public class Azimuth implements SensorEventListener, LocationListener, Comparable<Azimuth> {
    /*
     * I've made some pretty significant changes to this class. Instead of simply
     * storing a instance variable of type double, I have only a type double array of size 2 storing information
     * to compute an integral azimuth to accommodate for the error in the Android compass.
     *
     * This class still needs heavy testing with the robot.
     *
     * It's also waiting for your code to handle Locations, Raf
     */

    ArrayList<Double> compassData;
    private double latitude, longitude;
    private double direction;

    //FIFO controls whether the current direction should account for error
    private final boolean FIFO;

    public Azimuth(int dir, boolean fifo) {
        compassData = new ArrayList();
        Log.d("Azimuth", "I'm a new azimuth :)");
        FIFO = fifo;
        setDirection(dir);
    }

    /*
     * Sum of the 5-ish most recent compass values over 5-ish
     */
    public double getDirection() {
        return direction;
    }


    public void setDirection(double dir) {
        if(FIFO) {
            compassData.add(dir);
            float sum = 0;
            for (double d : compassData) {
                sum += d;
            }
            direction = Math.round(sum / compassData.size());
        }
        else {
            direction = dir;
        }
    }

    //    public void absoluteSetDirection(double newDirection) {
//        compassData = new double[] {newDirection, 1};
//    }
    /*
     * simple, add two compass bearings for the resultant direction
     */
    public static double compassAddition(double dir, double angle) {
        double result = dir + angle;
        if(result > 360)
            return result - 360;
        else if(result < 0)
            return result + 360;
        return result;
    }

    public double add(double angle) {
        setDirection(compassAddition(direction, angle));
        return getDirection();
    }

    /*
     * returns -1 if dir is to the left, 0 if equals, and 1 if to the right
     */
    public int getRelativeSide(Azimuth angle) {
        double dir = angle.getDirection();
        if (dir == getDirection()) return 0;
        if(getDirection() <= 180)
            return (dir < getDirection() || dir > getDirection() + 180) ? -1 : 1;
        return (dir > getDirection() || dir < getDirection() - 180) ? 1 : -1;
    }

    public int compareTo(Azimuth dir) {
        return Math.round(getRelativeSide(dir)
                * Math.round(Math.abs(180 - Math.abs(getDirection() - dir.getDirection())) - 180));
    }

    public void onLocationChanged(Location location) {
        latitude = location.getLatitude();
        longitude = location.getLongitude();
    }

    public void onStatusChanged(String provider, int status, Bundle extras) {

    }

    public void onSensorChanged(SensorEvent event) {
        setDirection(Math.round(event.values[0]));
        //Log.d("SensorEvent", "value: " + getDirection());
    }

    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public void onProviderEnabled(String provider) {

    }

    public void onProviderDisabled(String provider) {

    }
}