package com.robotlifealert3.examples.pointcloud;

import org.rajawali3d.math.vector.Vector2;

/**
 * Created by rafaelszuminski on 12/22/16.
 */

public class Bucket {
    private Vector2 approx_loc;
    private boolean scanned;

    public Bucket(Vector2 loc) {
        approx_loc = loc;
        scanned = false;
    }



    public void scanBucket() {
        //track the information from QR code somehow here
        //I assume this will be done through a method call from another class
        //store a more precise location of the bucket to approx_loc;
        scanned = true;
    }
}
