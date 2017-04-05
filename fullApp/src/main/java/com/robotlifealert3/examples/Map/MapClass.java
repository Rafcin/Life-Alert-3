package com.robotlifealert3.examples.Map;

import android.util.Log;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.experimental.TangoTransformation;

/**
 * Created by rafaelszuminski on 1/3/17.
 */

public class MapClass extends TangoPoseData{
//Defined Info//
    public int [][] map;
    public static final int MAP_DIM = 50;

    private static final String TAG = "MapClass";

    private TangoPoseData poseData;
    private Tango tangoMaster;
    private TangoTransformation tangoTransform;
    //Tango Pos Data//


    public MapClass(){
        //Set Map Dimensions N-by-N//
        map = new int[MAP_DIM][MAP_DIM];
        //Init the map//
        mapInit();
        getCurrentPos();

    }

    public void getCurrentPos(){
        Log.d(TAG,"Pose:");
    }



//Sets Map//
    private void mapInit() {
        for(int i = 0; i < map.length; i++)
            for(int j = 0; j < map[i].length; j++)
                map[i][j] = 0;
    }




}