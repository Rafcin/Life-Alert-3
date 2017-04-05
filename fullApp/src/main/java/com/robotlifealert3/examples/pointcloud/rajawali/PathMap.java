package com.robotlifealert3.examples.pointcloud.rajawali;

import android.opengl.GLES10;
import android.opengl.GLES20;

import org.rajawali3d.Object3D;

/**
 * Created by rafaelszuminski on 1/3/17.
 */

public class PathMap extends Object3D {

    public PathMap(){
        super();
    }

    @Override
    public void preRender() {
        super.preRender();
        setDrawingMode(GLES20.GL_POINTS);
        GLES10.glPointSize(5.0f);
    }
}
