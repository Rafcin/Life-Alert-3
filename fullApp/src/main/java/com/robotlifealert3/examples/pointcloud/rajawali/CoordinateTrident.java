package com.robotlifealert3.examples.pointcloud.rajawali;

/**
 * Created by rafaelszuminski on 11/26/16.
 */

import android.graphics.Color;

import org.rajawali3d.Object3D;
import org.rajawali3d.materials.Material;
import org.rajawali3d.math.vector.Vector3;
import org.rajawali3d.primitives.Cube;
import org.rajawali3d.primitives.Cylinder;

/**
 * @author Jared Woolston (jwoolston@idealcorp.com)
 */
public class CoordinateTrident extends Object3D {

    public CoordinateTrident() {

        final Material material = new Material();

        // Box at origin
        final Cube cube = new Cube(0.25f);
        cube.setMaterial(material);
        cube.setColor(Color.green(7)); // Yellow
        addChild(cube);

        // Axis Arms
        final Cylinder xAxis = new Cylinder(0.75f, 0.02f, 1, 6);
        xAxis.setMaterial(material);
        xAxis.setColor(0xffff0000); // Red
        xAxis.rotate(Vector3.Axis.Y, 90.0);
        xAxis.setPosition(0.375, 0, 0);
        addChild(xAxis);

        /*
        final NPrism xTip = new NPrism(8, 0, 0.1f, 0.25f);
        xTip.setMaterial(material);
        xTip.setColor(0xffff0000); // Red
        xTip.rotate(Vector3.Axis.Z, 90.0);
        xTip.setPosition(0.875f, 0, 0);
        addChild(xTip);
        */

        final Cylinder zAxis = new Cylinder(0.75f, 0.02f, 1, 6);
        zAxis.setMaterial(material);
        zAxis.setColor(0xff0000ff); // Blue
        zAxis.setPosition(0, 0, 0.375);
        addChild(zAxis);


        /*
        final NPrism zTip = new NPrism(8, 0, 0.1f, 0.25f);
        zTip.setMaterial(material);
        zTip.setColor(0xff0000ff); // Blue
        zTip.rotate(Vector3.Axis.X, -90.0);
        zTip.setPosition(0, 0, 0.875f);
        addChild(zTip);
        */

    }
}
