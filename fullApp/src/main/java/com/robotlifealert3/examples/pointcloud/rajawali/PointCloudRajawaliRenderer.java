/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.robotlifealert3.examples.pointcloud.rajawali;

import android.content.Context;
import android.graphics.Color;
import android.view.MotionEvent;

import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.robotlifealert3.examples.QuadTree.FloorPlan;
import com.robotlifealert3.examples.QuadTree.QuadTree;
import com.robotlifealert3.rajawali.Pose;

import org.rajawali3d.Object3D;
import org.rajawali3d.lights.DirectionalLight;
import org.rajawali3d.materials.Material;
import org.rajawali3d.math.Matrix4;
import org.rajawali3d.math.Quaternion;
import org.rajawali3d.math.vector.Vector2;
import org.rajawali3d.math.vector.Vector3;
import org.rajawali3d.primitives.Cube;
import org.rajawali3d.primitives.PointSprite;
import org.rajawali3d.renderer.RajawaliRenderer;

import java.util.ArrayList;
import java.util.List;
import java.util.Stack;


/**
 * Renderer for Point Cloud data.
 */
public class PointCloudRajawaliRenderer extends RajawaliRenderer {

    private static final float CAMERA_NEAR = 0.01f;
    private static final float CAMERA_FAR = 200f;
    private static final int MAX_NUMBER_OF_POINTS = 60000;

    private TouchViewHandler mTouchViewHandler;

    private DirectionalLight mDirectionalLight;

    // Objects rendered in the scene.
    private PointCloud mPointCloud;
    private FrustumAxes mFrustumAxes;
    private Grid mGrid;
    private PathMap mPathMap;


    private Pose mPose;

    public Vector3 lastPose;


    private Object3D mBoxesBox;


    //A*
    public static final int QUAD_TREE_START = -60;
    public static final int QUAD_TREE_RANGE = 120;
    private final QuadTree data;
    private FloorPlan floorPlan;
    private Pose startPoint;
    private Pose endPoint;
    private List<Cube> pathCubes = new ArrayList<>();
    private boolean fillPath = false;
    private Material blue;
    private boolean renderVirtualObjects;
    //

    public PointCloudRajawaliRenderer(Context context) {
        super(context);
        data = new QuadTree(new Vector2(QUAD_TREE_START, QUAD_TREE_START), QUAD_TREE_RANGE, 8);
        mTouchViewHandler = new TouchViewHandler(mContext, getCurrentCamera());
    }

    @Override
    protected void initScene() {
        mGrid = new Grid(100, 1, 1, 0x000000);
        mGrid.setPosition(0, -1.3f, 0);
        getCurrentScene().addChild(mGrid);

        mPathMap = new PathMap();
        mPathMap.setPosition(0, -1.3f, 0);
        getCurrentScene().addChild(mPathMap);

        final Material material = new Material();
        mBoxesBox = new Cube(1.0f);
        mBoxesBox.setMaterial(material);
        mBoxesBox.setColor(0xff990000);
        mBoxesBox.setScale(.2f);
        mBoxesBox.setShowBoundingVolume(true);
        getCurrentScene().addChild(mBoxesBox);

        mFrustumAxes = new FrustumAxes(3);
        getCurrentScene().addChild(mFrustumAxes);
        //getCurrentScene().addChild(new CoordinateTrident());

        mDirectionalLight = new DirectionalLight();
        mDirectionalLight.setLookAt(1, -1, -1);
        mDirectionalLight.enableLookAt();
        mDirectionalLight.setPosition(-4, 10, -4);
        mDirectionalLight.setPower(2);
        getCurrentScene().addLight(mDirectionalLight);


        // Indicate 4 floats per point since the point cloud data comes
        // in XYZC format.
        mPointCloud = new PointCloud(MAX_NUMBER_OF_POINTS, 4);
        mPointCloud.setShowBoundingVolume(true);
        getCurrentScene().addChild(mPointCloud);
        getCurrentScene().setBackgroundColor(Color.DKGRAY);
        getCurrentCamera().setNearPlane(CAMERA_NEAR);
        getCurrentCamera().setFarPlane(CAMERA_FAR);
        getCurrentCamera().setFieldOfView(87.5);



        //A*
        blue = new Material();
        blue.setColor(Color.BLUE);

        floorPlan = new FloorPlan(data);
        getCurrentScene().addChild(floorPlan);
        floorPlan.setVisible(renderVirtualObjects);
        //
    }
    

    /**
     * Updates the rendered point cloud. For this, we need the point cloud data and the device pose
     * at the time the cloud data was acquired.
     * NOTE: This needs to be called from the OpenGL rendering thread.
     */
    public void updatePointCloud(TangoPointCloudData pointCloudData, float[] openGlTdepth) {
        mPointCloud.updateCloud(pointCloudData.numPoints, pointCloudData.points);
        Matrix4 openGlTdepthMatrix = new Matrix4(openGlTdepth);
        mPointCloud.setPosition(openGlTdepthMatrix.getTranslation());
        // Conjugating the Quaternion is need because Rajawali uses left handed convention.
        mPointCloud.setOrientation(new Quaternion().fromMatrix(openGlTdepthMatrix).conjugate());


//            addBlueBox(openGlTdepthMatrix);

    }

    public QuadTree getFloorPlanData() {
        return data;
    }

    private void addBlueBox(Matrix4 openGlTdepthMatrix) {



        Stack<Vector3> points = new Stack<>();

        points.add(new Vector3(0,0,0));
        points.add(new Vector3(0.01,0.01,0.01));


        final Material material = new Material();
        PointSprite blueLine = new PointSprite(0.09f,0.09f);
        blueLine.setMaterial(material);
        blueLine.setColor(0xff0000ff);

        //blueLine.setScale(.2f);
        //blueLine.setShowBoundingVolume(true);
        blueLine.setPosition(openGlTdepthMatrix.getTranslation());
        getCurrentScene().addChild(blueLine);
    }

    /**
     * Updates our information about the current device pose.
     * NOTE: This needs to be called from the OpenGL rendering thread.
     */
    public void updateCameraPose(TangoPoseData cameraPose) {
        float[] rotation = cameraPose.getRotationAsFloats();
        float[] translation = cameraPose.getTranslationAsFloats();
        Quaternion quaternion = new Quaternion(rotation[3], rotation[0], rotation[1], rotation[2]);
        mFrustumAxes.setPosition(translation[0], translation[1], translation[2]);
        // Conjugating the Quaternion is need because Rajawali uses left handed convention for
        // quaternions.
        mFrustumAxes.setOrientation(quaternion.conjugate());
        mTouchViewHandler.updateCamera(new Vector3(translation[0], translation[1], translation[2]),
                quaternion);
    }

    @Override
    public void onOffsetsChanged(float v, float v1, float v2, float v3, int i, int i1) {
    }

    @Override
    public void onTouchEvent(MotionEvent motionEvent) {
        mTouchViewHandler.onTouchEvent(motionEvent);
    }

    public void setFirstPersonView() {
        mTouchViewHandler.setFirstPersonView();
    }

    public void setTopDownView() {
        mTouchViewHandler.setTopDownView();
    }

    public void setThirdPersonView() {
        mTouchViewHandler.setThirdPersonView();
    }


    @Override
    protected void onRender(long ellapsedRealtime, double deltaTime) {
        super.onRender(ellapsedRealtime, deltaTime);



    }


}
