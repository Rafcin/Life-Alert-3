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

/*
                [Robot Life Alert V3]
       Phab2Pro And Yellowstone Compatible.
       Written by Rafael Szuminski and Denny Frayne.

       Developed for the UCI Rescue Robotics competition.
       Special Thanks to:
       |--------------------------|
            Rafael Szuminski Sr.
            Chuck Knowledge.
            Doug Havard.
            James Tellier
            Terrilou Tiongson
            Denny Frayne
            Shara Chernow
            Kris Kitchen
            Zaid Abouqamar
            Woldo Wondare
            Danny Bernal
            Mrs. Q
        |--------------------------|
*/

package com.robotlifealert3.examples.pointcloud;

import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Matrix;
import android.hardware.Camera;
import android.hardware.display.DisplayManager;
import android.media.AudioTrack;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.Display;
import android.view.InputDevice;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.TextView;
import android.widget.Toast;

import com.google.atap.tango.ux.TangoUx;
import com.google.atap.tango.ux.TangoUx.StartParams;
import com.google.atap.tango.ux.TangoUxLayout;
import com.google.atap.tango.ux.UxExceptionEvent;
import com.google.atap.tango.ux.UxExceptionEventListener;
import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.Tango.OnTangoUpdateListener;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoTextureCameraPreview;
import com.google.atap.tangoservice.TangoXyzIjData;
import com.robotlifealert3.common.Robot.GenerateTone;
import com.robotlifealert3.examples.IOIO.IOIO_thread;
import com.robotlifealert3.examples.Map.Map;
import com.robotlifealert3.examples.Map.MapClass;
import com.robotlifealert3.examples.QuadTree.FloorPlan;
import com.robotlifealert3.examples.pointcloud.rajawali.PointCloudRajawaliRenderer;
import com.robotlifealert3.fullApp.R;

import junit.framework.Assert;

import org.rajawali3d.math.vector.Vector3;
import org.rajawali3d.scene.ASceneFrameCallback;
import org.tensorflow.demo.CameraConnectionFragment;
import org.tensorflow.demo.Classifier;
import org.tensorflow.demo.RecognitionScoreView;
import org.tensorflow.demo.TensorFlowClassifier;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import ioio.lib.util.IOIOLooper;
import ioio.lib.util.IOIOLooperProvider;
import ioio.lib.util.android.IOIOAndroidApplicationHelper;

/**
 * Main Activity class for the Point Cloud Sample. Handles the connection to the {@link Tango}
 * service and propagation of Tango PointCloud data to OpenGL and Layout views. OpenGL rendering
 * logic is delegated to the {@link PointCloudRajawaliRenderer} class.
 */
public class MainActivity extends Activity implements IOIOLooperProvider {

    private static final String TAG = MainActivity.class.getSimpleName();
    private static final int SECS_TO_MILLISECS = 1000;

    private static final String sTranslationFormat = "Translation: %S, %S, %S";
    private static final String sRotationFormat = "Rotation: %S, %S, %S, %S";

    //private final IOIOAndroidApplicationHelper helper_ = new IOIOAndroidApplicationHelper(this, this);

    // Video TextureCamPre allows you to do getBackground(); and that leads to getting the Bitmap.
    private TangoTextureCameraPreview tangoCameraPreview;

    public Bitmap bmA;

    TensorFlowClassifier mTFlow = new TensorFlowClassifier();

    private static final int NUM_CLASSES = 1001;
    private static final int INPUT_SIZE = 224;
    private static final int IMAGE_MEAN = 117;
    private static final float IMAGE_STD = 1;
    private static final String INPUT_NAME = "input:0";
    private static final String OUTPUT_NAME = "output:0";

    private static final String MODEL_FILE = "file:///android_asset/tensorflow_inception_graph.pb";
    private static final String LABEL_FILE =
            "file:///android_asset/imagenet_comp_graph_label_strings.txt";

    private Integer sensorOrientation;

    private TextView mFinalResTF;
    private TextView mAccurate;

    private Map myMap;


    private Tango mTango;
    private TangoConfig mConfig;
    private TangoUx mTangoUx;

    private PointCloudRajawaliRenderer mRenderer;
    //private RajawaliSurfaceView mSurfaceView;
    private TextView mPointCountTextView;
    private TextView mAverageZTextView;

    private PointCloudRajawaliRenderer mPointRender;

    private TextView mPositionText;
    private TextView mRotationText;
    private TextView mResultsTF;

    // ioio variables
    IOIO_thread m_ioio_thread;

    private BaseRover mRover;

    private final IOIOAndroidApplicationHelper helper_ = new IOIOAndroidApplicationHelper(this, this); // from IOIOActivity


    private double mPointCloudPreviousTimeStamp;
    private boolean mIsConnected = false;

    private FloorPlan mFloorPlan;



    private Vector3 currentPos;

    private static final DecimalFormat FORMAT_THREE_DECIMAL = new DecimalFormat("0.000");
    private static final double UPDATE_INTERVAL_MS = 100.0;

    private double mPointCloudTimeToNextUpdate = UPDATE_INTERVAL_MS;

    private int mDepthCameraToDisplayRotation = 0;

    private static final int PERMISSIONS_REQUEST = 1;

    private static final String PERMISSION_CAMERA = Manifest.permission.CAMERA;
    private static final String PERMISSION_STORAGE = Manifest.permission.WRITE_EXTERNAL_STORAGE;
    private RecognitionScoreView scoreView;

    final Bitmap[] bm = new Bitmap[1];
    Bitmap bmC = Bitmap.createBitmap(224, 224, Bitmap.Config.ARGB_8888);

    public MapClass mapClass;

    public MainActivity() throws FileNotFoundException {
    }





    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        this.requestWindowFeature(Window.FEATURE_NO_TITLE);
        this.getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_point_cloud);
        //setContentView(R.layout.activity_camera);



        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);



        mTFlow.initializeTensorFlow(getAssets(), MODEL_FILE, LABEL_FILE, NUM_CLASSES, INPUT_SIZE, IMAGE_MEAN, IMAGE_STD,
               INPUT_NAME, OUTPUT_NAME);




        //--Works--RGB Video
        tangoCameraPreview = (TangoTextureCameraPreview) findViewById(R.id.cameraPreview);

        helper_.create(); // from IOIOActivity

        mPointCountTextView = (TextView) findViewById(R.id.point_count_textview);
        mAverageZTextView = (TextView) findViewById(R.id.average_z_textview);
        //mSurfaceView = (RajawaliSurfaceView) findViewById(R.id.gl_surface_view);
        mPositionText = (TextView) findViewById(R.id.position_textview);
        mRotationText = (TextView) findViewById(R.id.rotation_textview);

        mFinalResTF = (TextView) findViewById(R.id.resultTF);

        mTangoUx = setupTangoUxAndLayout();
        mRenderer = new PointCloudRajawaliRenderer(this);

        mRover = new BaseRover();
        mapClass = new MapClass();

        //myMap = (Map) findViewById(R.id.myMap);






        setupRenderer();

        DisplayManager displayManager = (DisplayManager) getSystemService(DISPLAY_SERVICE);
        if (displayManager != null) {
            displayManager.registerDisplayListener(new DisplayManager.DisplayListener() {
                @Override
                public void onDisplayAdded(int displayId) {

                }

                @Override
                public void onDisplayChanged(int displayId) {
                    synchronized (this) {
                        //setAndroidOrientation();
                    }
                }

                @Override
                public void onDisplayRemoved(int displayId) {}
            }, null);
        }

        if (hasPermission()) {
            if (null == savedInstanceState) {

                //Instansiates the TensorflowView
                //setFragment();
            }
        } else {
            requestPermission();
        }
    }


    @Override
    public void onRequestPermissionsResult(int requestCode, String permissions[], int[] grantResults) {
        switch (requestCode) {
            case PERMISSIONS_REQUEST: {
                if (grantResults.length > 0
                        && grantResults[0] == PackageManager.PERMISSION_GRANTED
                        && grantResults[1] == PackageManager.PERMISSION_GRANTED) {
                    setFragment();
                } else {
                    requestPermission();
                }
            }
        }
    }

    private boolean hasPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            return checkSelfPermission(PERMISSION_CAMERA) == PackageManager.PERMISSION_GRANTED && checkSelfPermission(PERMISSION_STORAGE) == PackageManager.PERMISSION_GRANTED;
        } else {
            return true;
        }
    }

    private void requestPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            if (shouldShowRequestPermissionRationale(PERMISSION_CAMERA) || shouldShowRequestPermissionRationale(PERMISSION_STORAGE)) {
                Toast.makeText(MainActivity.this, "Camera AND storage permission are required for this demo", Toast.LENGTH_LONG).show();
            }
            requestPermissions(new String[] {PERMISSION_CAMERA, PERMISSION_STORAGE}, PERMISSIONS_REQUEST);
        }
    }

    private void setFragment() {
        getFragmentManager()
                .beginTransaction()
                .replace(R.id.container, CameraConnectionFragment.newInstance())
                .commit();
    }

    @Override
    protected void onResume() {
        super.onResume();


        new LongOperation().execute("");

        //setAndroidOrientation();

        mTangoUx.start(new StartParams());
        // Initialize Tango Service as a normal Android Service, since we call mTango.disconnect()
        // in onPause, this will unbind Tango Service, so every time when onResume gets called, we
        // should create a new Tango object.
        mTango = new Tango(MainActivity.this, new Runnable() {
            // Pass in a Runnable to be called from UI thread when Tango is ready, this Runnable
            // will be running on a new thread.
            // When Tango is ready, we can call Tango functions safely here only when there is no UI
            // thread changes involved.
            @Override
            public void run() {
                // Synchronize against disconnecting while the service is being used in the OpenGL
                // thread or in the UI thread.

                synchronized (MainActivity.this) {
                    try {
                        mConfig = setupTangoConfig(mTango);
                        mTango.connect(mConfig);
                        startupTango();
                        mIsConnected = true;
                    } catch (TangoOutOfDateException e) {
                        if (mTangoUx != null) {
                            mTangoUx.showTangoOutOfDate();
                        }
                        Log.e(TAG, getString(R.string.exception_out_of_date), e);
                    } catch (TangoErrorException e) {
                        Log.e(TAG, getString(R.string.exception_tango_error), e);
                    } catch (TangoInvalidException e) {
                        Log.e(TAG, getString(R.string.exception_tango_invalid), e);
                    }
                }
            }
        });

    }

    @Override
    protected void onPause() {
        super.onPause();

        // Synchronize against disconnecting while the service is being used in the OpenGL
        // thread or in the UI thread.
        // NOTE: DO NOT lock against this same object in the Tango callback thread.
        // Tango.disconnect will block here until all Tango callback calls are finished.
        // If you lock against this object in a Tango callback thread it will cause a deadlock.
        synchronized (this) {
            try {
                //tangoCameraPreview.disconnectFromTangoCamera();
                mTangoUx.stop();
                mTango.disconnect();
                mIsConnected = false;
            } catch (TangoErrorException e) {
                Log.e(TAG, getString(R.string.exception_tango_error), e);
            }
        }
    }

    /**
     * Sets up the tango configuration object. Make sure mTango object is initialized before
     * making this call.
     */
    private TangoConfig setupTangoConfig(Tango tango) {
        // Use the default configuration plus add depth sensing.
        TangoConfig config = tango.getConfig(TangoConfig.CONFIG_TYPE_DEFAULT);

        config.putBoolean(TangoConfig.KEY_BOOLEAN_COLORCAMERA, true);
        tangoCameraPreview.connectToTangoCamera(mTango, TangoCameraIntrinsics.TANGO_CAMERA_COLOR);
        //tangoCameraPreview.setRotation(90);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
        config.putInt(TangoConfig.KEY_INT_DEPTH_MODE, TangoConfig.TANGO_DEPTH_MODE_POINT_CLOUD);
        return config;
    }

    /**
     * Set up the callback listeners for the Tango service and obtain other parameters required
     * after Tango connection.
     * Listen to updates from the Point Cloud and Tango Events and Pose.
     */
    private void startupTango() {
        ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<TangoCoordinateFramePair>();

        framePairs.add(new TangoCoordinateFramePair(TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));

        mTango.connectListener(framePairs, new OnTangoUpdateListener() {
            @Override
            public void onPoseAvailable(TangoPoseData pose) {
                // Passing in the pose data to UX library produce exceptions.
                if (mTangoUx != null) {
                    mTangoUx.updatePoseStatus(pose.statusCode);
                }

                //0,1,2
                // XYZ, X Left,Right Y Forward,Back Z UP, DOWN
                float translation[] = pose.getTranslationAsFloats();

                float orientation[] = pose.getRotationAsFloats();

                if(translation[1] > translation[1]){

                }



                final String translationMsg = String.format(sTranslationFormat,
                        FORMAT_THREE_DECIMAL.format(pose.translation[0]),
                        FORMAT_THREE_DECIMAL.format(pose.translation[1]),
                        FORMAT_THREE_DECIMAL.format(pose.translation[2]));
                final String rotationMsg = String.format(sRotationFormat,
                        FORMAT_THREE_DECIMAL.format(pose.rotation[0]),
                        FORMAT_THREE_DECIMAL.format(pose.rotation[1]),
                        FORMAT_THREE_DECIMAL.format(pose.rotation[2]),
                        FORMAT_THREE_DECIMAL.format(pose.rotation[3]));


                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {

                        mPositionText.setText(translationMsg);
                        mRotationText.setText(rotationMsg);
                        //myMap.setMap(mRover.getMap());
                        //myMap.invalidate();




                    }
                });


            }


            @Override
            public void onXyzIjAvailable(TangoXyzIjData xyzIj) {
                // We are not using onXyzIjAvailable for this app.
                if (mTangoUx != null) {
                    mTangoUx.updateXyzCount(xyzIj.xyzCount);
                }
                determineDistance(xyzIj.xyz, xyzIj);
            }

            @Override
            public void onPointCloudAvailable(TangoPointCloudData pointCloud) {
                if (mTangoUx != null) {
                    mTangoUx.updateXyzCount(pointCloud.numPoints);
                }

                //Starts Rendering PointCloud if Enabled.
                //Turned off not to rednder.






                final double currentTimeStamp = pointCloud.timestamp;
                final double pointCloudFrameDelta =
                        (currentTimeStamp - mPointCloudPreviousTimeStamp) * SECS_TO_MILLISECS;
                mPointCloudPreviousTimeStamp = currentTimeStamp;
                final double averageDepth = getAveragedDepth(pointCloud.points,
                        pointCloud.numPoints);

                mPointCloudTimeToNextUpdate -= pointCloudFrameDelta;

                if (mPointCloudTimeToNextUpdate < 0.0) {
                    mPointCloudTimeToNextUpdate = UPDATE_INTERVAL_MS;
                    final String pointCountString = Integer.toString(pointCloud.numPoints);



                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            mPointCountTextView.setText(pointCountString);
                            mAverageZTextView.setText(FORMAT_THREE_DECIMAL.format(averageDepth));



                        }
                    });
                }
            }


            @Override
            public void onTangoEvent(TangoEvent event) {
                if (mTangoUx != null) {
                    mTangoUx.updateTangoEvent(event);
                }
            }

            @Override
            public void onFrameAvailable(int cameraId) {
                int[] loc = mRover.getLocation();
                // We are not using onFrameAvailable for this application.
                if (cameraId == TangoCameraIntrinsics.TANGO_CAMERA_COLOR) {
                    tangoCameraPreview.onFrameAvailable();
                    bm[0] = tangoCameraPreview.getBitmap();
                }
            }
        });
    }



    /**
     * Sets Rajawali surface view and its renderer. This is ideally called only once in onCreate.
     */
    public void setupRenderer() {
        //mSurfaceView.setEGLContextClientVersion(2);
        mRenderer.getCurrentScene().registerFrameCallback(new ASceneFrameCallback() {
            @Override
            public void onPreFrame(long sceneTime, double deltaTime) {
                // NOTE: This will be executed on each cycle before rendering, called from the
                // OpenGL rendering thread

                // Prevent concurrent access from a service disconnect through the onPause event.

            }

            @Override
            public boolean callPreFrame() {
                return true;
            }

            @Override
            public void onPreDraw(long sceneTime, double deltaTime) {

            }

            @Override
            public void onPostFrame(long sceneTime, double deltaTime) {

            }
        });
        //mSurfaceView.setSurfaceRenderer(mRenderer);
    }

    /**
     * Sets up TangoUX layout and sets its listener.
     */
    private TangoUx setupTangoUxAndLayout() {
        TangoUxLayout uxLayout = (TangoUxLayout) findViewById(R.id.layout_tango);
        TangoUx tangoUx = new TangoUx(this);
        tangoUx.setLayout(uxLayout);
        tangoUx.setUxExceptionEventListener(mUxExceptionListener);
        return tangoUx;
    }

    /*
    * This is an advanced way of using UX exceptions. In most cases developers can just use the in
    * built exception notifications using the Ux Exception layout. In case a developer doesn't want
    * to use the default Ux Exception notifications, he can set the UxException listener as shown
    * below.
    * In this example we are just logging all the ux exceptions to logcat, but in a real app,
    * developers should use these exceptions to contextually notify the user and help direct the
    * user in using the device in a way Tango service expects it.
    */
    private UxExceptionEventListener mUxExceptionListener = new UxExceptionEventListener() {

        @Override
        public void onUxExceptionEvent(UxExceptionEvent uxExceptionEvent) {
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_LYING_ON_SURFACE) {
                Log.i(TAG, "Device lying on surface ");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FEW_DEPTH_POINTS) {
                Log.i(TAG, "Very few depth points in mPoint cloud ");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FEW_FEATURES) {
                Log.i(TAG, "Invalid poses in MotionTracking ");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_INCOMPATIBLE_VM) {
                Log.i(TAG, "Device not running on ART");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_MOTION_TRACK_INVALID) {
                Log.i(TAG, "Invalid poses in MotionTracking ");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_MOVING_TOO_FAST) {
                Log.i(TAG, "Invalid poses in MotionTracking ");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_OVER_EXPOSED) {
                Log.i(TAG, "Camera Over Exposed");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_TANGO_SERVICE_NOT_RESPONDING) {
                Log.i(TAG, "TangoService is not responding ");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_UNDER_EXPOSED) {
                Log.i(TAG, "Camera Under Exposed ");
            }

        }
    };

    /**
     * First Person button onClick callback.
     */
    public void onFirstPersonClicked(View v) {
        mRenderer.setFirstPersonView();

    }

    /**
     * Third Person button onClick callback.
     */
    public void onThirdPersonClicked(View v) {
        mRenderer.setThirdPersonView();

    }

    /**
     * Top-down button onClick callback.
     */
    public void onTopDownClicked(View v) {
        mRenderer.setTopDownView();

    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        mRenderer.onTouchEvent(event);
        return true;
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        Log.i("activity cycle","main activity being destroyed");
        helper_.destroy();
    }

    @Override
    protected void onStart() {
        super.onStart();
        Log.i("activity cycle","main activity starting");
        helper_.start();
    }

    @Override
    protected void onStop() {
        Log.i("Stop","This log is here because it wants to be here. " +
                "Logs are nice!" +
                "What a waste of lines.." +
                "Welp back to the robot.");
        Log.i("activity cycle","main activity stopping");
        super.onStop();
        helper_.stop();
    }

    /**
     * Calculates the average depth from a point cloud buffer.
     *
     * @param pointCloudBuffer
     * @param numPoints
     * @return Average depth.
     */
    private float getAveragedDepth(FloatBuffer pointCloudBuffer, int numPoints) {
        float totalZ = 0;
        float averageZ = 0;
        if (numPoints != 0) {
            int numFloats = 4 * numPoints;
            for (int i = 2; i < numFloats; i = i + 4) {
                totalZ = totalZ + pointCloudBuffer.get(i);
            }
            averageZ = totalZ / numPoints;
        }
        return averageZ;
    }



    /**
     * Compute the depth camera to display's rotation. This is used for rendering
     * camera in the correct rotation.
     */
    private void setAndroidOrientation() {
        Display display = getWindowManager().getDefaultDisplay();
        Camera.CameraInfo depthCameraInfo = new Camera.CameraInfo();
        Camera.getCameraInfo(1, depthCameraInfo);

        int depthCameraRotation = Surface.ROTATION_0;
        switch(depthCameraInfo.orientation) {
            case 90:
                depthCameraRotation = Surface.ROTATION_90;
                break;
            case 180:
                depthCameraRotation = Surface.ROTATION_180;
                break;
            case 270:
                depthCameraRotation = Surface.ROTATION_270;
                break;
        }

        mDepthCameraToDisplayRotation = display.getRotation() - depthCameraRotation;
        if (mDepthCameraToDisplayRotation < 0) {
            mDepthCameraToDisplayRotation += 4;
        }
    }



    private Bitmap readBitmap() {
        File sdCard = Environment.getExternalStorageDirectory();

        File directory = new File(sdCard.getAbsolutePath() + "/saved_images");

        File file = new File(directory, "tCPP.jpg"); //or any other format supported

        FileInputStream streamIn = null;
        try {
            streamIn = new FileInputStream(file);
            final Bitmap beepmap = BitmapFactory.decodeStream(streamIn); //This gets the image
            streamIn.close();
            return beepmap;
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

        return null;
    }

    @Override
    public IOIOLooper createIOIOLooper(String connectionType, Object extra) {
        return null;
    }


    private class LongOperation extends AsyncTask<String, String, String> {





        public void SaveImage(Bitmap finalBitmap) {

            String root = Environment.getExternalStorageDirectory().toString();
            File myDir = new File(root + "/saved_images");
            myDir.mkdirs();
            Random generator = new Random();
            int n = 10000;
            n = generator.nextInt(n);
            String fname = "Image-"+ n +".jpg";
            File file = new File (myDir, fname);
            if (file.exists ()) file.delete ();
            try {
                FileOutputStream out = new FileOutputStream(file);
                finalBitmap.compress(Bitmap.CompressFormat.JPEG, 90, out);
                out.flush();
                out.close();

            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        private void drawResizedBitmap(final Bitmap src, final Bitmap dst) {
            Assert.assertEquals(dst.getWidth(), dst.getHeight());
            final float minDim = Math.min(src.getWidth(), src.getHeight());

            final Matrix matrix = new Matrix();

            // We only want the center square out of the original rectangle.
            final float translateX = -Math.max(0, (src.getWidth() - minDim) / 2);
            final float translateY = -Math.max(0, (src.getHeight() - minDim) / 2);
            matrix.preTranslate(translateX, translateY);

            final float scaleFactor = dst.getHeight() / minDim;
            matrix.postScale(scaleFactor, scaleFactor);

            final Canvas canvas = new Canvas(dst);
            canvas.drawBitmap(src, matrix, null);
        }

        public Bitmap converARG(Bitmap src){
            Bitmap bmOut = Bitmap.createBitmap(src.getWidth()+96,src.getHeight() + 96, Bitmap.Config.ARGB_8888);
            return bmOut;

        }


        @Override
        protected String doInBackground(String... params) {
            while(true) {
                try {

                    Log.d("Background","In the Loop");
                    //When Publish is called, onProgress code is called.
                    //publishProgress();

                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {


                        }
                    });


                    Thread.sleep(10);
                    Log.d("Background","slept...");
                    if (bm[0] != null) {


                        //crop it here
                        drawResizedBitmap(bm[0], bmC);

                        //SaveImage(bmC);

                        //List of results
                        final List<Classifier.Recognition> res = mTFlow.recognizeImage(bmC);
                        float conf = 0;
                        Classifier.Recognition highestRec = null;

                        //Goes into a loop.
                        for (Classifier.Recognition rec : res){
                            if (rec.getConfidence() > conf) {
                                conf = rec.getConfidence();
                                highestRec = rec;
                            }
                            Log.d("tag", "" + rec.getConfidence());
                        }

                        if(highestRec != null) {
                            /*
                            Talks to onProgressUpdate and from there the setText for the result is set.
                             */
                            Log.d("publishProg","Published");
                            publishProgress(highestRec.getTitle() + highestRec.getConfidence().toString());
                        }
                        Log.d("TF", "" + res.size());
                        Log.d("TF", "" + res);


                    }


                } catch (InterruptedException e) {
                    Thread.interrupted();
                }
            }
            //return "Executed";
        }

        @Override
        protected void onPostExecute(String result) {

        }

        @Override
        protected void onPreExecute() {}

        @Override
        protected void onProgressUpdate(String... values) {
            //Gets TangoCamera bitmap.

            //SaveImage(tangoCameraPreview.getBitmap());
            mFinalResTF.setText(""+ values[0]);
            Log.d("Background","setText for TF");


        }


    }

    //function that receives distance from
    private void determineDistance(FloatBuffer xyz, final TangoXyzIjData xyzIj){
        double centerCoordinateMax = 0.200;
        double cumulativeZ = 0.0;
        int numberOfPoints = 0;
        for (int i = 0; i < xyzIj.xyzCount; i += 3) {
            float x = xyz.get(i);
            float y = xyz.get(i + 1);

            if (Math.abs(x) < centerCoordinateMax && Math.abs(y) < centerCoordinateMax) {
                float z = xyz.get(i + 2);
                cumulativeZ += z;
                numberOfPoints++;
            }
        }
        Double distanceInMeters;
        if (numberOfPoints > 0) {
            distanceInMeters = cumulativeZ / numberOfPoints;
            alertUser(distanceInMeters);
        } else {
            distanceInMeters = null;
        }
    }

    //Function to alert user based on distance
    private void alertUser(double distance) {
        GenerateTone generateTone = new GenerateTone();

        if (distance > 0 && distance < 2) {
            AudioTrack closeTone = generateTone.generateTone(440, 250);
            //Log.i(TAG, "Distance" + distance);
            closeTone.play();
        } else if (distance > 2 && distance < 3) {
            AudioTrack farTone = generateTone.generateTone(250, 250);
            //Log.i(TAG, "Distance" + distance);
            farTone.play();
        }
    }


    @Override
    public boolean onGenericMotionEvent(MotionEvent event) {
        // Check that the event came from a game controller
        try {
            if ((event.getSource() & InputDevice.SOURCE_JOYSTICK) ==
                    InputDevice.SOURCE_JOYSTICK &&
                    event.getAction() == MotionEvent.ACTION_MOVE) {

                // Process all historical movement samples in the batch
                final int historySize = event.getHistorySize();

                // Process the movements starting from the
                // earliest historical position in the batch
                for (int i = 0; i < historySize; i++) {
                    // Process the event at historical position i
                    processJoystickInput(event, i);
                }

                // Process the current movement sample in the batch (position -1)
                processJoystickInput(event, -1);
                return true;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return super.onGenericMotionEvent(event);
    }

    /*
     * I'm not gonna mess with this, I'll take your word for it that it works
     * However, I don't think we need it because of how the processJoystickInput method works
    */
    private static float getCenteredAxis(MotionEvent event,
                                         InputDevice device, int axis, int historyPos) {
        final InputDevice.MotionRange range =
                device.getMotionRange(axis, event.getSource());

        // A joystick at rest does not always report an absolute position of
        // (0,0). Use the getFlat() method to determine the range of values
        // bounding the joystick axis center.
        if (range != null) {
            final float flat = range.getFlat();
            final float value =
                    historyPos < 0 ? event.getAxisValue(axis) :
                            event.getHistoricalAxisValue(axis, historyPos);

            // Ignore axis values that are within the 'flat' region of the
            // joystick axis center.
            if (Math.abs(value) > flat) {
                return value;
            }
        }
        return 0;
    }

    //Making the joystick work, this allows X and Y Axis controls on the stick to be detected.
    //This also enables us to then add keyevents using Dennys math to check the ammount of force each wheel must
    //turn.
    //   [LStick]:Forward and backwards movement.    [Rstick]: left and right turning.
    //
    private void processJoystickInput(MotionEvent event,
                                      int historyPos) {
        InputDevice mInputDevice = event.getDevice();

        float x = getCenteredAxis(event, mInputDevice,
                MotionEvent.AXIS_Z, historyPos);
        float y = getCenteredAxis(event, mInputDevice,
                MotionEvent.AXIS_Y, historyPos);
        Log.d("Controller", "AnalogX:" + x);
        Log.d("Controller", "AnalogY:" + y);

        // Channel 1 STOP:64 Channel 2 STOP:192

        // x = x+y
        int rightMotor = 0, leftMotor = 0;

        if (Math.round(Math.abs(55 * y)) > 0) {
            rightMotor += Math.round(55 * y);
            leftMotor += Math.round(55 * y);
            if (x > 0) {
                rightMotor -= Math.round(55 * (-x));
            } else if (x < 0) {
                leftMotor += Math.round(55 * (-x));
            }
        } else {
            rightMotor -= Math.round(55 * (-x));
            leftMotor += Math.round(55 * (-x));
        }

        Log.d("Controller", "Motor Left:" + leftMotor);
        Log.d("Controller", "Motor Right:" + rightMotor);

    }


}
