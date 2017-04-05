package com.robotlifealert3.examples.Map;

import android.os.AsyncTask;
import android.util.Log;

import org.rajawali3d.math.Quaternion;

import java.util.ArrayList;
import java.util.LinkedList;


/**
 * Created by dkfrayne on 2/26/16.
 */
public class BaseRover {
    public static final String TAG = BaseRover.class.getSimpleName();
    public static final int RIGHT_STOP = 192;
    public static final int LEFT_STOP = 64; //motor values
    public static final int MAP_DIM = 50;
    //protected boolean autonomous; //this being true is intended for overriding manual control with autonomous
    protected int leftMotor;
    protected int rightMotor; //these need to belong strictly to Rover class if Rover is in charge of motion
    protected double[] translationTango;
    protected Quaternion rotationTango;
    private int[][] map;
    private int[] location;
    private boolean out_of_bounds;
    private ArrayList<String> barcodes;

    public BaseRover() {
        leftMotor = LEFT_STOP;
        rightMotor = RIGHT_STOP;
        translationTango = new double[3];
        rotationTango = new Quaternion();
        barcodes = new ArrayList<String>();

        map = new int[MAP_DIM][MAP_DIM];
        location = new int[] {MAP_DIM/2, MAP_DIM/2};
        mapInit();
        out_of_bounds = false;

    }
    protected void safeSleep(int time) {
        try {
            Thread.sleep(time);
        }catch (Exception e){
            e.printStackTrace();
        }
    }

    public void saveBarcode(String barcode) {
        map[location[0]][location[1]] = 3;
        barcodes.add(barcode);
    }

    /*
     **********************************************************************************************
     * Information about the Rover
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

    public int[] getMotors() {
        return new int[] {leftMotor - LEFT_STOP, rightMotor - RIGHT_STOP};
    }

    /*
     * update translationTango from MainActivity
     * also updates coordinate location in the map using these values
     */
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
            //TODO: interrupt AutoAI
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
     * left is positive
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

    /*
     * signed angle between two radian angles, ignores multiples of 2pi
     */
    private double signedDeltaAngle(double source, double target) {
        return Math.atan2(Math.sin(target-source), Math.cos(target-source));
    }

    /*
     * returns current orientation in radians on (-pi, pi)
     * left is positive
     */
    public double getYaw() {
        return Math.atan2(
                2.0 * (rotationTango.x * rotationTango.y + rotationTango.w * rotationTango.z),
                Math.pow(rotationTango.w, 2) + Math.pow(rotationTango.x, 2) -
                        Math.pow(rotationTango.y, 2) - Math.pow(rotationTango.z, 2));
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




    /*
     **********************************************************************************************
     * Autonomous Interface to host the challenge
     **********************************************************************************************
     */
    protected class AutonomousAI extends AsyncTask<Void, Void, Void> {
        private BaseRover.FSM fsm;

        public boolean isPaused(){
            return fsm.isPaused();
        }

        public void setPaused(boolean paused) {
            fsm.setPause(paused);
        }

        @Override
        protected void onPostExecute(Void aVoid) {
            super.onPostExecute(aVoid);

            Log.d(TAG, "AutoAI Task finished");
        }
        @Override
        protected Void doInBackground(Void... params) {
            Log.d(TAG, "AutoAI On Execute");

            fsm = new BaseRover.FSM();
            addTasks();
            fsm.startFsm();

//            while(true) {
//                safeSleep(50);
//                if(isCancelled()) break;
//            }
            return null;
        }

        private void addTasks(){
            BaseRover.HomeTask hometask = new BaseRover.HomeTask();
            fsm.add(hometask);

//            DummyTestTask task = new DummyTestTask();
//            fsm.add(task);

            GotoTask gotoFourth = new GotoTask();
            gotoFourth.setDestination(25,25);
            fsm.add(gotoFourth);

            fsm.add(new FullRotationTask());

            GotoTask goToThird = new GotoTask();
            goToThird.setDestination(23, 23);
            fsm.add(goToThird);

            fsm.add(new FullRotationTask());

            GotoTask goToSecond = new GotoTask();
            goToSecond.setDestination(27, 23);
            fsm.add(goToSecond);

            fsm.add(new FullRotationTask());

            GotoTask goToFirst = new GotoTask();
            goToFirst.setDestination(27, 27);
            fsm.add(goToFirst);


        }

        public void cancel() {
            if (fsm != null){
                fsm.abortCurrentTaskAndKillTasks();
            }
        }

        public void obstacleDetected(byte b) {
            fsm.interruptCurrentTaskWith(new ObstacleDetectedTask(), Integer.toString(b));
        }
    }

    /*
     **********************************************************************************************
     * Following classes define the numerous Asynchronous tasks needed to complete the challenge
     **********************************************************************************************
     */
    public abstract class BaseTask extends AsyncTask<String, String, Void> {
        public final String TAG = BaseTask.class.getSimpleName();

        protected boolean shouldAbort = false;
        protected boolean pause = false;

        protected BaseTask parentTask;

        public abstract void abort();

        public boolean isPause() {
            return pause;
        }

        public void setPause(boolean pause) {
            this.pause = pause;
            if (pause) {
                try {
                    doPause();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

        protected void doPause() throws InterruptedException {
            while (pause){
                Thread.sleep(10);
            }
            Log.d(TAG, this.getClass().getSimpleName() + ".doPause is done");
        }

        /*
         **********************************************************************************************
         * autonomous methods that need to check isCancelled() must be here
         * this includes anything involving a loop or some sort of timer
         **********************************************************************************************
         */


        /***
         * Rotates until Rover faces newYaw
         * @param newYaw Direction to rotate towards. Value should be between -Pi and Pi
         */
        protected void rotateTowards(double newYaw){
            //figure out if we need to rotate left or right
            //for now we will rotate left!
            double direction = 0.5;

            //initiate turn
            Log.d(TAG, "Initiating turn with direction " + direction);
            turn(direction);

            //wait until turn completes or there is a abort or cancel signal
            while(true){
                if (isCancelled() || isPause()) break;
                double delta = getYaw() - newYaw;
                if (Math.abs(delta) < 0.1) break;
                safeSleep(20);
                Thread.yield();
            }

            //stop the rotation
            Log.d(TAG, "Stop turn ");
            stop();
        }


//        protected void goDistance(double distance) {
//            double[] startTango = translationTango;
//            double[] currentTango = startTango;
//
//            move(1);
//            while(distance(startTango, currentTango) < distance) {
//                currentTango = translationTango;
//                Thread.yield();
//                if(isCancelled())
//                    break;
//            }
//            stop();
//        }
        protected void goToCoordinate(int[] coordinate) {
            double targetDirection;
            double[] thisStart;
            while(coordinate[0] != location[0] || coordinate[1] != location[1]) {
                targetDirection = getTangoYawForCoordinates(location[0], location[1], coordinate[0], coordinate[1]);
                Log.d(TAG, "GotoTask.TargetDirection: " + targetDirection);
                if(Math.abs(signedDeltaAngle(getYaw(), targetDirection)) > .3) {
                    stop();
                    safeSleep(50);
                    adjustYaw(targetDirection);
                    safeSleep(50);
                }


                thisStart = getTangoLocation();
                move(.6);
                while(distance(thisStart, getTangoLocation()) < 1.0) {
                    Thread.yield();
                    if(isCancelled()||isPause()) break;
                }
                if(isCancelled()||isPause()) break;
            }
            stop();
        }
        protected void adjustYaw(double target) {
            double delta = signedDeltaAngle(target, getYaw());
            //turn at half power until within wide tolerance
            turn(Math.signum(delta)/2);
            while(Math.abs(delta) > 1) {
                delta = signedDeltaAngle(target, getYaw());
                Log.d(TAG, "signedDeltaAngle: " + signedDeltaAngle(target, getYaw()));
                Thread.yield();
                if(isCancelled()||isPause())
                    break;
            }
            //turn at third power while until smaller tolerance
            turn(Math.signum(delta)/3);
            while(Math.abs(delta) > .26) {
                delta = signedDeltaAngle(target, getYaw());
                Thread.yield();
                if(isCancelled()||isPause())
                    break;
            }
            stop();
        }

        @Override
        protected void onPostExecute(Void aVoid) {
            super.onPostExecute(aVoid);
            if (parentTask != null){
                parentTask.pause = false;
            }
        }
    }
    //end of BaseTask



    /*
     ***
     * GotoTask will go to coordinates passed in setDestination()
     */
    public class GotoTask extends BaseTask {
        private int coordinate[];

        public void setDestination(int... params){
            coordinate = new int[params.length];
            for(int i = 0; i < params.length; i++) {
                coordinate[i] = params[i];
            }
        }

        @Override
        protected Void doInBackground(String... params) {
            Log.d(TAG, GotoTask.class.getSimpleName() + ".start");
            if (params != null && params.length == 2) {
                //parse coordinates from params
                setDestination(Integer.parseInt(params[0]), Integer.parseInt(params[1]));
            }

            //get the direction of the new coordinates
            double[] currentLocation = getTangoLocation();
            double newYaw = getTangoYawForCoordinates(currentLocation[0], currentLocation[1], coordinate[0], coordinate[1]);

            //rotate towards new direction
            //rotateTowards(newYaw);

            if(coordinate != null) {
                goToCoordinate(coordinate);
            }
            Log.d(TAG, GotoTask.class.getSimpleName() + ".end");
            return null;
        }

        @Override
        public void abort() {
            shouldAbort = true;
            cancel(true);
        }

    }

    /***
     * The home task will run as the LAST task in the stack and should get the
     * rover home.
     *
     * TODO: Need to implement the home coordinates!
     *      home coorinates are (MAP_DIM/2, MAP_DIM/2)
     */
    public class HomeTask extends BaseTask {

        @Override
        protected Void doInBackground(String... params) {

            try {
                Log.d(TAG, HomeTask.class.getSimpleName() + ".start");
                for (int i = 0; i < 5; i++) {
                    if (shouldAbort) break;
                    if (pause) doPause();
                    Log.d(TAG, HomeTask.class.getSimpleName() + ".running...");
                    Thread.sleep(1000);
                }
                Log.d(TAG, HomeTask.class.getSimpleName() + ".end");
            } catch (InterruptedException e) {
                Log.d(TAG, HomeTask.class.getSimpleName() + ".abort");
            }

            return null;
        }

        @Override
        public void abort() {
            shouldAbort = true;
            cancel(true);
        }
    }

    public class DummyTestTask extends BaseTask {

        @Override
        protected Void doInBackground(String... params) {

            try {
                Log.d(TAG, DummyTestTask.class.getSimpleName() + ".start");
                for (int i = 0; i < 10; i++) {
                    if (shouldAbort) break;
                    if (pause) doPause();
                    Log.d(TAG, DummyTestTask.class.getSimpleName() + ".running...");
                    Thread.sleep(1000);
                }
                Log.d(TAG, DummyTestTask.class.getSimpleName() + ".end");
            } catch (InterruptedException e) {
                Log.d(TAG, DummyTestTask.class.getSimpleName() + ".abort");
            }

            return null;
        }

        @Override
        public void abort() {
            shouldAbort = true;
            cancel(true);
        }
    }

    /***
     * ObstacleDetectedTask is a interruption task. Essentially an existing task will be interrupted with it
     * and the main purpose is to go around an obstacle
     */
    public class ObstacleDetectedTask extends BaseTask {
        @Override
        protected Void doInBackground(String... params) {
            Log.d(TAG, ObstacleDetectedTask.class.getSimpleName() + ".start");
//            try {
////                for (int i = 0; i < 5; i++) {
////                    if (shouldAbort) break;
////                    Log.d(TAG, ObstacleDetectedTask.class.getSimpleName() + ".running...");
////                    Thread.sleep(1000);
////                }
////
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }

            stop();
            move(-.4);
            safeSleep(1500);
            stop();

            rotateTowards(getYaw() + Math.PI/4);
            move(.5);
            safeSleep(2000);
            stop();

            if (parentTask != null){
                parentTask.setPause(false);
            }

            Log.d(TAG, ObstacleDetectedTask.class.getSimpleName() + ".end");
            return null;
        }

        @Override
        public void abort() {
            shouldAbort = true;
            cancel(true);
        }
    }

    /***
     * Basic task for rotating towards to X, Y coordinates. Use setDestination() to set where to go
     * or override with params as part of the execute... call
     */
    public class RotateTowardsTask extends BaseTask {
        private int x = 0;
        private int y = 0;

        public void setDestination(int x, int y){
            this.x = x;
            this.y = y;
        }

        @Override
        protected Void doInBackground(String... params) {
            Log.d(TAG, RotateTowardsTask.class.getSimpleName() + ".start");
            try {
                if (params != null && params.length == 2) {
                    //parse coordinates from params
                    setDestination(Integer.parseInt(params[0]), Integer.parseInt(params[1]));
                }

                for (int i = 0; i < 5; i++) {
                    if (shouldAbort) break;
                    if (pause) doPause();
                    Log.d(TAG, RotateTowardsTask.class.getSimpleName() + ".running...to grid " + x + "x" + y);
                    Thread.sleep(1000);
                }

            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Log.d(TAG, RotateTowardsTask.class.getSimpleName() + ".end");
            return null;
        }

        @Override
        public void abort() {
            shouldAbort = true;
            cancel(true);
        }

    }

    /*
     * FullRotationTask does a 360 rotation at half power to scan for buckets
     * TODO implement a scan for buckets and record locations
     */

    protected class FullRotationTask extends BaseTask {

        @Override
        public void abort() {
            Log.d(TAG, "FullRotationTask: ABORT");
            shouldAbort = true;
            cancel(true);
        }

        @Override
        protected Void doInBackground(String... strings) {
            double startYaw = getYaw();
            double targetYaw = startYaw;

            Log.d(TAG, "FullRotationTask: turning");

            while(Math.abs(signedDeltaAngle(startYaw, getYaw())) < 2.0) {
                Log.d(TAG, "FullRotationTask: SmallestAngleDiff: " + signedDeltaAngle(getYaw(), targetYaw));
                stop();
                safeSleep(200);
                turn(.4);
                safeSleep(200);
            }
            turn(.2);
            while(Math.abs(signedDeltaAngle(getYaw(), targetYaw)) > 0.2) {
                Log.d(TAG, "FullRotationTask: SmallestAngleDiff: " + signedDeltaAngle(getYaw(), targetYaw));
                safeSleep(10);
            }
            Log.d(TAG, "FullRotationTask: stopping motors");
            stop();
            return null;
        }
    }


    /*
     **********************************************************************************************
     * FSM class executes the stack of tasks
     **********************************************************************************************
     */
    public static class FSM extends LinkedList<BaseTask> {
        public static final String TAG = FSM.class.getSimpleName();

        private BaseTask currentTask;
        private boolean paused = true;

        public void startFsm(){
            while (size() > 0){
                if (!paused) {
                    if (currentTask == null || isDone(currentTask)) {
                        currentTask = pollLast();
                        if (currentTask != null) {
                            currentTask.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
                        }
                    }

                    if (size() == 0) {
                        //wait for last task to finish
                        while (!isDone(currentTask)) {
                            try {
                                Thread.sleep(100);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }
                        currentTask = null;
                    }
                }
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            Log.d(TAG, "FSM Finished!");
        }

        private boolean isDone(BaseTask currentTask) {
            return currentTask.getStatus() == AsyncTask.Status.FINISHED || currentTask.isCancelled();
        }

        private BaseTask interrupTask;
        public void interruptCurrentTaskWith(BaseTask newTask, String newTaskParams){
            if (currentTask != null && (interrupTask == null || isDone(interrupTask))){
                interrupTask = newTask;
                interrupTask.parentTask = currentTask;
                interrupTask.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR, newTaskParams);
                currentTask.setPause(true);
            }
        }

        public void abortCurrentTask(){
            if (currentTask != null){
                currentTask.abort();
            }
        }

        public void abortCurrentTaskAndKillTasks(){
            if (currentTask != null){
                currentTask.abort();
            }
            while (this.poll()!=null){}
        }

        public void setPause(boolean pause) {
            paused = pause;
            if(currentTask != null) {
                try {
                    currentTask.doPause();
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

        public boolean isPaused() {
            return paused;
        }
    }
    //end of FSM class

}
