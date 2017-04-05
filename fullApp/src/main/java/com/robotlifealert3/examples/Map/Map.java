package com.robotlifealert3.examples.Map;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Point;
import android.util.AttributeSet;
import android.util.Log;
import android.view.Display;
import android.view.View;
import android.view.WindowManager;

public class Map extends View {
    private int[][] map;
    public static final int MAP_DIM = 50;
    private Paint black = new Paint();
    private Context mContext;
    public static final int TANGO_WIDTH = 1920;
    public static final int TANGO_HEIGHT = 1200;
    public static final int MAP_SCREEN_DIM = 1000;

    //Init Map//


    public Map(Context context) {
        super(context);
        mContext = context;
    }

    public Map(Context context, AttributeSet attrs) {
        super(context, attrs);
        mContext = context;
        black.setStyle(Paint.Style.FILL);
    }

    public int[][] getMap() {
        return map;
    }

    private void mapInit() {
        for(int i = 0; i < map.length; i++)
            for(int j = 0; j < map[i].length; j++)
                map[i][j] = 0;
    }

    public void setMap(int[][] map) {
        this.map = map;
    }

    public void draw(Canvas canvas, Paint paint, int i, int j) {
        canvas.drawCircle(MAP_SCREEN_DIM*i/50 + (TANGO_WIDTH-MAP_SCREEN_DIM)/2, TANGO_HEIGHT - (MAP_SCREEN_DIM*j/50 + (TANGO_HEIGHT-MAP_SCREEN_DIM+500)/2), 5, paint);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        Paint travelPaint = new Paint();
        travelPaint.setStyle(Paint.Style.FILL);
        travelPaint.setColor(Color.GREEN);

        Paint bucketPaint = new Paint();
        bucketPaint.setStyle(Paint.Style.FILL);
        bucketPaint.setColor(Color.RED);


        //int width = getScreenResolution().x;

        //This if the Map is not Null then draw the cricle of Pos i and j at size 4
        if (map != null) {
            for (int i = 0; i < map.length; i++) {
                for (int j = 0; j < map[i].length; j++) {
                    if (map[i][j] == 1) {
                        draw(canvas, travelPaint, i, j);
                    }
                    if (map[i][j] == 3) {
                        draw(canvas, bucketPaint, i, j);
                    }
                }
            }
        } else {
            Log.d("TEST", "map is null");
            //This runs in a loop remember, so once you let the tablet initialize then it will loop
            //back and you will no longer have this error.
        }
    }

    private Point getScreenResolution(){
        WindowManager wm = (WindowManager) mContext.getSystemService(Context.WINDOW_SERVICE);
        Display display = wm.getDefaultDisplay();
        Point size = new Point();
        display.getSize(size);
        return size;
    }

}