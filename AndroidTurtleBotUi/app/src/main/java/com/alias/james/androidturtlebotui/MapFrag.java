/*
 * File:   MapFrag.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class contains the map which the robot is traveling in.
 *
 * Reference: http://stackoverflow.com/questions/3058164/android-scrolling-an-imageview
 *
 * Signals: green = available
 *          yellow = selected; awaiting command
 *          red = executing command
 *
 *
 *
 * Last Modified 10/29/2015
 */

package com.alias.james.androidturtlebotui;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Point;
import android.graphics.drawable.BitmapDrawable;
import android.support.v4.app.Fragment;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.view.Display;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;
import android.widget.ImageView;

import java.util.ArrayList;
import java.util.logging.Handler;

/**
 * Created by root on 10/14/15.
 */
public class MapFrag extends Fragment implements ImageView.OnTouchListener
{

    private enum RobotColor
    {
        RED,
        YELLOW,
        GREEN
    }

    private enum RobotStatus
    {
        UNSELECTED,
        SELECTED,
        BUSY
    }

    private ImageView imageView; /**  */
    private Bitmap turtlebotBitmap;
    private Bitmap mapBitmap; /** Map of the location */
    private Canvas mapCanvas; /** This will be loaded from the bitmap */
    private Paint paint; /**  */
    private Handler handler; /**  */
    private boolean isRobotSelected = false; /**  */
    private float mapXPos = 0; /**  */
    private float mapYPos = 0; /**  */
    private int mapWidth = 0; /**  */
    private int mapHeight = 0; /**  */
    private int robotXPos = 240; /**  */
    private int robotYPos = 360; /**  */
    private int robotWidth = 0;
    private int robotHeight = 0;
    private Point displaySize; /**  */
    private ArrayList<Point> footPrintArrLst = new ArrayList<>();


    /**
     *
     *
     * @param savedInstanceState
     */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {

        computeDisplaySize();
        super.onCreate(savedInstanceState);
    }

    /**
     *
     *
     * @param inflater
     * @param container
     * @param savedInstanceState
     * @return
     */
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState)
    {

        System.out.println("^^^@ MapFrag::onCreateView");
        View view = inflater.inflate(R.layout.map_layout, container, false);

        Bitmap tmpBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.test_world);
        mapBitmap = tmpBitmap.copy(Bitmap.Config.ARGB_8888, true);
        mapWidth = mapBitmap.getWidth();
        mapHeight = mapBitmap.getHeight();

        mapCanvas = new Canvas(mapBitmap);

        paint = new Paint();

        //mapCanvas.drawCircle(200, 350, 10, paint);
        //mapCanvas.drawCircle(200, 600, 10, paint);
        //mapCanvas.drawCircle(500, 200, 10, paint);

        turtlebotBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.turtlebot);
        turtlebotBitmap = Bitmap.createScaledBitmap(turtlebotBitmap, 60, 60, true);
        robotWidth = turtlebotBitmap.getWidth();
        robotHeight = turtlebotBitmap.getHeight();
        paint.setColor(Color.RED);
        mapCanvas.drawBitmap(turtlebotBitmap, 240, 360, paint);

        imageView = (ImageView) view.findViewById(R.id.map_image);
        //imageView.setImageBitmap(mapBitmap);
        imageView.setImageDrawable(new BitmapDrawable(getResources(), mapBitmap));
        imageView.setOnTouchListener(this);


        return view;
    }


    /**
     *
     *
     * @param v
     * @param event
     * @return
     */
    @Override
    public boolean onTouch(View v, MotionEvent event)
    {

        int action = event.getAction();
        float fingerPressX0 = 0, fingerPressY0 = 0, fingerPressX1 = 0, fingerPressY1 = 0;

        switch (action & MotionEvent.ACTION_MASK)
        {
            case MotionEvent.ACTION_DOWN:

                if (!isRobotSelected && event.getPointerCount() == 1 && (event.getX()+120) < (robotXPos + 90) && (event.getX()+120) > (robotXPos - 90)
                        && (event.getY()-100) < (robotYPos + 90) && (event.getY()-100) > (robotYPos - 90) )
                {
                    isRobotSelected = true;
                    updateView();
                }
                else if (isRobotSelected && event.getPointerCount() == 1 && (event.getX()+120) < (robotXPos + 90) && (event.getX()+120) > (robotXPos - 90)
                        && (event.getY()-100) < (robotYPos + 90) && (event.getY()-100) > (robotYPos - 90) )
                {
                    isRobotSelected = false;
                    updateView();
                }
                else if (isRobotSelected && event.getPointerCount() == 1 && ( !((event.getX()+120) < (robotXPos + 90)) || !((event.getX()+120) > (robotXPos - 90))
                        || !((event.getY()-100) < (robotYPos + 90)) || !((event.getY()-100) > (robotYPos - 90)) ) )
                {
                    System.out.println("^^^drawing circle");
                    //mapCanvas.drawCircle(event.getX() + 120, event.getY() - 60, 5, paint);//offset of bitmap vs imageview
                    //imageView.invalidate();
                    footPrintArrLst.add(new Point((int) event.getX() + 120, (int)event.getY()-60) );
                    updateView();
                }
                break;
            case MotionEvent.ACTION_POINTER_DOWN:
                System.out.println("^^^ACTION_POINTER_DOWN");
                System.out.println("^^^# of fingers: " + event.getPointerCount());

                for(byte i = 0; i < event.getPointerCount(); i++)
                {
                    System.out.println("finger number: " + i + "position: " + event.getX(i));
                }

                break;
            case MotionEvent.ACTION_MOVE:
                //System.out.println("^^^ACTION_MOVE");

                //if robot is selected
                //updateView();

                //-------------------------------
                if (event.getPointerCount() == 2)
                {
                    final int historySize1 = event.getHistorySize();
                    for (int h = 0; h < historySize1; h++)
                    {
                        // historical point
                        float hx0 = event.getHistoricalX(0, h);
                        float hy0 = event.getHistoricalY(0, h);
                        float hx1 = event.getHistoricalX(1, h);
                        float hy1 = event.getHistoricalY(1, h);
                        // make historical point the start point for next loop iteration
                        fingerPressX0 = hx0;
                        fingerPressY0 = hy0;
                        fingerPressX1 = hx1;
                        fingerPressY1 = hy1;
                    }
                    // add distance from last historical point to event's point
                    float dx0 = (event.getX(0) - fingerPressX0);
                    float dy0 = (event.getX(0) - fingerPressY0);
                    float dx1 = (event.getX(1) - fingerPressX1);
                    float dy1 = (event.getY(1) - fingerPressY1);

                    if (dx1 < 100 && dy1 < 100 )
                    {
                        //System.out.println("dx: " + dx1 + " dy: " + dy1);

                        //the finger on right goes right; finger on left goes left
                        if ( (fingerPressX1 > fingerPressX0 && dx1 >= 0 && dx0 <= 0) ||
                                (fingerPressX0 > fingerPressX1 && dx0 >= 0 && dx1 <= 0) ||
                                (fingerPressY1 > fingerPressY0 && dy1 >= 0 && dy0 <= 0) ||
                                (fingerPressY0 > fingerPressY1 && dy0 >= 0 && dy1 <= 0) )
                        {

                            mapCanvas.drawColor(Color.BLACK);
                            mapBitmap = Bitmap.createScaledBitmap(mapBitmap, mapBitmap.getWidth()+10, mapBitmap.getHeight()+10, false);
                            turtlebotBitmap = Bitmap.createScaledBitmap(turtlebotBitmap, turtlebotBitmap.getWidth()+1, turtlebotBitmap.getHeight()+1, false);
                            mapWidth = mapBitmap.getWidth();
                            mapHeight = mapBitmap.getHeight();
                            mapCanvas.drawBitmap(mapBitmap, mapXPos, mapYPos, paint);
                            mapCanvas.drawBitmap(turtlebotBitmap, robotXPos, robotYPos, paint);
                            imageView.invalidate(); //tells app to redraw view; also see onDraw();

                        }
                        else if ( (fingerPressX1 > fingerPressX0 && dx1 >= 0 && dx0 >= 0) ||
                                (fingerPressX0 > fingerPressX1 && dx0 <= 0 && dx1 >= 0) ||
                                (fingerPressY1 > fingerPressY0 && dy1 <= 0 && dy0 >= 0) ||
                                (fingerPressY0 > fingerPressY1 && dy0 <= 0 && dy1 >= 0) )
                        {

                            mapCanvas.drawColor(Color.BLACK);
                            mapBitmap = Bitmap.createScaledBitmap(mapBitmap, mapBitmap.getWidth()-10, mapBitmap.getHeight()-10, false);
                            turtlebotBitmap = Bitmap.createScaledBitmap(turtlebotBitmap, turtlebotBitmap.getWidth()-1, turtlebotBitmap.getHeight()-1, false);
                            mapWidth = mapBitmap.getWidth();
                            mapHeight = mapBitmap.getHeight();
                            mapCanvas.drawBitmap(mapBitmap, mapXPos, mapYPos, paint);
                            mapCanvas.drawBitmap(turtlebotBitmap, robotXPos, robotYPos, paint);
                            imageView.invalidate(); //tells app to redraw view; also see onDraw();
                        }
                    }
                } // End of cases when fingercount = 1
                else if (event.getPointerCount() == 3)
                {
                    //rotate map around its center axis
                    mapCanvas.rotate(45, mapCanvas.getWidth()/2, mapCanvas.getHeight()/2);
                }

                //else move map
                if (event.getPointerCount() == 1)
                {
                    float distanceSum = 0;
                    final int historySize = event.getHistorySize();
                    for (int h = 0; h < historySize; h++)
                    {
                        // historical point
                        float hx = event.getHistoricalX(0, h);
                        float hy = event.getHistoricalY(0, h);
                        // distance between startX,startY and historical point
                        float dx = (hx - fingerPressX1);
                        float dy = (hy - fingerPressY1);
                        distanceSum += Math.sqrt(dx * dx + dy * dy);
                        // make historical point the start point for next loop iteration
                        fingerPressX1 = hx;
                        fingerPressY1 = hy;
                    }
                    // add distance from last historical point to event's point
                    float dx = (event.getX(0) - fingerPressX1);
                    float dy = (event.getY(0) - fingerPressY1);
                    distanceSum += Math.sqrt(dx * dx + dy * dy);

                    if (dx < 100 && dy < 100 )
                    {
                        mapCanvas.drawColor(Color.BLACK);
                        mapBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.test_world);
                        mapBitmap = Bitmap.createScaledBitmap(mapBitmap, mapWidth, mapHeight, false);
                        mapCanvas.drawBitmap(mapBitmap, (mapXPos += dx), (mapYPos += dy), paint);
                        mapCanvas.drawBitmap(turtlebotBitmap, (robotXPos += dx), (robotYPos += dy), paint);
                        //mapCanvas.drawBitmap(turtlebotBitmap, (robotXPos += 10), robotYPos, paint);
                        imageView.invalidate(); //tells app to redraw view; also see onDraw();
                        //mapCanvas.drawCircle(event.getX()+(dx*10), event.getY()+(dy*5), 2, paint);
                    }
                }

                break;
            case MotionEvent.ACTION_UP:
                //System.out.println("^^^ACTION_UP");
                break;
            case MotionEvent.ACTION_POINTER_UP:
                System.out.println("^^^ACTION_POINTER_UP");
                break;
            default:
                //System.out.println("^^^default");
        }


        return true;
    }


    /**
     *
     *
     * @param press
     * @param release
     * @param margin
     * @return
     */
    private boolean isWithinMargin(float press, float release, float margin)
    {
        if(Math.abs(press - release) - margin < 0)
        {
            return true;
        }
        else {
            return false;
        }
    }

    //FIXME: this method is NOT deleting the old bitmap--it is merely drawing a new one over it!
    public void updateView()
    {
        //Bitmap turtlebotBitmap;
        if(isRobotSelected)
        {
            //System.out.println("^^^Robot is selected!");
            turtlebotBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.turtlebot2);
        }
        else
        {
            //System.out.println("^^^Robot is NOT selected!");
            turtlebotBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.turtlebot);
        }

        turtlebotBitmap = Bitmap.createScaledBitmap(turtlebotBitmap, 60, 60, true);

        mapCanvas.drawColor(Color.BLACK);
        mapBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.test_world);
        mapCanvas.drawBitmap(mapBitmap, mapXPos, mapYPos, paint);
        mapCanvas.drawBitmap(turtlebotBitmap, (robotXPos += 10), robotYPos, paint);
        for(int i = 0; i < footPrintArrLst.size(); i++)
        {
            mapCanvas.drawCircle(footPrintArrLst.get(i).x, footPrintArrLst.get(i).y, 5, paint);//offset of bitmap vs imageview
        }
        imageView.invalidate(); //tells app to redraw view; also see onDraw();
    }


    private void rotateRobot(int degrees)
    {
        Matrix matrix = new Matrix();
        matrix.postRotate(degrees);
        turtlebotBitmap = Bitmap.createBitmap(turtlebotBitmap, 0, 0, turtlebotBitmap.getWidth(), turtlebotBitmap.getHeight(), matrix, true);
    }


    /**
     *
     */
    public void computeDisplaySize()
    {
        WindowManager wm = (WindowManager) getContext().getSystemService(Context.WINDOW_SERVICE);
        Display display = wm.getDefaultDisplay();
        displaySize = new Point();
        display.getSize(displaySize);
    }


    @Override
    public void onDestroyView() {
        super.onDestroyView();
    }


    @Override
    public void onDestroy() {
        super.onDestroy();
    }


    /**
     *
     *
     * @param imageView
     */
    public void setImageView(ImageView imageView)
    {
        imageView = imageView;
    }


    /**
     *
     *
     * @return
     */
    public ImageView getImageView()
    {
        return imageView;
    }


    public void setMapBitmap(Bitmap mapBitmap)
    {
        this.mapBitmap = mapBitmap;
    }


    public Bitmap getMapBitmap()
    {
        return mapBitmap;
    }


    public void setMapCanvas(Canvas mapCanvas)
    {
        this.mapCanvas = mapCanvas;
    }


    public Canvas getMapCanvas()
    {
        return mapCanvas;
    }


    public void setPaint(Paint paint)
    {
        this.paint = paint;
    }


    public Paint getPaint()
    {
        return paint;
    }


    public void setHandler(Handler handler)
    {
        this.handler = handler;
    }


    public Handler getHandler()
    {
        return handler;
    }


    public void setIsRobotSelected(boolean isRobotSelected)
    {
        this.isRobotSelected = isRobotSelected;
    }


    public boolean getIsRobotSelected()
    {
        return isRobotSelected;
    }


    public void setMapXPos(float mapXPos)
    {
        this.mapXPos = mapXPos;
    }


    public float getMapXPos()
    {
        return mapXPos;
    }


    public void setMapYPos(float mapYPos)
    {
        this.mapYPos = mapYPos;
    }


    public float getMapYPos()
    {
        return mapYPos;
    }


    public void setMapWidth(int mapWidth)
    {
        this.mapWidth = mapWidth;
    }


    public int getMapWidth()
    {
        return mapWidth;
    }


    public void setMapHeight(int mapHeight)
    {
        this.mapHeight = mapHeight;
    }


    public int getMapHeight()
    {
        return mapHeight;
    }


    public void setRobotXPos(int robotXPos)
    {
        this.robotXPos = robotXPos;
    }


    public int getRobotXPos()
    {
        return robotXPos;
    }


    public void setRobotYPos(int robotYPos)
    {
        this.robotYPos = robotYPos;
    }


    public int getRobotYPos()
    {
        return robotYPos;
    }


    public void setRobotWidth(int robotWidth)
    {
        this.robotWidth = robotWidth;
    }


    public int getRobotWidth()
    {
        return robotWidth;
    }


    public void setRobotHeight(int robotHeight)
    {
        this.robotHeight = robotHeight;
    }


    public int getRobotHeight()
    {
        return robotHeight;
    }


    public void setDisplaySize(Point displaySize)
    {
        this.displaySize = displaySize;
    }


    public Point getDisplaySize()
    {
        return displaySize;
    }


    /**
     *
     *
     * @return
     */
    public String toString()
    {
        return "***METHOD STUB***";
    }


} // End of class MapFrag