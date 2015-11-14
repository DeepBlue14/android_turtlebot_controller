/*
 * File:   JoyFrag.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class contains a virtual joystick which the user can use to drive the
 *                   robot directly.  Above the joystick is a small window in which will be
 *                   displayed a portion of the map where the robot is, or the video feed (this is
 *                   up do the user, and can be changed dynamically).
 *
 * Last Modified 11/03/2015
 */

package com.alias.james.androidturtlebotui;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Point;
import android.graphics.drawable.BitmapDrawable;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageView;

import org.opencv.core.Mat;

/**
 * Created by root on 10/29/15.
 */
public class JoyFrag extends Fragment implements ImageView.OnTouchListener
{

    private Paint paint;
    private static ImageView viewImageView;
    private ImageView joyImageView;
    private Bitmap cameraBitmap;
    private Bitmap mapBitmap;
    private Bitmap joyBitmap;
    private Bitmap compassBitmap;
    private Canvas mapCanvas;
    private Canvas joyCanvas;
    private Button mapBtn;
    private Button videoBtn;
    private boolean isDriving = false;
    private boolean dreamIsOn = true; /** Use modified DREAM version instead of static placement. */
    private boolean camNotMap = true; /** Display video feed, rather then map view */
    private float compassX;
    private float compassY;
    private int counter = 0; //send msg every x iterations
    private String tmpLocCmd = new String("000|000");

    @Override
    public void onCreate(Bundle savedInstanceState)
    {

        //http://stackoverflow.com/questions/4207067/how-to-implement-touch-listener-on-image

        super.onCreate(savedInstanceState);
    }


    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState)
    {
        System.out.println("^^^@ JoyFrag::onCreateView");
        View view = inflater.inflate(R.layout.joy_layout, container, false);

        paint = new Paint();
        viewImageView = (ImageView) view.findViewById(R.id.user_view);
        joyImageView = (ImageView) view.findViewById(R.id.joystick);

        if(camNotMap)
        {
            Matrix matrix = new Matrix();
            matrix.postRotate(0);
            Bitmap tmpBitmap0 = BitmapFactory.decodeResource(getResources(), R.drawable.compass_mini);
            tmpBitmap0 = Bitmap.createBitmap(tmpBitmap0, 0, 0, tmpBitmap0.getWidth(), tmpBitmap0.getHeight(), matrix, true);
            viewImageView.setImageBitmap(tmpBitmap0);
        }
        else
        {
            //load part of the map where robot is at currently
            Matrix matrix = new Matrix();
            matrix.postRotate(0);
            Bitmap tmpBitmap0 = BitmapFactory.decodeResource(getResources(), R.drawable.james_world2);
            tmpBitmap0 = Bitmap.createBitmap(tmpBitmap0, 0, 0, 480, 640, matrix, true);
            viewImageView.setImageBitmap(tmpBitmap0);
        }


        Bitmap tmpBitmap1;
        if(dreamIsOn)
        {
            tmpBitmap1 = BitmapFactory.decodeResource(getResources(), R.drawable.compass2);
        }
        else
        {
            tmpBitmap1 = BitmapFactory.decodeResource(getResources(), R.drawable.dream_slate);
        }


        joyBitmap = tmpBitmap1.copy(Bitmap.Config.RGB_565, true);
        joyCanvas = new Canvas(joyBitmap);

        Bitmap tmpBitmap2 = BitmapFactory.decodeResource(getResources(), R.drawable.compass_mini);
        compassBitmap = tmpBitmap2.copy(Bitmap.Config.ARGB_8888, true);
        //TODO: canvas for compass?

        joyImageView.setImageDrawable(new BitmapDrawable(getResources(), joyBitmap));
        joyImageView.setBackgroundColor(Color.WHITE);
        joyImageView.setOnTouchListener(this);


        return view;
    }


    @Override
    public void onDestroyView()
    {
        super.onDestroyView();
    }


    @Override
    public void onDestroy()
    {
        super.onDestroy();
    }


    public String toString()
    {
        return "***METHOD STUB";
    }

    @Override
    public boolean onTouch(View v, MotionEvent event)
    {
        //System.out.println("^^^JoyFrag::onTouch(...) activiated: (" + event.getX() + ", " + event.getY() + ")");
        int action = event.getAction();

        switch (action & MotionEvent.ACTION_MASK)
        {
            case MotionEvent.ACTION_DOWN:
                counter = 0; //reset counter
                //System.out.println("^^^paint GREEN");
                tmpLocCmd = "000|000";
                LocCmdServer.setLocCmd("000|000");
                isDriving = true;

                if(dreamIsOn)
                {
                    joyCanvas.drawColor(Color.RED);
                    joyImageView.setBackgroundColor(Color.RED);

                    Matrix inverse = new Matrix();
                    joyImageView.getImageMatrix().invert(inverse);
                    float[] touchPoint = new float[] {event.getX(), event.getY()};
                    inverse.mapPoints(touchPoint);

                    joyCanvas.drawBitmap(compassBitmap, Math.round(touchPoint[0]) - (compassBitmap.getWidth()/2), Math.round(touchPoint[1]) - (compassBitmap.getHeight()/2), paint);
                    compassX = touchPoint[0];//event.getX();
                    compassY = touchPoint[1];//event.getY();
                }
                else
                {
                    joyImageView.setBackgroundColor(Color.RED);
                }

                joyImageView.invalidate(); //tells app to redraw view; also see onDraw();
                break;
            case MotionEvent.ACTION_MOVE:
                counter++;
                /*FIXME: these conditions are based on a square, but the compass is a circle*/

                /*if(event.getX() > ((compassX - compassBitmap.getWidth() / 2)) ||
                   event.getX() < ((compassX - compassBitmap.getWidth() / 2)) ||
                   event.getY() > ((compassY - compassBitmap.getHeight() / 2)) ||
                   event.getY() < ((compassY - compassBitmap.getHeight() / 2)))*/
                //System.out.println("^^^math: " + Math.abs(Math.abs(event.getY()) - Math.abs(compassY- compassBitmap.getHeight())));
                /*if( Math.abs(Math.abs(event.getY()) - Math.abs(compassY- compassBitmap.getHeight())) < 90 )
                {

                    joyCanvas.drawColor(Color.GREEN);
                    joyImageView.setBackgroundColor(Color.GREEN);
                    joyCanvas.drawBitmap(compassBitmap, compassX - (compassBitmap.getWidth() / 2), compassY - (compassBitmap.getHeight() / 2), paint);
                    joyImageView.invalidate();
                    if ( event.getX() > ((compassX - compassBitmap.getWidth() / 2)+75) &&
                        event.getY() > ((compassY - compassBitmap.getWidth() / 2)+75) ) {
                        //System.out.println("^^^ROTATE_LEFT?");
                        LocCmdServer.setLocCmd("333|333");
                    } else if ( event.getX() < ((compassX - compassBitmap.getWidth() / 2)-75) &&
                                event.getY() < ((compassY - compassBitmap.getWidth() / 2)-75)) {
                        //System.out.println("^^^MOVE_FORWARD?");
                        LocCmdServer.setLocCmd("111|111");
                    } else if ( event.getX() > ((compassX - compassBitmap.getWidth() / 2)+75) &&
                                event.getY() < ((compassY - compassBitmap.getWidth() / 2)-75)) {
                        //System.out.println("^^^ROTATE_RIGHT?");
                        LocCmdServer.setLocCmd("444|444");
                    } else if ( event.getX() < ((compassX - compassBitmap.getWidth() / 2)-75) &&
                                event.getY() > ((compassY - compassBitmap.getWidth() / 2)+75)) {
                        //System.out.println("^^^MOVE_BACKWARD?");
                        LocCmdServer.setLocCmd("222|222");
                    }
                }*/


                //System.out.println("^^^math: " + (event.getY()-230 - (compassY-compassBitmap.getHeight()/2))
                //                + ", " + ((compassY-compassBitmap.getHeight()/2)-event.getY()+230));
                //System.out.println("^^^math: " + (event.getX()-130 - (compassX-compassBitmap.getWidth()/2))
                //                + ", " + ((compassX-compassBitmap.getWidth()/2)-event.getX()+130));


                if((event.getY()-230 - (compassY-compassBitmap.getHeight()/2)) < -75)
                {
                    joyCanvas.drawColor(Color.GREEN);
                    joyImageView.setBackgroundColor(Color.GREEN);
                    joyCanvas.drawBitmap(compassBitmap, compassX - (compassBitmap.getWidth() / 2), compassY - (compassBitmap.getHeight() / 2), paint);
                    joyImageView.invalidate();
                    //System.out.println("^^^FOREWARD");
                    tmpLocCmd = "111|111";
                }
                else if((event.getY()-230 - (compassY-compassBitmap.getHeight()/2)) > 75)
                {
                    joyCanvas.drawColor(Color.GREEN);
                    joyImageView.setBackgroundColor(Color.GREEN);
                    joyCanvas.drawBitmap(compassBitmap, compassX - (compassBitmap.getWidth() / 2), compassY - (compassBitmap.getHeight() / 2), paint);
                    joyImageView.invalidate();
                    //System.out.println("^^^BACKARD");
                    tmpLocCmd = "222|222";
                }
                else if( (event.getX()-130 - (compassX-compassBitmap.getWidth()/2)) < -75)
                {
                    joyCanvas.drawColor(Color.GREEN);
                    joyImageView.setBackgroundColor(Color.GREEN);
                    joyCanvas.drawBitmap(compassBitmap, compassX - (compassBitmap.getWidth() / 2), compassY - (compassBitmap.getHeight() / 2), paint);
                    joyImageView.invalidate();
                    //System.out.println("^^^LEFT");
                    tmpLocCmd = "333|333";
                }
                else if( (event.getX()-130 - (compassX-compassBitmap.getWidth()/2)) > 75)
                {
                    joyCanvas.drawColor(Color.GREEN);
                    joyImageView.setBackgroundColor(Color.GREEN);
                    joyCanvas.drawBitmap(compassBitmap, compassX - (compassBitmap.getWidth() / 2), compassY - (compassBitmap.getHeight() / 2), paint);
                    joyImageView.invalidate();
                    //System.out.println("^^^RIGHT");
                    tmpLocCmd = "444|444";
                }
                else
                {
                    joyCanvas.drawColor(Color.RED);
                    joyImageView.setBackgroundColor(Color.RED);
                    joyCanvas.drawBitmap(compassBitmap, compassX - (compassBitmap.getWidth() / 2), compassY - (compassBitmap.getHeight() / 2), paint);
                    joyImageView.invalidate();
                    //System.out.println("^^^STOP");
                    tmpLocCmd = "000|000";
                }

                if(counter == 70)
                {
                    System.out.println("^^^sending msg");
                    LocCmdServer.setLocCmd(tmpLocCmd);
                    counter = 0;
                }

                break;
            case MotionEvent.ACTION_UP:
                //System.out.println("^^^paint WHITE");
                LocCmdServer.setLocCmd("000|000"); //stop the robot
                isDriving = false;

                if(dreamIsOn)
                {
                    joyCanvas.drawColor(Color.WHITE);
                    joyBitmap = Bitmap.createScaledBitmap(joyBitmap, joyBitmap.getWidth()-(joyBitmap.getWidth()/100), joyBitmap.getHeight()-(joyBitmap.getHeight()/100), false);
                    joyCanvas.drawBitmap(joyBitmap, 0, 0, paint);
                    joyImageView.setBackgroundColor(Color.WHITE);
                }
                else
                {
                    joyImageView.setBackgroundColor(Color.WHITE);
                }

                joyImageView.invalidate(); //tells app to redraw view; also see onDraw();
                break;
        }

        return true; //FIXME: change this return value. see http://stackoverflow.com/questions/15799839/motionevent-action-up-not-called
    }


    private Point computeCompassSize()
    {
        Point tmpPoint = new Point(-1, -1);
        tmpPoint.x = joyImageView.getWidth();
        tmpPoint.y = joyImageView.getHeight();

        return tmpPoint;
    }


    public static void setViewImageView(ImageView viewImageView) {
        JoyFrag.viewImageView = viewImageView;
    }


    public static ImageView getViewImageView()
    {
        return viewImageView;
    }



}//end of class JoyFrag
