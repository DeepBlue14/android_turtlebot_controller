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

/**
 * Created by root on 10/29/15.
 */
public class JoyFrag extends Fragment implements ImageView.OnTouchListener
{

    private Paint paint;
    private ImageView viewImageView;
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
        mapBtn = (Button) view.findViewById(R.id.map_button);
        videoBtn = (Button) view.findViewById(R.id.video_button);

        Bitmap tmpBitmap0 = BitmapFactory.decodeResource(getResources(), R.drawable.turtlebot);
        viewImageView.setImageBitmap(tmpBitmap0);

        Bitmap tmpBitmap1;
        if(dreamIsOn)
        {
            tmpBitmap1 = BitmapFactory.decodeResource(getResources(), R.drawable.compass2);
        }
        else
        {
            tmpBitmap1 = BitmapFactory.decodeResource(getResources(), R.drawable.dream_slate);
        }


        joyBitmap = tmpBitmap1.copy(Bitmap.Config.ARGB_8888, true);
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
        System.out.println("^^^JoyFrag::onTouch(...) activiated: (" + event.getX() + ", " + event.getY() + ")");
        int action = event.getAction();

        switch (action & MotionEvent.ACTION_MASK)
        {
            case MotionEvent.ACTION_DOWN:
                //System.out.println("^^^paint GREEN");
                isDriving = true;

                if(dreamIsOn)
                {
                    joyCanvas.drawColor(Color.GREEN);
                    joyImageView.setBackgroundColor(Color.GREEN);
                    joyCanvas.drawBitmap(compassBitmap, event.getX() - compassBitmap.getWidth() / 4, event.getY() - compassBitmap.getHeight() / 4, paint);
                }
                else
                {
                    joyImageView.setBackgroundColor(Color.GREEN);
                }

                joyImageView.invalidate(); //tells app to redraw view; also see onDraw();
                break;
            case MotionEvent.ACTION_MOVE:
                ;
                break;
            case MotionEvent.ACTION_UP:
                //System.out.println("^^^paint WHITE");
                isDriving = false;

                if(dreamIsOn)
                {
                    joyCanvas.drawColor(Color.WHITE);
                    joyBitmap = Bitmap.createScaledBitmap(joyBitmap, joyBitmap.getWidth()-(joyBitmap.getWidth()/100), joyBitmap.getHeight()-(joyBitmap.getHeight()/100), false);
                    compassBitmap = Bitmap.createScaledBitmap(compassBitmap, compassBitmap.getWidth() - 1, compassBitmap.getHeight() - 1, false);
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



}//end of class JoyFrag
