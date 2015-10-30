/*
 * File:   JoyFrag.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class contains a virtual joystick which the user can use to drive the
 *                   robot directly.  Above the joystick is a small window in which will be
 *                   displayed a portion of the map where the robot is, or the video feed (this is
 *                   up do the user, and can be changed dynamically).
 *
 * Last Modified 10/29/2015
 */

package com.alias.james.androidturtlebotui;

import android.graphics.Bitmap;
import android.graphics.Canvas;
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
public class JoyFrag extends Fragment
{

    private ImageView viewImageView;
    private ImageView joyImageView;
    private Bitmap cameraBitmap;
    private Bitmap mapBitmap;
    private Bitmap joyBitmap;
    private Canvas mapCanvas;
    private Canvas joyCanvas;
    private Button mapBtn;
    private Button videoBtn;

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
        View view = inflater.inflate(R.layout.map_layout, container, false);
        viewImageView = (ImageView) view.findViewById(R.id.user_view);
        joyImageView = (ImageView) view.findViewById(R.id.joystick);
        mapBtn = (Button) view.findViewById(R.id.map_button);
        videoBtn = (Button) view.findViewById(R.id.video_button);



        return null;
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
}
