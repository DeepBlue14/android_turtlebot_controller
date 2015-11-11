/*
 * File:   CamClient.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class contains a thread which will receive the video feed from the robot.
 *
 *
 * Reference: http://developer.android.com/intl/ko/training/multiple-threads/communicate-ui.html
 *
 *
 * Last Modified 11/08/2015
 */


package com.alias.james.androidturtlebotui;

import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Handler;
import android.widget.ImageView;

/**
 * Created by root on 11/6/15.
 */
public class CamClient implements Runnable
{
    private Handler handler;
    private ImageView cameraView;
    private Bitmap bitmap;
    private Resources res;

    @Override
    public void run()
    {
        bitmap = BitmapFactory.decodeResource(res, R.drawable.turtlebot);
        handler.postDelayed(this, 1000);
    }


} // End of class CamClient