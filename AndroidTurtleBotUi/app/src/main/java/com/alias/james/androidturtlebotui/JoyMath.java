/*
 * File:
 * Author:
 * Email:
 * File Description:
 *
 * Last Modified:
 */

package com.alias.james.androidturtlebotui;

import android.graphics.PointF;

/**
 * Created by root on 11/25/15.
 */
public class JoyMath {




    public double computeAngleInRad(PointF centerPnt, PointF touchPnt)
    {
        double opposite = Math.abs(centerPnt.y - touchPnt.y);
        double hypotenuse = Math.sqrt((opposite * opposite) + (Math.abs(centerPnt.x - touchPnt.x) * Math.abs(centerPnt.x - touchPnt.x)) );
        double angle = Math.sin(opposite/hypotenuse);

        return angle;
    }


    //public

} // END of class JoyMath