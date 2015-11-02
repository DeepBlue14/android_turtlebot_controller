/*
 * File:   MapGestures.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This helper class contains the logic of the gestures.
 *
 * Last Modified 10/22/2015
 */


package com.alias.james.androidturtlebotui;

/**
 * Created by root on 10/22/15.
 */
public class MapGestures {

    public enum Gestures {
        ZOOM_OUT,
        ZOOM_IN,
        ROTATE_CLOCKWISE,
        ROTATE_COUNTERCLOCKWISE,
        SELECT_ROBOT,
        DESELECT_ROBOT,
        DRAW_FOOTPRINT,
        COMPLETE_PATH
    }

    public MapGestures() {
        ;
    }


    //public Gestures evalTouchEvent(int numOfFingers, )


    public String toString() {
        return "***METHOD STUB***";
    }
}