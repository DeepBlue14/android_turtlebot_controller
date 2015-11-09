/*
 * File:   Constants.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class contains constants (such as voice commands).
 *
 *
 *
 * Last Modified 11/08/2015
 */


package com.alias.james.androidturtlebotui;

/**
 * Created by root on 11/8/15.
 */
public final class Constants
{
    private static final String ROBOT_NAME = "Genevieve";
    private static final String START = "Initiate activation sequence";
    private static final String END = "Terminate all processes";
    private static final String[] DIRECTIONS = {"Forward", "Rotate left", "Rotate right", "Backward", "Stop"};
    private static final String[] E_STOP = {"Stop"};//do .contains(), not .equal()
}