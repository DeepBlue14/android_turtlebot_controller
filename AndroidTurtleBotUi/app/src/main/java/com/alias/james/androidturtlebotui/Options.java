/*
 * File:   Options.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This interface allows the fragments to communicate with the main activity
 *                   via callback methods.
 *
 *
 * Last Modified 10/19/2015
 */


package com.alias.james.androidturtlebotui;

/**
 * Created by root on 10/19/15.
 */
public interface Options {

    /**
     * Method prototype for callback.
     *
     * @param position
     */
    public void onImgSelected(int position);
}
