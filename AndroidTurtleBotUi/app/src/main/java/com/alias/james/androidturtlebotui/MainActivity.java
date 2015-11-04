/*
 * File:   MainActivity.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class manages the control flow of the app.
 *
 * Last Modified 11/03/2015
 */

package com.alias.james.androidturtlebotui;

import android.graphics.BitmapFactory;
import android.graphics.Point;
import android.os.AsyncTask;
import android.support.v4.app.FragmentTransaction;
import android.os.Bundle;
import android.support.v4.app.FragmentActivity;
import android.view.Display;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.Toolbar;

public class MainActivity extends FragmentActivity implements Options
{

    private MapFrag mapFrag = new MapFrag();
    private JoyFrag joyFrag = new JoyFrag();
    private Menu m_menu;

    /**
     * Sets up the fragment manager and main layout.
     *
     * @param savedInstanceState
     */
    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        //getSupportFragmentManager().beginTransaction().add(R.id.fragment_container, mapFrag).commit();
        getSupportFragmentManager().beginTransaction().add(R.id.fragment_container, joyFrag).commit();
    }


    /**
     * Initializes the menu.
     *
     * @param menu
     * @return
     */
    @Override
    public boolean onCreateOptionsMenu(Menu menu)
    {
        // Inflate the menu; this adds items to the action bar if it is present.

        getMenuInflater().inflate(R.menu.menu_main, menu);
        m_menu = menu;

        return super.onCreateOptionsMenu(menu);
    }


    /**
     * Handles items selected from the options menu.
     *
     * @param item
     * @return
     */
    @Override
    public boolean onOptionsItemSelected(MenuItem item)
    {
        switch (item.getItemId() )
        {
            case R.id.map:
                System.out.println("\"Map\" selected");
                return true;
            case R.id.joystick:
                System.out.println("\"Joystick\" selected");
                onJoySelected(0);
                break;
            case R.id.video:
                System.out.println("\"Video\" selected");
                onImgSelected(0);
                return true;
            case R.id.help:
                System.out.println("\"Help\" selected");
                return true;
            default:
                return super.onOptionsItemSelected(item);
        }
        return false;//FIXME: hardcoded return statement stub.
    }


    /**
     * Handles the fragment transactions (i.e. swaps in and out the different fragments).
     *
     * @param position
     */
    @Override
    public void onImgSelected(int position)
    {
        System.out.println("^^^swapping; postion: " + position);
        VideoFrag secondFrag = new VideoFrag();


        secondFrag.setArguments(getIntent().getExtras());

        FragmentTransaction transaction = getSupportFragmentManager().beginTransaction();

        transaction.replace(R.id.fragment_container, secondFrag);
        transaction.addToBackStack(null); //this enables the back button
        transaction.commit();

    }


    @Override
    public void onJoySelected(int position)
    {
        System.out.print("^^^swapping; position: " + position);
        JoyFrag thirdFrag = new JoyFrag();

        thirdFrag.setArguments(getIntent().getExtras());

        FragmentTransaction transaction = getSupportFragmentManager().beginTransaction();

        transaction.replace(R.id.fragment_container, thirdFrag);
        transaction.addToBackStack(null);
        transaction.commit();
    }


    public String toString()
    {
        return "***METHOD STUB***";
    }


} // End of MainActivity