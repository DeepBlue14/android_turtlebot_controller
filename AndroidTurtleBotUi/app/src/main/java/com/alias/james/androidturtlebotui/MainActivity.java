/*
 * File:   MainActivity.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class manages the control flow of the app.
 *
 * Created 10/14/2015 at 7:30pm
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

public class MainActivity extends FragmentActivity implements Options {

    private MapFrag m_mapFrag = new MapFrag();
    private Menu m_menu;

    /**
     * Sets up the fragment manager and main layout.
     *
     * @param savedInstanceState
     */
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        getSupportFragmentManager().beginTransaction().add(R.id.fragment_container, m_mapFrag).commit();
    }


    /**
     * Initializes the menu.
     *
     * @param menu
     * @return
     */
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
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
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId() ) {
            case R.id.map:
                System.out.println("\"Map\" selected");
                return true;
            case R.id.video:
                System.out.println("\"Video\" selected");
                return true;
            case R.id.help:
                System.out.println("\"Help\" selected");
                return true;
            default:
                return super.onOptionsItemSelected(item);
        }
    }


    /**
     * Handles the fragment transactions (i.e. swaps in and out the different fragments).
     *
     * @param position
     */
    @Override
    public void onImgSelected(int position)
    {
        System.out.println("swapping");
        VideoFrag secondFrag = new VideoFrag();

        secondFrag.setArguments(getIntent().getExtras());

        FragmentTransaction transaction = getSupportFragmentManager().beginTransaction();

        transaction.replace(R.id.fragment_container, secondFrag);
        transaction.addToBackStack(null); //this enables the back button
        transaction.commit();

    }


    /*public void computeDisplaySize() {
        Display display = getWindowManager().getDefaultDisplay();
        Point size = new Point();
        display.getSize(size);
        int width = size.x;
        int height = size.y;
    }*/


    public String toString()
    {
        return "***METHOD STUB***";
    }


} // End of MainActivity