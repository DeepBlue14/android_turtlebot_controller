/*
 * File:   MainActivity.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class handles administrator login.  After logging in, the admin is able
 *                   to modify parameters (such as the expected IP address of the robot server)
 *                   which a regular user would is denied access to.
 *
 * References:
 *
 * Last Modified: 8/11/2015
 */

package com.alias.james.androidturtlebotui;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.view.LayoutInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.EditText;

public class Login
{
    private AlertDialog.Builder loginDialog; /** Is the dialog which allows the user to gain administrator privileges. */
private MenuItem adminCb; /** Is the checkbox which is checked if the current user is logged in as administrator. */
private static boolean userIsAdmin = false;


    /**
     * Initializes the dialog.
     *
     * @param activity
     * @param layoutInflater
     */
    public void initLoginDialog(Activity activity, LayoutInflater layoutInflater)
    {
        LayoutInflater inflater = layoutInflater;
        final View view = inflater.inflate(R.layout.login, null);

        loginDialog = new AlertDialog.Builder(activity)
                .setTitle("Frankenscooter UI")
                .setMessage("Welcome to Frankenscooter UI")
                .setView(view)
                .setPositiveButton(android.R.string.ok, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {

                        String userNameStr = ((EditText) view.findViewById(R.id.username)).getText().toString();
                        String passwordStr = ((EditText) view.findViewById(R.id.password)).getText().toString();

                        if (userNameStr.equals("h") && passwordStr.equals("r")) {
                            if (adminCb.isChecked() ) {
                                setUserIsAdmin(false);
                                adminCb.setChecked(false); // log out
                            } else {
                                setUserIsAdmin(true);
                                adminCb.setChecked(true); // log in
                            }
                        }


                    }
                })
                .setNegativeButton(android.R.string.cancel, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        System.out.println("\"Cancel\" alert button selected");
                    }
                })
                .setIcon(android.R.drawable.ic_dialog_alert);
        //.show();
    }


    /**
     * Initializes the checkbox object and displays the login dialog to the user.
     *
     * @param checkbox
     */
    public void show(MenuItem checkbox)
    {
        adminCb = checkbox;
        loginDialog.show();
    }


    /**
     * Mutator.
     *
     * @param userIsAdmin
     */
    public static void setUserIsAdmin(boolean userIsAdmin)
    {
        Login.userIsAdmin = userIsAdmin;
    }


    /**
     * Accessor.
     *
     * @return
     */
    public static boolean getUserIsAdmin()
    {
        return userIsAdmin;
    }


    /**
     * TODO: implement.
     *
     * @return
     */
    public String toString()
    {
        return "^^^*** METHOD STUB ***^^^";
    }



} // End of class Login