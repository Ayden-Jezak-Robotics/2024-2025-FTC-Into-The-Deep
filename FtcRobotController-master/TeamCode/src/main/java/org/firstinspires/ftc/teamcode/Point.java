package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

public class Point {
    private double _x;
    private double _y;

    public Point(double x, double y) {
        this._x = x;
        this._y = y;
    }

    public double getX() {
        return _x;
    }

    public double getY() {
        return _y;
    }

    public void setX(double x) {
        this._x = x;
    }

    public void setY(double y) {
        this._y = y;
    }

    @NonNull
    @Override
    public String toString() {
        return "(" + _x + ", " + _y + ")";
    }
}