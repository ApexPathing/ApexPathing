package com.apexpathing.util

abstract class Controller {
    var goal: Double = 0.0

    fun setGoal(newGoal:Double) {
        this.goal = newGoal
    }

    abstract fun calculate(currentPosition:Double):Double
}