/*
From everpolate.js --> https://github.com/BorisChumichev/everpolate/tree/master/lib
The MIT License (MIT)

Copyright (c) 2015 Boris Chumichev

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.*/

/**
 * constructor for Interpolator "class"
 *
 * @param xValues array of values representing existing x-values
 * @param yValues array of values representing existing y-values
 * @param xyValues 2-dimensional array representing known values of the function, so that f(xValues[i], yValues[j]) = xyValues[i][j]
 * @constructor
 */
function Interpolator(xValues, yValues, xyValues) {
    var thiz = this;

    /**
     * bilinear interpolates values using the arrays given with the constructor.
     * uses bilinear interpolation algorithm described at https://en.wikipedia.org/wiki/Bilinear_interpolation
     * (1) interpolate in x direction, (2) interpolate in y direction
     *
     * @param x
     * @param y
     * @return {Array}
     */
    thiz.interpolateBilinear = function (x, y) {
        var interpolX = [];
        xyValues.forEach(function (values) {
            interpolX.push(evaluatePolynomial(x, xValues, values));
        });
        return evaluatePolynomial(y, yValues, interpolX);
    };

    thiz.interpolateBilinearArray = function(xArray, yArray) {
        var result = [];
        yArray.forEach(function (yVal) {
            var row = [];
            xArray.forEach(function (xVal) {
                row.push(thiz.interpolateBilinear(xVal, yVal))
            });
            result.push(row);
        });
        return result;
    };

    /**
     * Evaluates interpolating polynomial at the set of numbers
     * or at a single number for the function y=f(x)
     *
     * @param {Number|Array} pointsToEvaluate     number or set of numbers
     *                                            for which polynomial is calculated
     * @param {Array} functionValuesX             set of distinct x values
     * @param {Array} functionValuesY             set of distinct y=f(x) values
     * @returns {Array}                           interpolating polynomial
     */
    function evaluatePolynomial(pointsToEvaluate, functionValuesX, functionValuesY) {
        var results = []
        pointsToEvaluate = makeItArrayIfItsNot(pointsToEvaluate)
        // evaluate the interpolating polynomial for each point
        pointsToEvaluate.forEach(function (point) {
            results.push(nevillesIteratedInterpolation(point, functionValuesX, functionValuesY))
        })
        return results
    };

    /**
     * Neville's Iterated Interpolation algorithm implementation
     * http://en.wikipedia.org/wiki/Neville's_algorithm <- for reference
     *
     * @param {Number} x                           number for which polynomial is calculated
     * @param {Array} X                            set of distinct x values
     * @param {Array} Y                            set of distinct y=f(x) values
     * @returns {number}                           interpolating polynomial
     */

    function nevillesIteratedInterpolation(x, X, Y) {
        var Q = [Y]
        for (var i = 1; i < X.length; i++) {
            Q.push([])
            for (var j = 1; j <= i; j++) {
                Q[j][i] = ((x - X[i - j]) * Q[j - 1][i] - (x - X[i]) * Q[j - 1][i - 1]) / ( X[i] - X[i - j] )
            }
        }
        return Q[j - 1][i - 1]
    }

    function makeItArrayIfItsNot(input) {
        return Object.prototype.toString.call(input) !== '[object Array]'
            ? [input]
            : input
    }

}

window.interpolator = new Interpolator();