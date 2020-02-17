#!/usr/bin/env python3
import numpy as np
import scipy.integrate as spi
import random as rand
import argparse
import os
import pickle
import shelve
from scipy.optimize import curve_fit
from scipy.optimize import OptimizeWarning
import warnings
from tqdm import trange


def straightline_func(x, a, b):
    return a*x+b

def exponential_func(x, a, b):
    with warnings.catch_warnings():
        warnings.filterwarnings("error")
        try:
            y= a*np.exp(b*x)
            return y
        except:
            return 0*x

def square_func(x,a,b,c):
    return a*np.power(x,2) + b*x + c

def generateStepInput(responseDuration,startInput,minInput,maxInput):

    input = np.zeros( (responseDuration,1) )
    timestep = startInput

    while timestep < responseDuration:

        magInput = (maxInput-minInput)*np.random.random()+minInput # Magnitude Size of Input
        inputDur = int(responseDuration/10*(np.random.random() ) ) # Duration of input
        zeroInputDur = int(responseDuration/10*(np.random.random()) ) # Duration of zero input


        input[timestep:timestep+inputDur] = magInput
        timestep += inputDur
        input[timestep:timestep+zeroInputDur] = 0
        timestep += zeroInputDur

    return input

def generateRampInput(responseDuration,startInput,minInput,maxInput):

    input = np.zeros( (responseDuration,1) )
    timestep = startInput

    while timestep < responseDuration:
        magInput = (maxInput-minInput)*np.random.random()+minInput # peak point in ramp
        firstDur = int(responseDuration/10*(np.random.random()))+1 # Duration of first half ramp
        secondDur = int(responseDuration/10*(np.random.random()))+1 # Duration of second half ramp

        if(timestep + firstDur+secondDur < responseDuration):

            grad1 = magInput/firstDur   # gradient of first part
            grad2 = -magInput/secondDur  # Gradientr of second part

            firstLine = np.arange(firstDur)*grad1

            secondLine = -1*np.arange(secondDur,0,-1)*grad2
            input[timestep:timestep+firstDur] = np.transpose(np.array([firstLine]))
            timestep += firstDur
            input[timestep:timestep+secondDur] = np.transpose(np.array([secondLine]))
            timestep += secondDur
        else:
            break

    # input = addNoise(input,250)
    return input


def generateSquareInput(responseDuration,startInput,minInput,maxInput):

    input = np.zeros( (responseDuration,1) )
    timestep = startInput

    while timestep < responseDuration:

        y1 = ((maxInput-minInput)*np.random.random()+minInput)/3
        y2 = ((maxInput-minInput)*np.random.random()+minInput)/3
        y3 = ((maxInput-minInput)*np.random.random()+minInput)/3
        y4 = ((maxInput-minInput)*np.random.random()+minInput)/3

        x1 = int(responseDuration/10*(np.random.random()))+1
        x2 = int(responseDuration/10*(np.random.random()))+1
        x3 = int(responseDuration/10*(np.random.random()))+1

        x  = np.array([timestep+1,timestep+x1,timestep+x1+x2,timestep+x1+x2+x3])
        y = np.array([y1,y1+y2,y1+y2+y3,y1+y2+y3+y4])

        if(timestep + x1 + x2 +x3 < responseDuration):

            popt, pcov = curve_fit(square_func, x, y)
            c = popt[2]
            b = popt[1]
            a = popt[0]
            curve = np.arange(timestep,timestep+x1+x2+x3)
            curve = square_func(curve, a, b, c)
            input[timestep:timestep+x1+x2+x3] = np.transpose(np.array([curve]))
            timestep = timestep + x1 + x2 + x3

        else:
            break

    # input = addNoise(input,250)
    return input

def generateExpoInput(responseDuration,startInput,minInput,maxInput):

    input = np.zeros( (responseDuration,1) )
    timestep = startInput
    error_check = 0

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        while timestep < responseDuration:


            y11 = ((maxInput-minInput)*np.random.random()+minInput)/2 #
            y12 = ((maxInput-minInput)*np.random.random()+minInput)/2 #

            x11 = int(responseDuration/10*(np.random.random())) #
            x12 = int(responseDuration/10*(np.random.random()))+20 #

            y21 = ((maxInput-minInput)*np.random.random()+minInput)/2 #

            x21 = int(responseDuration/10*(np.random.random())) #
            x22 = int(responseDuration/10*(np.random.random()))+20 #

            if(timestep + x11+ x12 + x21 + x22 + 1 < responseDuration):

                y = np.log(np.array([0.00001, y11, y11+y12]))
                x = np.array([timestep+1,timestep+x11,timestep+x11+x12])
                popt, pcov = curve_fit(straightline_func, x, y)
                b = popt[0]
                a = np.exp(popt[1])
                curve = np.arange(timestep,timestep+x11+x12)
                curve = exponential_func(curve, a, b)
                input[timestep:timestep+x11+x12] = np.transpose(np.array([curve]))

                y = np.log(np.array([y11+y12,y21, 0.0001]))
                x = np.array([timestep+x11+x12,timestep+x11+x12+x21, timestep+x11+x12+x21+x22])
                popt, pcov = curve_fit(straightline_func, x, y)
                b = popt[0]
                a = np.exp(popt[1])
                curve = np.arange(timestep+x11+x12,timestep+x11+x12+x21+x22)
                curve = exponential_func(curve, a, b)
                input[timestep+x11+x12:timestep+x11+x12+x21+x22] = np.transpose(np.array([curve]))
                timestep = timestep + x11 + x12 + x21 + x22
            else:
                break
    return input

def generateNoiseInput(responseDuration,startInput,minInput,maxInput):

    input = np.zeros( (responseDuration,1) )
    input += (maxInput-minInput)*np.random.random((np.size(input),1))+minInput
    return input

def addNoise(response,level):
    sizeOfArray = np.size(response)
    response += np.random.random((sizeOfArray,1))/level
    return response

def generateCombinationInput(responseDuration,startInput,minInput,maxInput):
    input1 = generateStepInput(responseDuration,startInput,minInput/4,maxInput/4)
    input2 = generateRampInput(responseDuration,startInput,minInput/4,maxInput/4)
    input3 = generateExpoInput(responseDuration,startInput,minInput/4,maxInput/4)
    input4 = generateSquareInput(responseDuration,startInput,minInput/4,maxInput/4)
    input = input1+input2+input3+input4
    # input = addNoise(input1+input2+input3+input4,0.5)
    return input
