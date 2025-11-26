# SPI board: FTDIFT232H
# IMU chip: ICM20948 -> 7MHz SPI chip and 9Dof data

# 
import os, time, struct
import matplotlib.pyplot as plt
import numpy as np 
import board, busio , digitalio 
import adafruit_blinka.microcontroller.ftdi_mpsse.mpsse as mpsse