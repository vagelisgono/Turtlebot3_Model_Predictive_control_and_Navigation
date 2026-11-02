#!/usr/bin/env python3

import csv
import os
import numpy as np
import matplotlib.pyplot as plt

from scipy.optimize import curve_fit



filename = os.path.expanduser("~/turtlebot_data.csv") #read datafrom csv

with open(filename, "r") as file:
    data = list(csv.DictReader(file))




def identify(phase, command_column, measured_column):

    
    samples = [row for row in data if row["phase"] == phase] #select mesurements

    if len(samples) < 5:
        raise ValueError("Not enough measurements for " + phase)

    
    t = np.array([float(row["time"]) for row in samples])            #get mesurements from time and velocity
    y = np.array([float(row[measured_column]) for row in samples])

    
    u = float(samples[0][command_column]) #get mesurements for cmd velocity

   
    t = t - t[0] #set time at 0

    
    y0 = y[0]

    # First-order step-response model
    #
    # tau * y_dot + y = K * u
    #
    # Solution:
    # y(t) = K*u + (y0 - K*u)*exp(-t/tau)

    def model(t, K, tau):

        return K*u + (y0 - K*u)*np.exp(-t/tau)

   
    parameters, _ = curve_fit( #Estimate K and tau using none linear least squares
        model,
        t,
        y,
        p0=[1.0, 0.2],
        bounds=([0.0, 0.001], [3.0, 5.0]),
        maxfev=10000
    )

    
    K, tau = parameters #extract parameters

    
    y_model = model(t, K, tau) #calculate predicted velocity

    
    rmse = np.sqrt(np.mean((y - y_model)**2)) #calculate rmse

    
    print("\nExperiment:", phase)
    print("K =", K)
    print("tau =", tau, "seconds")
    print("RMSE =", rmse)

   
    dt = np.median(np.diff(t))

    if np.ptp(y) < 0.05 * abs(u) or tau < 2 * dt:
        print("WARNING: tau may not be reliably identifiable.")

   
    plt.figure(figsize=(9, 5)) #plot mesured an predicted velocities

    plt.plot(t, y, label="Measured Velocity")
    plt.plot(t, y_model, "--", label="Identified Model")
    plt.axhline(u, color="gray", linestyle=":", label="Command")

    plt.xlabel("Time (s)")
    plt.ylabel("Velocity")
    plt.title(phase)

    plt.legend()
    plt.grid(True)

    plt.savefig(phase + ".png")
    plt.show()

    return K, tau




Kv, tau_v = identify( #for linear
    "linear_motion",
    "v_cmd",
    "v_measured"
)




Kw, tau_w = identify( #for angular
    "angular_motion",
    "omega_cmd",
    "omega_measured"
)




print("\n========== IDENTIFIED PARAMETERS ==========")

print("Kv =", Kv)
print("tau_v =", tau_v)

print("Kw =", Kw)
print("tau_w =", tau_w)