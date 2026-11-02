#!/usr/bin/env python3

import csv
import numpy as np
import matplotlib.pyplot as plt



Kv = 1.0011045      #identified parameters
tau_v = 0.0774945

Kw = 0.9942472
tau_w = 0.032835



with open("/home/vaggelis/turtlebot_data.csv", "r") as file: #load data
    data = [row for row in csv.DictReader(file)
            if row["phase"] == "combined_motion"]



def column(name):
    return np.array([float(row[name]) for row in data]) #convert csv to colums


t = column("time")
v_cmd = column("v_cmd")
w_cmd = column("omega_cmd")

v = column("v_measured")
w = column("omega_measured")

x = column("x")
y = column("y")
theta = column("theta")


# Initialize model predictions
v_model = np.zeros(len(t))
w_model = np.zeros(len(t))

x_model = np.zeros(len(t))
y_model = np.zeros(len(t))
theta_model = np.zeros(len(t))

v_model[0] = v[0]
w_model[0] = w[0]

x_model[0] = x[0]
y_model[0] = y[0]
theta_model[0] = theta[0]



for i in range(1, len(t)): #model simulation

    dt = t[i] - t[i-1]

    
    av = np.exp(-dt / tau_v)
    aw = np.exp(-dt / tau_w)

    v_model[i] = av * v_model[i-1] + Kv * (1-av) * v_cmd[i-1]
    w_model[i] = aw * w_model[i-1] + Kw * (1-aw) * w_cmd[i-1]

    
    v_avg = (v_model[i] + v_model[i-1]) / 2
    w_avg = (w_model[i] + w_model[i-1]) / 2

    theta_mid = theta_model[i-1] + w_avg * dt / 2

    x_model[i] = x_model[i-1] + v_avg * np.cos(theta_mid) * dt
    y_model[i] = y_model[i-1] + v_avg * np.sin(theta_mid) * dt
    theta_model[i] = theta_model[i-1] + w_avg * dt



v_rmse = np.sqrt(np.mean((v - v_model)**2)) #RMSE calculation
w_rmse = np.sqrt(np.mean((w - w_model)**2))




position_rmse = np.sqrt(np.mean(
    (x - x_model)**2 + (y - y_model)**2
))

theta_error = np.arctan2(
    np.sin(theta - theta_model),
    np.cos(theta - theta_model)
)
theta_rmse = np.sqrt(np.mean(theta_error**2))


print("Linear velocity RMSE:", v_rmse, "m/s")
print("Angular velocity RMSE:", w_rmse, "rad/s")
print("Position RMSE:", position_rmse, "m")
print("Heading RMSE:", theta_rmse, "rad")



plt.plot(x, y, label="Measured")  
plt.plot(x_model, y_model, "--", label="Predicted")  #plot

plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("TurtleBot3 Model Validation")
plt.axis("equal")
plt.legend()
plt.grid()
plt.show()

