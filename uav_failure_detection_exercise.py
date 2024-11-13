#!/usr/bin/env python3

'''
Copyright 2024 Kjeld Jensen <kjen@sdu.dk>
SDU UAS Center
University of Southern Denmark

Course: Introduction to Drone Technology
Module: UAV attitude failure detection
This code has been tested to work under Ubuntu 22.04 and 24.04

2024-10-31 KJ First version
'''
from select import error

from pyulog import ULog
import matplotlib.pyplot as plt
import numpy as np


#####################################################################
'''
This is the section where you have to modify the code to create your
UAV failure detection algorithm. For now the algorithm only prints
the values and some are plotted at the end.

At these links you will find videos of the flights corresponding to
the log files:

Test 5: https://youtu.be/raK2fnk5ULk
Test 8: https://youtu.be/gJK06kHUvz8
Test 9: https://youtu.be/opDSC2ox-c4
'''

#log_file_path = 'TEST5_30-01-19.ulg'
#log_file_path = 'TEST8_30-01-19.ulg'
log_file_path = 'TEST9_08-02-19.ulg'

start_seconds = 250
end_seconds = 350
pressure_time = []
pressure_log = []
acceleration_time = []
acceleration_log = []
kill_sw_time = []
kill_sw_log = []

#control logs
control_time = []
control_log = []

discrepancy_time = []

def algorithm_init():
    print ('Algorithm init')



# Define thresholds for discrepancy detection
DISCREPANCY_THRESHOLD = 40  # Set this based on observed normal operation values

# Lists to store data for analysis
pressure_time = []
pressure_log = []
acceleration_time = []
acceleration_log = []
gyro_time = []
gyro_log = []
kill_sw_time = []
kill_sw_log = []
control_time = []
control_log = []

erro_time = []
error_log = []



# ADRC parameters
B0 = 0.1  # Control gain (tuned empirically)
ESO_GAIN = np.array([0.1, 0.3, 0.3])  # Observer gains for [acc_x, gyro_x, disturbance]
state_estimate = np.zeros(6)  # [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
disturbance_estimate = np.zeros(6)  # Disturbance estimates for each axis

# Simplified dynamics model parameters (these would ideally be tuned for the drone)
mass = 1.5  # Mass of the quadcopter in kg
inertia = np.array([0.02, 0.02, 0.04])  # Simplified moment of inertia for each axis

def adrc_eso_update(control_input, measurement):
    global state_estimate, disturbance_estimate

    # Extended State Observer update
    observed_state = state_estimate[:6]
    disturbance = disturbance_estimate[:6]

    for i in range(3):  # Loop through axes for both acceleration and gyro states
        # Apply the simplified drone model
        if i < 3:  # Translational motion (acceleration)
            control_effect = B0 * control_input[i] / mass  # Translational force -> acceleration
        else:  # Rotational motion (angular rate)
            control_effect = B0 * control_input[i - 3] / inertia[i - 3]  # Torque -> angular acceleration

        # ESO update for each axis independently (both acceleration and gyro)
        error = measurement[i] - (observed_state[i] + disturbance[i])  # Observation error
        state_estimate[i] += ESO_GAIN[0] * error  # State estimate correction
        disturbance_estimate[i] += ESO_GAIN[1] * error - ESO_GAIN[2] * disturbance[i] + control_effect

    return state_estimate, disturbance_estimate
def algorithm_update(new_data):
    # Check if the data is within our time interval of interest
    if start_seconds < new_data['timestamp'] < end_seconds:

        # Handle barometer data
        if new_data['field'] == 'vehicle_air_data':
            pressure_time.append(new_data['timestamp'])
            pressure_log.append(new_data['baro_pressure_pa'])

        # Handle accelerometer and gyroscope data
        if new_data['field'] == 'sensor_combined':
            acceleration_time.append(new_data['timestamp'])
            acceleration_log.append([new_data['acc_x'], new_data['acc_y'], new_data['acc_z']])
            gyro_log.append([new_data['gyro_rad_x'], new_data['gyro_rad_y'], new_data['gyro_rad_z']])
            gyro_time.append(new_data['timestamp'])

        # Handle kill switch status change
        if new_data['field'] == 'manual_control_setpoint':
            kill_sw_time.append(new_data['timestamp'])
            kill_sw_log.append(new_data['aux1'])

            # Log control inputs (x, y, z, r axes)
            control_time.append(new_data['timestamp'])
            control_log.append([new_data['x'], new_data['y'], new_data['z'], new_data['r'], new_data['aux1']])

        # Check for discrepancy using ADRC observer
        check_for_discrepancy()

def check_for_discrepancy():
    # Ensure we have enough data to compare recent control inputs and sensor readings
    if len(control_log) > 0 and len(acceleration_log) > 0 and len(gyro_log) > 0:
        latest_control = control_log[-1]
        latest_acceleration = acceleration_log[-1]
        latest_gyro = gyro_log[-1]

        # Normalize control input to match expected acceleration and angular rate
        control_input = np.array(latest_control)  # scale factor based on empirical data

        # Combine accelerometer and gyro data as measurement
        measurement = np.array(latest_acceleration + latest_gyro)

        # Use ADRC ESO to estimate the state and disturbance
        estimated_state, disturbance_estimate = adrc_eso_update(control_input, measurement)

        # Calculate discrepancies between observed and estimated states for acceleration and gyro
        acc_discrepancies = np.abs(estimated_state[:3] - np.array(latest_acceleration))
        gyro_discrepancies = np.abs(estimated_state[3:] - np.array(latest_gyro))

        # Calculate disturbance magnitudes
        disturbance_magnitude = np.linalg.norm(disturbance_estimate)

        erro_time.append(acceleration_time[-1])
        er = np.linalg.norm(acc_discrepancies) + np.linalg.norm(gyro_discrepancies) + disturbance_magnitude
        error_log.append(er)

        # Check if any discrepancy or disturbance magnitude exceeds the threshold
        if er > DISCREPANCY_THRESHOLD :
            discrepancy_time.append(acceleration_time[-1])

def algorithm_done():
    print ('Algorithm done')

    # convert Pa to meters
    barometric_altitude_meters = []
    for p in pressure_log:
        barometric_altitude_meters.append(44330.77 * (1.0 - (p/101325.0)**0.190284))

    mean_barometric_altitude = np.mean(barometric_altitude_meters)

    barometric_altitude_meters = [x - mean_barometric_altitude for x in barometric_altitude_meters]

    #acceleration vector length
    acc_vector = []
    for acc in acceleration_log:
        acc_vector.append((acc[0]**2 + acc[1]**2 + acc[2]**2)**0.5)


    barometric_velocity = []
    barometric_velocity.append(0)

    alpha = 0.1
    # calculate velocity from barometric altitude calculate gradietn and smooth it
    for i in range(1, len(barometric_altitude_meters)):
        barometric_velocity.append(alpha * (barometric_altitude_meters[i] - barometric_altitude_meters[i-1]) + (1-alpha) * barometric_velocity[-1])


    # plots stacked on top of each other
    plt.figure(1)

    plt.title('Fail detection algorithm')


    plt.subplot(311)
    plt.plot(pressure_time, barometric_altitude_meters)
    plt.legend(['Barometric height'])
    # plot discrepancy time
    for t in discrepancy_time:
        plt.axvline(x=t, color='r', linestyle='--')

    plt.xlim(start_seconds, end_seconds)
    plt.grid(True)


    plt.subplot(312)
    plt.plot(control_time, control_log)
    plt.title('Control inputs')
    plt.legend(['x', 'y', 'z', 'r','aux1'])
    plt.xlim(start_seconds, end_seconds)

    plt.grid(True)

    plt.subplot(313)
    plt.plot(erro_time, error_log)
    plt.title('Error')
    plt.ylim(0, 100)
    plt.xlim(start_seconds, end_seconds)

    #horizontal line at the threshold
    plt.axhline(y=DISCREPANCY_THRESHOLD, color='r', linestyle='--', label='Threshold')
    plt.text(300, DISCREPANCY_THRESHOLD, 'Threshold', color='r')

    plt.grid(True)

    plt.tight_layout()
    plt.show()


    plt.show()

#####################################################################

def load_data(log_file_path):
    # Load the ULog file
    ulog = ULog(log_file_path)
    d = []
    
    # loop through all data types in the ulog file
    for data in ulog.data_list:
        #print (data.name)

        if data.name == 'sensor_combined':
            data_keys = [f.field_name for f in data.field_data]
            #print ('Keys', data_keys)
            timestamps = data.data['timestamp']
            gyro_rad_x = data.data['gyro_rad[0]']
            gyro_rad_y = data.data['gyro_rad[1]']
            gyro_rad_z = data.data['gyro_rad[2]']
            accelerometer_x = data.data['accelerometer_m_s2[0]']
            accelerometer_y = data.data['accelerometer_m_s2[1]']
            accelerometer_z = data.data['accelerometer_m_s2[2]']
            for ts, g_x, g_y, g_z, acc_x, acc_y, acc_z in zip(timestamps, gyro_rad_x, gyro_rad_y, gyro_rad_z, accelerometer_x, accelerometer_y, accelerometer_z):
                d.append({"timestamp": int(ts)/1000000., "field": data.name, "gyro_rad_x": float(g_x), "gyro_rad_y": float(g_y), "gyro_rad_z": float(g_z), "acc_x": float(acc_x), "acc_y": float(acc_y), "acc_z": float(acc_z)})

        if data.name == 'vehicle_air_data':
            data_keys = [f.field_name for f in data.field_data]
            #print ('Keys', data_keys)
            timestamps = data.data['timestamp']
            pressures = data.data['baro_pressure_pa']
            for ts, pres in zip(timestamps, pressures):
                d.append({"timestamp": int(ts)/1000000., "field": data.name, "baro_pressure_pa": float(pres)})

        if data.name == 'manual_control_setpoint':
            data_keys = [f.field_name for f in data.field_data]
            print ('Keys', data_keys)
            timestamps = data.data['timestamp']
            aux1 = data.data['aux1']

            x = data.data['x']
            y = data.data['y']
            z = data.data['z']
            r = data.data['r']

            for ts, aux, x, y, z, r in zip(timestamps, aux1 , x, y, z, r):
                d.append({"timestamp": int(ts)/1000000., "field": data.name, "aux1": float(aux), "x": float(x), "y": float(y), "z": float(z), "r": float(r)})


    # sort all data using the timestamp (first entry) as key
    sorted_data = sorted(d, key=lambda x: list(x.values())[0])
    return sorted_data    

def run_simulation(data):
    algorithm_init()
    for d in data:
        algorithm_update(d)
    algorithm_done()

if __name__ == "__main__":
    print ('Loading data from the ulog file')
    data = load_data(log_file_path)

    print ('Run simulation')
    run_simulation(data)

    print ('Simulation completed')

