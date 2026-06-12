# Vehicle Position

ros2 topic echo /fmu/out/vehicle_odometry | grep position -A 3

# Sensor Data

listener sensor_combined

# Estimator Status

listener estimator_status_flags
