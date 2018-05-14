Building an Estimator
===============================
Frederick Chyan  
2018/05/14

### Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.

This was done by loading `log/Graph1.txt` and `log/Graph2.txt` into an array and compute the standard deviation using numpy. 

### Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.

The attitude integration scheme was not practical as it assumes the Euler angles were small so there is small difference between body and inertial frame. To improve this, we must take coordinate transformation into account. First, create a Quatertinion from the Euler angle. Then, use its built-in function `IntegrateBodyRate()` to directly perform the integration given gyro measurement and delta time. Finally, we can convert this back to Euler angle.

### Implement all of the elements of the prediction step for the estimator.
The prediction step provides estimate of the new probablity distribution, which means both the mean and variance needs to be predicted. The state (mean) is predicted in `PredictState`, this is basically updating the position and velocity. Position is just `current_position + current_velocity * dt` for x, y, z. Velocity is similar `current_velocity + acceleration * dt`. However, since acceleration was measured in body frame, we need to first convert it to global frame. Also, gravity needs to be included in the z componenet. The uncertainty is calculated in `Predict` using the Kalman filter equation ` ekfCov = gPrime * ekfCov * gPrime^T + Q`. Since linearization is needed for 3D quadrotor, the jacobian (gPrime) has additional math that needs to be carried out. Namely the Rbg_prime, which is the partial derivative of the velocity component with respect to the yaw angle. This 3 by 3 matrix is implemented in `GetRbgPrime`.

### Implement the magnetometer update.
For the simulator we are assuming the magnetometer output has been post processed and so the result is measuring the quad's yaw angle directly. Then the h_prime is simply 1 for the yaw position, and the measurement model zFromX is simply the current yaw state. For the measuerement z, we do have to change the angle so the update is less than pi. For example, if the magnetometer is measuring 170 degrees, and the current state is -175 degrees. This is 15 only degrees counter-clockwise, but if we don't change the magnetometer measurement, the upadte will be 345 degrees clockwise. So if the update is greater than pi we subtract 2pi from the magnetometer measurement, and if the update is less than -pi we will add 2*pi to the magnetometer measurement.

### Implement the GPS update.
The coding portion of the GPS update is simple as it is directly measuring the position and velocity. The hprime is basically a identity matrix and measurement model is the same as the current ekf estimate.

### De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.
Although the GPS update is simple, it took a while to tune the drone using non-ideal estimator. After replacing the code with `QuadControl` and `QuadControlParams` implemented in previous project, the quad crashed right away. First I lowered all PID gains by 20~30%. Now the quad is able to complete the flight and achived < 1m error, however, it still crashed in the end, when it began to hover. I think this was due to the negative feedback loop coupled with noisy sensor update, so when the motion is (supposed) to be still there is less motion predict for the kalman filter to work with, so the error is amplified and quad is trying too hard to correct it so it crashed. After lowering the integral gain and reduce the proportional gain for the bank angle, now the quad is able to hover without crashing.