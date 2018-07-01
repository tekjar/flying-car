* Body rate controller is a p controller which calculates desired pitch, roll and yaw moments for the next iteration based on commanded and current body rates.

```
V3F w = kpPQR * (pqrCmd - pqr);
// L = IW
momentCmd = I * w;
```

* We feed the moment and collective thrust to `GenerateMotorCommands` to calculate necessary individual motor thrust to produce the inputs.

```
f_total = f1 + f2 + f3 + f4
tau_x = (f1 + f4 - f2 - f3) * l
tau_y = (f1 + f2 - f3 - f4) * l
tau_z = (-f1 + f2 - f3 -f4) * kappa
```

We use these 4 equations to solve for 4 variable f1, f2, f3, f4

* The `RollPitchCotrol` takes necessary x, y acceleration in world frame along with current attitude and thrust to calculate control knobs which inturn calculates necessary body rates to produce input acceleration

* `LateralPositionControl` caculates the accelerations necessary for commanded positions in world frame. It is pd controller with feed forward component. feedforward controller helps in maintaining fast response to changing trajectory while d controllers helps with the overshoots

```
accelCmd = kpPosXY * posErr + kpVelXY * velErr + accelCmd;
```

* Altitude controller is a pid controller with feed forward which calculates necessary total thrust (taking current drone attitude into account) to go to commanded z target.

The `i term` is important here to correct steady state errors when actual drone has inconsistencies with the model.