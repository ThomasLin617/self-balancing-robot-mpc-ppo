# Trajectory-Conditioned PPO

The policy tracks a timed reference trajectory and outputs independent left and right wheel torques. The shared C++ planner supplies the same 250-step reference horizon used by MPC.

## Observation

The input has 1256 values. The first six describe the current estimated state in local position and yaw coordinates. Each of the next 250 groups contains five future reference values.

```text
[0, forward_velocity, pitch, pitch_rate, 0, yaw_rate]
[position_ref - current_position, velocity_ref, pitch_ref,
 unwrapped_yaw_ref - current_yaw, yaw_rate_ref] × 250
```

Samples cover `t + 0.002` through `t + 0.500` seconds. Longitudinal references include the segment's saved position origin. Yaw references are unwrapped sequentially from current estimated yaw. At the trajectory endpoint, position and yaw hold while velocity and yaw rate become zero.

| Quantity | Normalization scale |
|---|---:|
| Position | 1 m |
| Velocity | 1 m/s |
| Pitch | 0.35 rad |
| Pitch rate | 2 rad/s |
| Yaw | π rad |
| Yaw rate | 2 rad/s |

Normalized inputs are clipped to ±5. The actor uses two 128-unit tanh layers and two linear outputs. Outputs are clipped to ±1 and scaled by 2 N·m. The backend applies its own ±2 N·m clamp.

## Training

Stable-Baselines3 PPO trains the actor and a separate value network in PyTorch. Eight environments collect rollouts with 1000 Hz physics and 500 Hz torque updates. A pybind11 bridge calls the production C++ trajectory generator and sampler. Sensor processing follows the C++ estimator equations and is checked against deterministic sensor traces.

The curriculum progresses through balance, signed straight motion, turns and known-map routes. Each stage continues from a selected validation checkpoint. Full-circle turning and route refinement train the final actor on goals in all directions. The final route refinement uses learning rate 1e-6, 2048 steps per environment, batch size 256, ten optimization epochs, discount 0.999 and GAE parameter 0.98.

For the selected training configuration,

$$
C=(e_p/0.03)^2+(e_v/0.20)^2+(\theta_{true}/0.10)^2
 +(\dot\theta/1.0)^2+(\mathrm{wrap}(e_\psi)/0.10)^2
 +(e_{\dot\psi}/0.50)^2+0.02(\tau_L^2+\tau_R^2).
$$

$$
r=0.002(1-C)+30I_{success}-20I_{failure}.
$$

Tracking errors use estimated state and the current reference. Simulator pitch contributes to the training reward and fall detection. Contact and boundary checks determine failure. Actor inputs come from estimated state and the reference horizon.

Success requires the trajectory to have ended, estimated goal distance below 0.10 m, speed below 0.05 m/s, yaw rate below 0.10 rad/s and pitch below 0.10 rad for 0.5 s. Timeout is trajectory duration plus ten seconds.

## C++ and ROS execution

The exported actor stores network dimensions, normalization scales and weights. C++ inference uses dense matrix-vector products and tanh with owned weight buffers. The policy core consumes state and references independently of the simulator and ROS.

`trajectory_policy_smoke_test` runs either MPC or the trajectory actor in the same scene. `trajectory_ppo.launch.py` runs backend, estimator and policy as separate ROS processes with simulation-time cadence, reset coordination and freshness checks.

The selected actor is `artifacts/learning/d18_selected_full_circle_v2/actor.txt`. Its format identifier and artifact path are used by the loader and reproduction tools.

[Final results and reproduction](CONTROLLER_COMPARISON.md) · [ROS runtime](ROS2_INTEGRATION.md)
