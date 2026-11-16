| Metric | MPC | PPO |
|---|---:|---:|
| Settled successes | 100/100 | 100/100 |
| Falls / collisions / timeouts | 0 / 0 / 0 | 0 / 0 / 0 |
| Mean true final goal distance (m) | 0.028977 | 0.049744 |
| Mean time-aligned planar RMSE (m) | 0.022198 | 0.036628 |
| Mean longitudinal tracking RMSE (m) | 0.021061 | 0.01483 |
| Mean yaw tracking RMSE (rad) | 0.014402 | 0.028933 |
| Mean pitch RMS (rad) | 0.0098015 | 0.014645 |
| Mean integrated squared torque (N² m² s) | 0.26941 | 0.29492 |
| Mean successful completion time (s), n=100 each | 34.18 | 34.35 |

| Computation (µs) | Mean | Median | p95 | p99 | Maximum |
|---|---:|---:|---:|---:|---:|
| PPO actor | 91.89 | 90.58 | 95.84 | 126.2 | 689.6 |
| PPO full update | 96.55 | 95.04 | 99.92 | 139 | 397.2 |
| MPC compute | 4580 | 4519 | 4825 | 5669 | 1.287e+04 |
| MPC full update | 4584 | 4523 | 4829 | 5673 | 1.287e+04 |
