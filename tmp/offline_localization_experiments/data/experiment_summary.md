# Offline Experiment Summary

Generated at: 2026-06-11T13:12:19.174443+00:00

## PCD Inputs

- Files analyzed: 8
- Total original points: 1797796
- Total synthetic rear-box removals: 302
- Total far-return boost extra points: 660905

## Synthetic Gate Sweep

Baseline profile accepted ratios:
- A_clean_map_default: 80/80 accepted (1.0)
- B_ghost_map: 46/80 accepted (0.575)
- C_no_odom_prior: 12/80 accepted (0.15)
- D_strict_score_sensitive: 79/80 accepted (0.9875)
- E_u_turn: 69/80 accepted (0.8625)

Strict score 0.5 accepted ratios:
- A_clean_map_default: 0/80 accepted (0.0)
- B_ghost_map: 1/80 accepted (0.0125)
- C_no_odom_prior: 0/80 accepted (0.0)
- D_strict_score_sensitive: 0/80 accepted (0.0)
- E_u_turn: 0/80 accepted (0.0)

Old 0.25 rad yaw gate accepted ratios:
- A_clean_map_default: 80/80 accepted (1.0)
- B_ghost_map: 46/80 accepted (0.575)
- C_no_odom_prior: 12/80 accepted (0.15)
- D_strict_score_sensitive: 79/80 accepted (0.9875)
- E_u_turn: 36/80 accepted (0.45)

## Interpretation Boundary

These are offline map statistics and deterministic synthetic gate decisions. They are useful for parameter sensitivity and report drafting, but they are not real vehicle NDT fitness, drift, or Nav2 abort measurements.
