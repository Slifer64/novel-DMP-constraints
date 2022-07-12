# novel-DMP-constraints : Simulations

Containts scripts that run the simulations and plot the corresponding figures. In each case, the figures may not be concatenated as in the paper, or some additional plots may also appear (e.g. the relaxation variables).

The code was tested in Ubuntu 20.04 using MatlabR20a.

## Dependencies
- QSQP (matlab interface)
For installing QSQP you can follow the instruction in https://osqp.org/docs/get_started/matlab.html

## Simulations
Run the following matlab scripts for each Figure:

**Off-line DMP simulations**
Here, we use the training data in `data/pos_data.bin`.
- Figure 1a: `fig1a_offline_opt_pos_vs_vel.m`
- Figure 1b: `fig1b_offline_opt_comparison.m`

**On-line DMP simulations**
Here, we use the pretrained DMP model in `data/model.bin`.
- Figure 3: `fig3_viariable_duration.m`, `fig3_viariable_target.m`
- Figure 4: `fig4_via_points.m`
- Figure 5: `fig5_obstacle_avoidance_3D.m`  
  (line 45, set `opt_pos = 1;` for position and `opt_pos = 0;` for velocity optimization)
- Figure 6: `fig6_compare_with_rf.m`
- Figure 7: `fig7_compare_with_mpc.m`

**Appendix A: Modifications for On-line optization**
- Figure 10: `fig10_effect_of_pred_time_step.m`, `fig10_effect_of_relaxations.m`