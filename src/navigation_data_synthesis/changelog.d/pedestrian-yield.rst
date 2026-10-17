Fixed pedestrians losing patrol progress after robot blockage by preserving their target during a standing wait and resuming after clearance.
Restored unfinished triggered patrols when the Isaac Sim 6.1 runtime recreated agents after a pause.
Reduced navigation overhead near the EVT robot by separating pure visual meshes from rigid-body subtrees during playback.
The third-floor settings enabled this mode; render mesh paths moved under ``RenderVisuals`` during playback, so consumers selecting individual visual prims must use those paths or disable ``separate_robot_visuals``.
