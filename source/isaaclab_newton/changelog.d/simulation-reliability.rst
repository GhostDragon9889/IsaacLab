Fixed
^^^^^

* Fixed missing GPU box-box contacts at symmetric placements by selecting MuJoCo Warp's dedicated
  primitive algorithm by default. Added ``MJWarpSolverCfg.use_native_ccd`` to opt into the general
  convex GJK/EPA algorithm when required.
* Fixed eager resets reading uninitialized MPM storage after deferred CUDA graph capture by
  initializing the captured graph before the reset boundary.
