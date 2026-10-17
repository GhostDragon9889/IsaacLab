# Runtime-specific image references

Shared images remain the fallback. `../profiles.json` selects an existing override only when the
GPU model and the Isaac Sim, Newton, and Warp package versions all match exactly. Missing images,
CPU-only tests, and other runtimes continue to use the shared images. Pixel and SSIM thresholds
are identical for both paths. A mismatch never regenerates a runtime reference automatically.

The RTX 4080 SUPER profile contains only the five Cable geometry/shading outputs and two MPM
color outputs that differ from the shared references. The MPM references were originally captured
on A40 (commit `348636777`); the Cable shared RTX references predate the backend-specific Franka
payload change in `741b164eb`. These references describe the current runtime without claiming
identical rendering or deformable trajectories across hardware and solver versions.

To review or add a profile, record the exact runtime, capture the unchanged test in a fresh
process, inspect the outputs, and verify at least two subsequent fresh-process runs with the
original thresholds. Also verify that hiding the target geometry still fails comparison.
The per-profile file list and SHA-256 values in the manifest identify the reviewed images.
