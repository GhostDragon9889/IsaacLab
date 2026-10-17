Reduced repeated USD change processing by batching robot visual transform updates within each physics step.
Added the optional ``robot_yield_min_walk_speed`` setting [m/s] and enabled a 0.5 m/s slowdown floor for the third-floor profiles to avoid the observed low-speed standing translation.
Kept the original patrol speed as the upper bound; omitting the setting or setting it to zero restored the previous 40% slowdown.
