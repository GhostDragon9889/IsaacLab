Expanded pedestrian body capsules in the third-floor configurations to cover the standing body envelope and avoid delayed contact after visible overlap.
Changed the shared ``body_radius_scale`` from 0.70 to 2.50; custom profiles can retain their previous value if they require the smaller contact region.
Allowed finite positive ``body_radius_scale`` values above 1.0 while rejecting radii larger than half the original capsule's total height.
Preserved capsule total height and added live skinned-geometry and PhysX boundary checks on initial playback and replay.
