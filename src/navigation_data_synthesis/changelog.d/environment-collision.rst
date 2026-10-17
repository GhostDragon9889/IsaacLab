Reduced visible robot/environment overlap by applying chassis and arm contact margins in the scene override layer while preserving wheel and roller contact settings.
Replaced third-floor stopping and robot-yield waits with a looping animated standing action that ignored root motion.
Adjusted case_1 standing placement, patrol timing, and corner waypoints to leave clearance for the enlarged body capsules while preserving final patrol destinations.
Added live skinned-body wall checks and motor-driven PhysX contact regressions for walls, the washstation, partitions, and a movable door.
Extended the regression to replay sampled gamepad axes through the authored mixer, safety guards, and articulation controller, including lateral contact and neutral rearming after invalid input.
Added optional pedestrian footprint waiting through ``pedestrian_yield_clearance`` and moved the case_1 initial waiting position out of the shared passage while retaining patrol destinations.
Extended contact regressions with pairwise body separation, a blocked shared passage, washstation end and corner approaches, and sampled whole-robot visual overlap checks.
Fixed missing robot collisions when visual separation ran before PhysX initialization by loading 25 independent cooked convex hulls and preserving meshes used by enabled ancestor colliders.
Kept the robot's detailed rendering separate from its lightweight physics geometry, and added a regression for visual-first initialization and real PhysX hull queries.
Documented the remaining Full-scene FPS regression after restoring collision geometry; the 30 FPS performance target was not met.
The shared third-floor settings now required the bundled ``standing_motion_library.usda`` when ``stationary_action = "Standing"`` was enabled.
