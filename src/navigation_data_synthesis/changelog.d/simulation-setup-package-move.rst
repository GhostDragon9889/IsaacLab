Moved the navigation simulation setup scripts from ``navigation_data_synthesis.task`` to ``navigation_data_synthesis.config.simulation_setup_script``.
Updated the third_floor, new_metro, and yoho USD script paths and the collision test import.
Breaking change: direct imports must use the new package path, and external USD layers must reference ``config/simulation_setup_script/character_body_colliders.py`` instead of ``task/character_body_colliders.py``.
