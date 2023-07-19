
cd ~/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples
cd config

# edit extension.toml

* extension examples
```
[[python.module]]
name = "omni.isaac.examples.isaac_extension_examples.example1_cube"

[[python.module]]
name = "omni.isaac.examples.isaac_extension_examples.example2_robot"

[[python.module]]
name = "omni.isaac.examples.isaac_extension_examples.example3_controller"

[[python.module]]
name = "omni.isaac.examples.isaac_extension_examples.example4_manipulator"
```

* standalone examples
```
cd ~/.local/share/ov/pkg/isaac_sim-2022.2.1
./python.sh ./extension_examples/isaac_extension_examples/standalone_apps/fancy_cube.py
```