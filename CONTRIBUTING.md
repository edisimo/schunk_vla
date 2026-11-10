# Contributing

- Branch from `main` as `feat/<snake_case>` or `fix/<snake_case>`; merge through a reviewed PR.
- Bump the version in `pyproject.toml` (and `ros/lwa4p_vla_bridge/package.xml` for bridge changes).
- Python: black + isort, line length 100. Docstrings on modules, classes and functions.
- Before a PR: `scripts/vla.sh black src tests ros && scripts/vla.sh isort src tests ros`, then
  `scripts/vla.sh pytest`, and a run against the mock robot
  (`ros2 launch lwa4p_vla_bridge robot.launch.py mode:=mock`).
- Changes to the driver go into `patches/lwa4p_driver/` (`git diff` inside the submodule),
  never as commits in the submodule.
