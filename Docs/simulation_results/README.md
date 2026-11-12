# Retained simulation evidence

`acceptance.json` is the case index, including metrics, the raw run directory,
and SHA256 digests of generated configuration. `group: acceptance` contains
completed selected tests; `superseded_failure` contains two representative
failed tuning attempts. It is not a count of every trial attempted.

Every selected case retains its original `evaluation.json` and `run.json`.
Representative cases additionally retain generated config, truth CSV and PNG.
CSV line endings and trailing whitespace in retained text logs are normalized;
raw files remain in the run directories. The manifest records the seed, model, encounter, command arguments, commit,
working-tree status and Python source hashes. A commit alone does not identify
a dirty trial's exact configuration.

The final compiled goal-checker source/library hashes and runtime versions are
in `build_fingerprints.json`.

Full stdout/stderr, intermediate successes/failures and interrupted overload
trials remain in `/tmp/njord-night/` on the test host. They are not required to
view these retained summaries and representative trajectories.

`unit_tests.log`, `gui_tests.log`, `goal_checker_test.log` and
`foxglove_bridge_smoke.json` record focused verification. The separate
`baseline_robot_test_failures.log` preserves seven existing launch/network/video
assertion failures instead of treating the whole repository as green.

See [the Japanese report](../simulation_acceptance.md) and
[tuning notes](../simulation_tuning_log.md) for interpretation and limitations.
