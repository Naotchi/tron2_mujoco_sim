# Changelog

All notable changes to `tron2-mujoco-sim` will be documented here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- `tron2_sim/` variant-registry package: robot selection is now
  data-driven (`tron2_sim/variants/{sf,wf,da,dach,dasf}.py`) instead of
  hard-coded in `simulator.py`. Supported robot types expanded from two
  (`SF_TRON2A`, `WF_TRON2A`) to ten: `SF`, `WF`, `DA`, `DACH`, `DASF`,
  each in a `TRON2A` and a `TRON2B` variant.
- `pyproject.toml` + `uv.lock`: the runtime environment is now managed
  with [uv](https://docs.astral.sh/uv/). `uv sync --extra sdk` installs
  everything; a documented pip installation path remains available
  alongside it.
- Startup capability guard: the simulator now fails immediately with a
  readable message when the installed `limxsdk` lacks the `*ForSim`
  methods a robot type needs, instead of raising `AttributeError` on
  the first publish.
- Asset path fallback: models are located under either
  `robot-description/<tron2a|tron2b>/<TYPE>/xml/` or the flat
  `robot-description/tron2/<TYPE>/xml/`, easing the asset-repo
  transition.
- Support matrix in the README stating, per robot type, which upstream
  SDK and asset versions are required.
- `doc/gif/`: per-variant demo animations (`SF_TRON2A.gif`,
  `WF_TRON2A.gif`, `DACH_TRON2A.gif`, `DASF_TRON2A.gif`).
- Open-source scaffolding: `NOTICE`, `THIRD_PARTY_NOTICES.md`,
  `SECURITY.md`, `CONTRIBUTING.md`, `CHANGELOG.md`.
- `.gitignore` covering Python build artefacts (`__pycache__/`,
  `*.pyc`, `.venv/`, `.pytest_cache/`, `.ruff_cache/`,
  `*.egg-info/`), uv/simulator-specific artefacts (`MUJOCO_LOG.TXT`,
  `asset_converter/`), editor / OS junk, and an explicit deny-list for
  run-time artefacts that must not be committed here (`*.onnx`,
  `*.pt`, `*.pth`, `*.ckpt`, `*.so`, `*.dll`, `*.dylib`, `*.lib`,
  `*.whl`, `*.bag`, `*.mcap`).
- GitHub CI workflow: `uv lock --check`, Python byte-compile
  (`python -m py_compile`), `ruff` lint, a "no second messaging stack"
  check (bans importing `mros`), submodule status non-empty check,
  optional MuJoCo import smoke test, EXIF sanity scan on doc images,
  private-IP scan with allowlist, and a deny-list check that fails
  on any committed `__pycache__/`, `*.pyc`, `*.onnx`, `*.pt`,
  `*.so`, `*.dll`, `*.whl`, or `*.bag`.
- Issue templates and PR template under `.github/`; the PR template
  includes an explicit **submodule-pin-update** checkbox.
- `.github/CODEOWNERS` with maintainers / legal / SDK / sim /
  robotics team routing, extended to cover `tron2_sim/`,
  `pyproject.toml`, `uv.lock`, and `.python-version`.
- `README.md`: SPDX header, "License & attribution" block, "Scope /
  not included" (with submodule init note and pointer to the
  external `tron2-rl-deploy-python` ONNX repository), "Verification"
  section, and "Cite & support" block with `contact@limxdynamics.com`.
- README images under `doc/`: `banner.png` (1600x640),
  `social_preview.png` (1280x640, for GitHub Settings -> Social
  preview), model cards `model_{sf,wf,da,dach,dasf}.png` (800x800,
  rendered in MuJoCo from the pinned `robot-description` MJCF with one
  camera and light setup), and English / Chinese diagrams
  `architecture_{en,zh}.png`, `channels_{en,zh}.png` and
  `tron2a_vs_tron2b_{en,zh}.png`. All PNGs carry no metadata chunks.

### Changed
- **Communication is now limxsdk-only.** Every channel of every
  supported robot type runs over the SDK `*ForSim` API. The previous
  dual-stack arrangement, where channels the SDK did not cover fell
  back to a separate messaging library, is gone.
- `gripper_config.yaml` moved to `tron2_sim/config/`, next to the
  loader that reads it; `tron2_sim/config.py` became the
  `tron2_sim/config/` package, so the `from ..config import
  load_gripper_config` import is unchanged. `GRIPPER_CONFIG` still
  overrides the path.
- Comments, docstrings and runtime messages are in English.
- Submodule pins bumped to current public commits: `limxsdk-lowlevel`
  to `70ff83c22f5f54a07c2ddf7e32a2c6ffb1d2ebc7` and
  `robot-description` to `9939c22e69d27653ec0ba8a505859a2903dd1a71`
  (`https://github.com/limxdynamics/tron2-robot-description.git`,
  branch `main`).
- `README.md`, `README_zh-CN.md`, `SECURITY.md`, `CONTRIBUTING.md`,
  and the `.github/PULL_REQUEST_TEMPLATE.md` note now consistently
  describe `<robot-ip>` as a placeholder token that users substitute
  with their own robot / simulator IP. No production or
  internal-network IP is embedded in this repository.
- `README.md` / `README_zh-CN.md` restructured in parallel: banner,
  badges, highlights, model lineup, demos, a "How it works" diagram and
  a short quick start come first; the numbered reference sections
  follow, then license, scope, distribution and citation text, moved
  without wording changes. The English "Cite and support" section, left
  empty by the previous rewrite, is filled in to match the Chinese one.
  Corrected: the clone directory (`tron2_mujoco_sim`, was
  `tron2-mujoco-sim`), the `limxsdk` version (`4.0.2`, the wheel pinned
  in `limxsdk-lowlevel` and `pyproject.toml`; was `4.8+`), and the
  gamepad section now links `limxdynamics/tron2_rl_deploy_python`.

### Removed
- The `SFYG`, `WFYG` and `DASF2` robot types, whose arm and rear-body
  channels have no SDK equivalent.
- The bundled non-SDK messaging wheels and every module that depended
  on them: base-pose ground truth, teleoperation target markers, and
  the dexterous-hand module (which no shipped asset had joints for, so
  it never activated).
- `tools/` and the two root-level test clients; their role is taken
  over by CI.
- Committed `__pycache__/*.pyc` files that had been accidentally
  tracked despite `.gitignore` already covering `__pycache__/`.

### Fixed
- The previous `limxsdk-lowlevel` submodule pin referenced a commit
  predating the `*ForSim` API, so a fresh
  `git clone --recurse-submodules` produced a checkout that could not
  run any robot type. The pin now points at a public commit that
  supports it.
- `simulator.py` exits through `os._exit(0)` to dodge a teardown race
  in the SDK's native layer, which also skipped flushing buffered
  streams: all output was discarded whenever stdout was a pipe or a
  file. Streams are now flushed first.

### Blocked on upstream
- `DASF_TRON2B` needs the public asset repository to publish a
  `tron2b/` DASF model.

### Resolved (2026-07-16)
- **Private-IP handling** — resolved 2026-07-16 per owner decision.
  All Markdown / YAML command examples now use `<robot-ip>` as a
  placeholder token. No production IP is embedded in this
  repository. The sibling `tron2-rl-deploy-ros` retains a
  documentation-example literal `10.192.1.2` in its source / launch
  files, declared in that repo's `SECURITY.md`.

### Pending owner sign-off (blocks first public tag)
- **Submodule clearance — `robot-description`.** Pin now points at
  `9939c22e69d27653ec0ba8a505859a2903dd1a71` on branch `main` of
  `github.com/limxdynamics/tron2-robot-description`. Anonymous
  `git clone --recurse-submodules` succeeds. The **license
  (expected Apache-2.0 per the sibling repo `NOTICE`) and
  re-distribution terms** still require legal / mechanical owner
  sign-off before the first public tag. See `THIRD_PARTY_NOTICES.md` §2.
- **Submodule clearance — `robot-joystick`.** Confirm the pinned
  commit (`30f69a9…`), the license, and whether the submodule ships
  pre-built binaries subject to additional terms.
- **Submodule clearance — `limxsdk-lowlevel`.** Confirm the license
  and re-distribution terms for the wheels under
  `python3/{amd64,aarch64}/limxsdk-*.whl` at pin `70ff83c2…`.
- **Documentation media.** EXIF strip and content review of
  `doc/deploy.jpg`, `doc/sf.GIF`, `doc/wf.GIF`, the two
  `doc/*mj-*.gif` legacy simulation captures, and the new
  `doc/gif/*.gif` per-variant captures — including publication
  clearance from every identifiable person visible in the
  real-world deploy photo.
- **Linked external repository.** Confirm that
  `tron2-rl-deploy-python` will be published under the
  `limxdynamics` organization and that referencing its model paths
  in `README.md` is safe.

## [0.1.0] — TBD

First public release. Contents:

- `simulator.py` + `tron2_sim/`: MuJoCo simulator bridging LimX SDK
  `RobotCmd` (`q`, `dq`, `tau`, `Kp`, `Kd`) to `mujoco.MjData` for the
  `SF`, `WF`, `DA`, `DACH`, and `DASF` variants (`TRON2A`/`TRON2B`).
- Three git submodules (`robot-description`, `robot-joystick`,
  `limxsdk-lowlevel`) — pinned by commit, not vendored.
- Documentation under `doc/` and `README.md`.

[Unreleased]: https://github.com/limxdynamics/tron2_mujoco_sim/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/limxdynamics/tron2_mujoco_sim/releases/tag/v0.1.0
