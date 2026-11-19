# The clean Ford lateral build

BluePilot's Ford steering, rebuilt as one controller that steers by curvature. It has a table of known
Ford steering modules, a fallback for the rest, a panda check on lateral acceleration and jerk, and
path angle as a per-row switch that is off on every row in this build.

- [USER_GUIDE.md](USER_GUIDE.md), for owners: how Ford steering works, what this build changes and why,
  what each message means and what to do.
- [DEVELOPER_GUIDE.md](DEVELOPER_GUIDE.md), for people who work on it: each design decision with its
  evidence and costs, then the code path, the panda's checks, how a row is made, and the tests.
- [INSTALL.md](INSTALL.md): installing on a comma 3X or comma four, updating, and going back.

The library's own docs are `opendbc_repo/opendbc/bluepilot_lateral/README.md` and `HOSTS.md`.
