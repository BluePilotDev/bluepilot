# The clean Ford lateral build

BluePilot's Ford steering, rebuilt as one controller that steers by curvature. It has a table of known
Ford steering modules, a fallback for the rest, a panda check on lateral acceleration and jerk, and
path angle as a per-row switch. In this version it is on for the rows whose module's whole steering step has
run in an emulator: the 2021-23 F-150 on software BD and BA, the 2020-24 Explorer, the 2022-24 Maverick and the
2020-22 Corsair, Escape and Bronco Sport. It is off on the others and on the fallback.

- [USER_GUIDE.md](USER_GUIDE.md), for owners: how Ford steering works, what this build changes and why,
  what each message means and what to do.
- [DEVELOPER_GUIDE.md](DEVELOPER_GUIDE.md), for people who work on it: each design decision with its
  evidence and costs, then the code path, the panda's checks, how a row is made, and the tests.
- [INSTALL.md](INSTALL.md): installing on a comma 3X or comma four, updating, and going back.

The library's own docs are `opendbc_repo/opendbc/bluepilot_lateral/README.md` and `HOSTS.md`.
