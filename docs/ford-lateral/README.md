# The clean Ford lateral build

BluePilot's Ford steering, rebuilt as one controller on curvature, with a table of Ford steering modules,
a fallback, a panda check on lateral acceleration and jerk, and path angle as a per-row switch (off on
every row in this build).

- [USER_GUIDE.md](USER_GUIDE.md): for owners. How Ford steering works, what this build changes and why,
  what you'll see, and the questions owners ask.
- [DEVELOPER_GUIDE.md](DEVELOPER_GUIDE.md): for people who work on it. Each design decision with its
  evidence and trade-offs, then the code path, the panda's checks, how a row is made and the tests.
- [INSTALL.md](INSTALL.md): installing on a comma 3X or comma four from the private repo, updating and
  going back.

The library's own docs: `opendbc_repo/opendbc/bluepilot_lateral/README.md` and `HOSTS.md`.
