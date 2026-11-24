# Bringing BluePilot lateral to a fork

How an openpilot fork takes BluePilot's Ford lateral control. There are two ways to do it:

1. **Use a release on your own** (preferred). Pin a release of this library and write your own hooks and settings against [HOSTS.md](HOSTS.md). BluePilot isn't in the loop and nothing waits on BluePilot: you consume lateral code that has been driven on Fords.
2. **Work with BluePilot.** BluePilot helps develop your hooks, keeps them in this repo, and tests the library on your fork in its CI, every PR and every night.

Either way the library is the same, and the same rules apply to it (below).

**This is a modified Ford safety model.** The panda safety allows signals and ranges that upstream's Ford safety blocks. Your fork decides whether to ship it, and whether BluePilot lateral is on or off by default for your users.

## The rules for either way

- **Pin a release, unmodified.** The library sits at `<pkg>/bluepilot_lateral/` in your opendbc (`<pkg>` is its package name) as one of:
  - a **git submodule** pointing at a release tag, if your fork uses submodules; updating is moving the pointer;
  - a **stamped copy** made with `tools/vendor.sh`, if it doesn't (see "The stamp tools").
- **Don't edit inside the library folder**, carry it as a git subtree with its history, or copy parts of it elsewhere. A fix to the library is a PR to this repo; once it's released, every fork gets it.
- **Everything else is yours:** the hooks in your opendbc, the code in your openpilot that fills the library's inputs every frame, and your settings UI.

## Option 1: use a release on your own

1. Pick a release from this repo's releases. Each one says what changed in the lateral control, the CAN messages and the panda safety, and what was validated on a car.
2. Mount it (submodule or stamped copy, above).
3. Write the hooks in your opendbc. [HOSTS.md](HOSTS.md) has the contract: the hook sites, the per-frame inputs and settings, the startup safety bits, and the host bindings to define if your safety function names differ from upstream opendbc's. Each release attaches the hook patch for sunnypilot/opendbc as a worked example.
4. Write your adapter: the small module that hands the library your fork's types (`hosts/sunnypilot/` is the example to copy). It lives in your fork, outside the library folder.
5. Run the library's tests in your tree ([HOSTS.md](HOSTS.md), "Running the tests") and drive it before you ship it.
6. To update, move to a newer release and rerun your tests.

BluePilot doesn't review or approve any of it. Questions and bugs in the library are welcome as issues in this repo.

## Option 2: work with BluePilot

For a fork that wants BluePilot to help with the hooks and keep them working. Open an issue in this repo, "Host: <fork>": where the fork's source and integration branch are, how it carries its opendbc, and how it takes outside changes. From there, this repo keeps for that fork:

- **its adapter**, `hosts/<fork>/`, plus host bindings if its safety names differ from upstream's;
- **its hooks**, `hosts/<fork>/hooks.patch`, until the fork carries them;
- **`hosts/<fork>/host.sh`**: `tree <dir>` builds the fork's opendbc at a pinned commit with the hooks and the library, placed by `vendor.sh`; `test <dir>` runs the library tests, the fork's safety suite with the BluePilot safety tests, and coverage; `make <dir>` writes `hooks.patch` back after a hook is edited in `<dir>/host`;
- **a CI job** that runs `host.sh tree` and `test` on every PR to this repo, so a library change that would break the fork fails here first;
- **a nightly check** that tries the fork's latest integration branch: a pass moves the pinned commit, a failure opens one issue saying which step broke. Once the fork carries the hooks it tests the fork's tree as it is, and runs `check_vendored.sh` on its copy.

The steps:
1. Develop and test the hooks and the fork's side in the fork, together.
2. Add the adapter, `hooks.patch`, `host.sh`, the CI job and the nightly check here, in one PR.
3. Release the library with them.
4. BluePilot opens the PR to the fork: the hooks, its settings UI and the library at that release. Later releases arrive the same way, as PRs the fork takes when it chooses.

An adapter lives in this repo only if this repo's CI tests the library on that fork.

## The stamp tools

For a fork that takes no submodules. Both run from a clone of this repo that has the release tag (`git fetch --tags`).

**`tools/vendor.sh <tag> <folder>`** makes `<folder>` exactly the library at `<tag>`:
- it empties the folder first, so files a release removed don't linger;
- it copies the release's files, without the library's own CI (`.github/`);
- it writes `<folder>/VERSION`: the tag and the commit, e.g. `v1.2.3 0a1b2c3d...`.

```bash
git clone https://github.com/BluePilotDev/bluepilot-lateral.git
cd bluepilot-lateral
tools/vendor.sh <tag> <fork checkout>/<path to its opendbc package>/bluepilot_lateral
```

Commit the folder in the fork. To update, run it again with the new tag: the diff shows exactly what changed between the two releases.

**`tools/check_vendored.sh <folder>`** reads `<folder>/VERSION` and compares the folder with that release, file by file (Python caches ignored). It lists every added, removed or changed file, and exits non-zero if there is one:

```bash
tools/check_vendored.sh <fork checkout>/<path to its opendbc package>/bluepilot_lateral
```

Run it in your CI to catch an edit to the copy. For a fork working with BluePilot, this repo's nightly check runs it too.

## Installer note: nested submodule

A fork that embeds the library as a *nested* submodule (Option 2 — e.g. `fork → opendbc → opendbc/bluepilot_lateral`) can end up with the library missing after an end-user install. Some installers run a non-recursive `git submodule update --init`, which checks out the first-level submodule but not the library nested inside it, and the build then fails to import it. Installs that use `git submodule update --init --recursive` are unaffected.

If your fork is installed that way, self-heal on launch: before the build imports the library, when its directory is empty, run a recursive submodule update scoped to the embedding submodule — guard on the library's `__init__.py`, retry, and fail soft so an offline boot is no worse than before.

## How BluePilot releases

- Lateral changes and changes to the library's safety headers are driven on a car before they're released.
- Every release says what changed in the lateral control, the CAN messages and the panda safety, and what was validated.
