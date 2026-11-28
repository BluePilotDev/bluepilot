# Releasing BluePilot

How a BluePilot release is made, for the maintainer who makes it. A release is a tag: pushing `v7.2.0` is the act of
releasing, and a job does the rest. Only repo admins can push `v*` tags.

## The branches

| Branch | What it is | Who moves it |
|---|---|---|
| `bp-dev` | Where all work lands. Every new version is released from here. | Pull requests |
| `bp-7.1`, `bp-7.2`, ... | One per version: what that version's users install. Takes only fixes. | The release job creates it; fixes arrive by pull request |
| `bp-stable` | Always the highest version released. | The release job only; it never moves backward |

A version number is `MAJOR.MINOR.PATCH`. `7.2.0` is a **new version**; `7.2.1` is a **patch** to it.

## Releasing a new version (7.2.0)

1. **Open a pull request into `bp-dev`** that:
   - sets `BPVERSION` to `7.2.0`;
   - adds a `7.2.0` entry to `BP_CHANGES.json`. This is the "recent changes" dialog every device shows after the
     update, so write it for owners. An empty entry shows an empty dialog.
2. **Merge it and wait for the push checks on that commit to go green**: `build release`, `static analysis`,
   `unit tests` and `install smoke`.
3. **Push the tag on that commit:**
   ```bash
   git tag v7.2.0 <commit> && git push origin v7.2.0
   ```
4. **Watch the `bpx release` run** in the Actions tab. It creates `bp-7.2` at the tagged commit and moves
   `bp-stable` to it.
5. **Update the flasher** if this version runs a different AGNOS (see below).
6. **Tell people:** the install links in `README.md`, the short links, and the announcement on Discord.

## Releasing a patch (7.2.1)

A fix always lands on `bp-dev` first, so the next version has it too.

1. Merge the fix into `bp-dev`.
2. Open a pull request into `bp-7.2` that cherry-picks the fix, sets `BPVERSION` to `7.2.1` and adds a `7.2.1` entry
   to `BP_CHANGES.json`. The version branch requires the same six checks as `bp-dev`.
3. A fix to steering or safety is driven before it is tagged.
4. Merge it, wait for the push checks, and tag the **tip** of `bp-7.2`:
   ```bash
   git tag v7.2.1 <tip of bp-7.2> && git push origin v7.2.1
   ```
   `bp-stable` follows only if 7.2 is still the highest version. A patch to an older version leaves it alone.

## What the job refuses

The job checks everything before it writes anything; a refusal changes nothing and says what to do. Fix the cause,
then delete and re-push the tag, or run the job again from the Actions tab.

| It refuses when | Do this |
|---|---|
| The tag doesn't equal `v` + `BPVERSION` on the tagged commit | Tag the commit that has the right `BPVERSION` |
| A new version's commit isn't on `bp-dev` | Tag a `bp-dev` commit |
| The version branch already exists | A version is released once: release the change as a patch |
| A patch tag isn't on the tip of its version branch | Tag the tip |
| A patch's version branch doesn't exist | Release `.0` first |
| The commit's push checks aren't all green | Wait, or fix and tag the fixed commit |
| `bp-stable` already holds that version at another commit | A release is never re-pointed: release a patch |

The rules are in `.github/bp_release.py`, with tests in `bluepilot/tests/test_bp_release.py`.

## The flasher

[BluePilotDev/flash](https://github.com/BluePilotDev/flash) is the web flasher at
<https://BluePilotDev.github.io/flash/>. It installs the AGNOS (the device's operating system) that a chosen
BluePilot version was built for, so it has to know about every version.

- **A new version** gets an entry in the flasher, whether or not AGNOS changed.
- **A patch** needs a flasher change only if it changes AGNOS, which should be rare.

To add or update an entry (the flasher's README has the detail):

1. Copy `openpilot/common/hardware/comma/all-partitions.json` and `agnos.json` from the released commit into
   `public/manifests/bp-7.2/` in the flasher repo.
2. Add the version at the top of `VERSIONS` in `src/versions.js` with its AGNOS number (`AGNOS_VERSION` in
   `launch_env.sh`) and devices, and move `default: true` to it.
3. Run `bun run validate` and `bun run test`. Read the bootloader differences `validate` prints: flashing a
   different bootloader is the risky part.
4. Push to `master`; the site deploys itself.

Do this after the release job succeeds, so the flasher never offers a version that can't be installed.

## Testing the release job

Never test with real names. Run the workflow by hand from the Actions tab (`bpx release`, "Run workflow") with
throwaway names, all under `bpx-test/`:

| Input | Test value |
|---|---|
| `stable` | `bpx-test/stable` |
| `branch_prefix` | `bpx-test/bp-` |
| `tag_prefix` | `bpx-test/v` |
| `source` | a `bpx-test/...` branch, or leave as `bp-dev` |
| `dry_run` | on prints what it would do; off does it |

Push CI runs on `bpx-test/**` branches, so a test commit can get the green checks the job looks for. Delete the test
branches and tags afterward.

## Why it works this way

- **Only the default branch's copy of the workflow holds the key** that can move `bp-stable`. A tag push runs the
  tagged commit's copy, which can only ask the default branch's copy to run.
- **Branch rules back the job up:** `v*` tags are admin-only, `bp-stable` accepts only the release job, and version
  branches can't be deleted or force-pushed.

The design and its history are in [#227](https://github.com/BluePilotDev/bluepilot/issues/227).
