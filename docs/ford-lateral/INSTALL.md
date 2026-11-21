# Installing the clean Ford steering build

For a comma 3X or a comma four. What the build does and why: [USER_GUIDE.md](USER_GUIDE.md).

**Checked:** the clone command below, on a computer (all three parts fetched, every submodule at its
pinned commit). **Not checked:** these steps on a comma four, and the clone command on a comma itself.

## What you need

- SSH access to the comma (Settings → Developer → Enable SSH, with your GitHub username's keys added).
  comma's guide: [docs/how-to/connect-to-comma.md](../how-to/connect-to-comma.md).
- Read access to the private repository **github.com/jrneulight/bluepilot-clean** for the same GitHub
  account. It holds all three parts of this build as branches: `clean/ford-lateral-pathangle`
  (BluePilot), `clean/opendbc-pathangle` and `clean/bluepilot-lateral-pathangle`. One recursive clone
  fetches all three. The build without path angle is `clean/ford-lateral`, the default branch.
- The comma on Wi-Fi. The download is about 1 GB plus the driving models.
- The build runs on AGNOS 19.9 (`launch_env.sh`), as sunnypilot and BluePilot bp-7.1-dev (its base) do; comma's current code needs 20.6 and BluePilot bp-dev 18.5. A comma on another AGNOS is moved to 19.9 at its first start, with the update screen.

## Install

### Step 1: connect, and keep your current install

On your computer:

```
ssh -A comma@<comma's IP address>
```

`-A` lets the comma use the GitHub key on your computer for this one session. Nothing is stored on the
comma. If the clone later says `Permission denied (publickey)`, run `ssh-add` on your computer and
connect again.

Then, on the comma:

```
cd /data
mv openpilot openpilot.before-clean
```

This is how you go back later. Don't delete it.

### Step 2: clone

On the comma, one line:

```
git -c url."git@github.com:jrneulight/".insteadOf="https://github.com/jrneulight/" clone --recurse-submodules --depth 1 --shallow-submodules -b clean/ford-lateral-pathangle git@github.com:jrneulight/bluepilot-clean.git openpilot
```

Answer `yes` if it asks whether to trust github.com. The `-c … insteadOf` part sends the build's two
private parts through your SSH key; the public parts (panda, msgq and the others) come over https. The
large files (driving models, images, fonts) come from sunnypilot's public large-file store.

**No SSH agent on your computer?** Clone there with the same line (git and git-lfs installed), then
copy it over: `rsync -a openpilot/ comma@<comma's IP address>:/data/openpilot/`.

### Step 3: reboot and let it build

```
sudo reboot
```

The first start builds the software on the device, which takes a while. **Not checked:** how long.

### Step 4: check it

Start the car. Look at **Settings → BluePilot → Lateral Tuning → Steering Support** (comma four: the
lateral page, **steering support**). It should show a level and your steering module's part numbers.
"Not read yet" means the car hasn't been started with this build.

## Update

Connect with `ssh -A` again, then on the comma:

```
cd /data/openpilot
git pull
git -c url."git@github.com:jrneulight/".insteadOf="https://github.com/jrneulight/" submodule update --init --recursive
sudo reboot
```

**Not checked:** what openpilot's own background updater does with a private repo it can't reach. It is
expected to fail to fetch and change nothing.

## Go back

On the comma:

```
cd /data
mv openpilot openpilot.clean
mv openpilot.before-clean openpilot
sudo reboot
```

Delete `/data/openpilot.clean` once you're sure you won't need it again.

**A lighter way back:** to keep this build and steer the way stock openpilot does, switch on **Disable
BP Lateral Control** in the same Lateral Tuning section. It takes effect at the next start.

## If something goes wrong

| Problem | Try |
|---|---|
| `Permission denied (publickey)` while cloning | Your GitHub account can't read the private repo, or the comma can't see your key: check `ssh-add -l` on your computer lists a key, and that you connected with `ssh -A` |
| A submodule fails with a username or password prompt | The `-c … insteadOf` part of the line is missing, so the private parts were asked for over https |
| The build fails on the device | Go back (above), then report the error screen's text |
| It loops at first start with an updater error (`module 'pyray' has no attribute 'GuiTextAlignment'`) | The update screen bundled before this build (built from older code) can't start on a newer AGNOS. This build ships the screen rebuilt from its own code, and if the screen still can't start, its launch script flashes AGNOS 19.9 without it: the display then stays as it is for several minutes while about 4.7 GB of AGNOS is written (downloaded compressed, without asking, over whatever connection the comma has, cellular included), and the comma reboots. comma's own build is not affected. On a build without either fix, run it by hand over SSH: `cd /data/openpilot && openpilot/common/hardware/comma/agnos.py --swap openpilot/system/hardware/comma/agnos.json && sudo reboot`. It writes the unused slot, checks it by hash and only then switches to it |
| Large files are missing or tiny text stubs | Run `git lfs pull` in `/data/openpilot` (or in the computer's clone, then copy again) |
