# Installing the clean Ford steering build

For a comma 3X or a comma four. What the build does and why: [USER_GUIDE.md](USER_GUIDE.md).

**Not checked:** these steps on a comma four. **Not checked:** these steps end to end on any device.
They follow the repo's layout and the usual openpilot install by hand. Read each step before you run it.

## What you need

- SSH access to the comma (Settings → Developer → Enable SSH, with your GitHub username's keys added).
  comma's guide: [docs/how-to/connect-to-comma.md](../how-to/connect-to-comma.md).
- Access to the private repository **github.com/jrneulight/bluepilot-clean**. It holds all three parts
  of the build as branches: `clean/ford-lateral` (BluePilot, the default branch), `clean/opendbc` and
  `clean/bluepilot-lateral`. A recursive clone of `clean/ford-lateral` fetches the other two. The
  version with path angle on is `clean/ford-lateral-pathangle` (with `clean/opendbc-pathangle` and
  `clean/bluepilot-lateral-pathangle`): use it in place of `clean/ford-lateral` in the commands below.
- Free space on the comma for a second copy of openpilot while you keep the old one. **Not checked:**
  how much space this needs.

The repository is private, so the comma can't use the usual install URL. There are two ways to get it
onto the device.

## Install

### Step 1: keep your current install

On the comma (`ssh comma@<comma's IP address>`):

```
cd /data
mv openpilot openpilot.before-clean
```

This is how you go back later. Don't delete it.

### Step 2, way A (recommended): clone on a computer, copy to the comma

No GitHub credentials end up on the comma. On a computer that can read the private repo, with git and
git-lfs installed:

```
git clone --recurse-submodules -b clean/ford-lateral git@github.com:jrneulight/bluepilot-clean.git openpilot
rsync -a openpilot/ comma@<comma's IP address>:/data/openpilot/
```

**Not checked:** that `rsync` is on the comma; if it isn't, `scp -r` does the same, more slowly.

The large files (driving models, images, fonts) come from sunnypilot's public large-file store through
git-lfs. The repo's own settings point there.

### Step 2, way B: clone on the comma over SSH

This needs an SSH key on the comma that can read the private repo, for example a read-only deploy key
added to bluepilot-clean. One key covers all three branches, because they are in one repo. On the comma:

```
ssh-keygen -t ed25519 -f ~/.ssh/bluepilot_clean -N ""
cat ~/.ssh/bluepilot_clean.pub        # add this as a read-only deploy key on the repo
cat >> ~/.ssh/config <<'EOF'
Host github.com
  IdentityFile ~/.ssh/bluepilot_clean
EOF
git config --global url."git@github.com:jrneulight/".insteadOf "https://github.com/jrneulight/"
cd /data && git clone --recurse-submodules -b clean/ford-lateral git@github.com:jrneulight/bluepilot-clean.git openpilot
```

The `insteadOf` line is needed because the two submodules that point at the private repo are written
with `https://` addresses. It sends only `jrneulight` addresses through SSH. The public submodules
(panda, msgq and the others) still come over https.

**Not checked:** whether `~/.ssh` and `~/.gitconfig` on the comma survive a reboot or an OS update. If
they don't, way B's update step stops working and way A still does.

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

- **Way A:** on the computer, `git pull && git submodule update --init --recursive` in the clone, then
  run the `rsync` line again and reboot the comma.
- **Way B:** on the comma, `cd /data/openpilot && git pull && git submodule update --init --recursive`,
  then `sudo reboot`.

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
| `Permission denied (publickey)` while cloning | The key in use can't read the private repo: way A from a computer that can, or add the deploy key (way B) |
| A submodule fails with a username or password prompt | The `insteadOf` line from way B is missing, so the private submodules were asked for over https |
| The build fails on the device | Go back (above), then report the error screen's text |
| Large files are missing or tiny text stubs | git-lfs wasn't installed on the computer that cloned (way A). Install it, run `git lfs pull` in the clone, copy again |
