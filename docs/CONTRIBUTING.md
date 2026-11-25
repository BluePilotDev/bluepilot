# Contributing to BluePilot

BluePilot is a Ford-focused fork of [sunnypilot](https://github.com/sunnypilot/sunnypilot), which is a fork of
[openpilot](https://github.com/commaai/openpilot). It steers and controls the speed of real cars, so the priorities
are safety, stability, quality and features, in that order ([SAFETY.md](SAFETY.md)).

Development is coordinated on GitHub and on the [BluePilot Discord](https://discord.gg/z8KAQ4qrkW). The project's
website is [bluepilot.dev](https://bluepilot.dev).

## Before you write code

1. **Find the right repo.** BluePilot is four repos, and Ford steering and Ford car code live outside this one. The
   routing table at the top of [AGENTS.md](../AGENTS.md) says where a change belongs and which branch to target.
2. **Ask first for anything large.** Open an issue or ask on Discord before a new feature, a new process, new
   third-party code or a change to driving behavior. A small, tested fix needs no discussion.
3. **Send upstream fixes upstream.** A bug in sunnypilot's or openpilot's own code is fixed there, and reaches
   BluePilot with the next sync.

[AGENTS.md](../AGENTS.md) holds the working rules: where code goes, how to change an upstream file, the tests, and
the mistakes that have already cost someone a PR. It is written for AI agents and for people; read it once.

## Pull requests

Open pull requests against `bp-dev`. The setup guide is in [tools/](../tools/).

A pull request that gets merged has:

* **One purpose.** Every changed line serves it. A feature and an unrelated fix are two pull requests.
* **A description that says why**, what changes for users who leave the feature off, and how it was tested.
* **All six required checks green**: `build release`, `static analysis`, `unit tests`, `process replay`,
  `install smoke`, `bluepilot-hooks`. A fork's checks start after a maintainer approves the run.
* **A `## Validation` checklist** at the end of the description, listing the six checks and any device or drive
  check, with only the passed ones ticked.
* **Evidence for driving changes**: the route, the car, the device, and what the logs show. Say plainly what was and
  wasn't driven. A change to what the car does is off by default unless it fixes existing behavior.
* **A device check for UI changes**, naming the device. The comma 3X and the comma four run different UI code.

Large pull requests are slow to review and easy to get wrong. If one grows past a few hundred lines, split it.

## Using AI tools

AI tools are welcome under the [AI policy](AI_POLICY.md): understand and test everything you submit, and disclose
AI help with an `Assisted-by:` line in the commit message.

## Contributing without code

* Report bugs in [GitHub issues](https://github.com/BluePilotDev/bluepilot/issues) with your car, model year, device,
  BluePilot version and, for a driving problem, the route.
* Test a new build before it is released and report what you find. Owners of Fords the maintainers don't drive are
  the only way those cars get checked.

## Thanks

BluePilot exists because comma and sunnypilot publish their work. Improvements that help every car belong upstream
with them.
