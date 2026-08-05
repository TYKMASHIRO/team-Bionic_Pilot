# team-Bionic_Pilot

Initial repository setup.

See [CONTRIBUTING.md](CONTRIBUTING.md) for branch and code review rules.

## SDK submodules

- `linkerhand-cpp-sdk`: LinkerHand C++ SDK with O6 support, tracked as a Git submodule.
- `RM_API2`: RealMan API2 SDK with C/C++ support for RM75-6F, tracked as a Git submodule from the `Yubo-Cui/RM_API2` `dev/cyb` branch. That branch removes the top-level `Python` SDK directory.

After cloning this repository, initialize the SDK submodules with:

```bash
git submodule update --init --recursive
```
