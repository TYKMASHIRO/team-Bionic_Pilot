# team-Bionic_Pilot

Initial repository setup.

See [CONTRIBUTING.md](CONTRIBUTING.md) for branch and code review rules.

## SDK directories

- `linkerhand-cpp-sdk`: LinkerHand C++ SDK with O6 support, tracked as a Git submodule.
- `RM_API2`: RealMan API2 SDK with C/C++ support for RM75-6F, vendored as regular files without the top-level `Python` directory.

After cloning this repository, initialize the LinkerHand SDK submodule with:

```bash
git submodule update --init --recursive linkerhand-cpp-sdk
```
