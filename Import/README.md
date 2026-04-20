CARLA Simulator
===============

Place the standalone CARLA asset packages containing props and maps in this folder before running the import pipeline. For more information, see [Creating standalone asset packages for distribution](https://carla.readthedocs.io/en/latest/asset_packages_for_dist/).

For this UE5 branch, the expected flow is:

1. Copy the exported package tarballs, for example `*.tar.gz`, into this directory.
2. Extract them from the repository root with `./Util/ImportAssets.sh`.
3. Run the importer with the Unreal Engine path set, for example `UE5_ROOT=/path/to/UE_5.7 ./Util/BuildTools/Import.sh`.

The importer now resolves the active Unreal project automatically and writes imported content into the detected project `Content` directory, so the same command works for `CarlaUE5` on macOS.
