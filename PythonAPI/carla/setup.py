#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from __future__ import annotations

import contextlib
import fnmatch
import os
import shutil
import sys
from distutils.command.install_lib import install_lib
from typing import TYPE_CHECKING

from setuptools import Extension, setup

if TYPE_CHECKING:
    from collections.abc import Iterator


def is_rss_variant_enabled() -> bool:
    return bool("BUILD_RSS_VARIANT" in os.environ and os.environ["BUILD_RSS_VARIANT"] == "true")


def _build_posix_extensions(
    pwd: str,
    include_dirs: list[str],
    library_dirs: list[str],
    libraries: list[str],
    sources: list[str],
    walk_fn: object,
) -> list[Extension]:
    """Build extension list for POSIX (Linux) platforms."""
    import distro as _distro  # noqa: PLC0415
    supported_dists = ["ubuntu", "debian", "deepin"]
    linux_distro = _distro.id().lower()
    if linux_distro not in supported_dists:
        raise NotImplementedError(linux_distro + " not in supported posix platforms: " + str(supported_dists))

    pylib_static = f"libboost_python{sys.version_info.major}{sys.version_info.minor}.a"
    pylib_so = f"libboost_python{sys.version_info.major}{sys.version_info.minor}.so"
    # Prefer static .a, fallback to dynamic .so
    if os.path.exists(os.path.join(pwd, "dependencies/lib", pylib_static)):
        pylib = pylib_static
    elif os.path.exists(os.path.join(pwd, "dependencies/lib", pylib_so)):
        pylib = pylib_so
    else:
        pylib = pylib_static  # will error at link time

    if is_rss_variant_enabled():
        extra_link_args = [os.path.join(pwd, "dependencies/lib/libcarla_client_rss.a")]
    else:
        extra_link_args = [os.path.join(pwd, "dependencies/lib/libcarla_client.a")]

    extra_link_args += [
        os.path.join(pwd, "dependencies/lib/librpc.a"),
        os.path.join(pwd, "dependencies/lib/libboost_filesystem.a"),
        os.path.join(pwd, "dependencies/lib/libRecast.a"),
        os.path.join(pwd, "dependencies/lib/libDetour.a"),
        os.path.join(pwd, "dependencies/lib/libDetourCrowd.a"),
        os.path.join(pwd, "dependencies/lib/libosm2odr.a"),
        os.path.join(pwd, "dependencies/lib/libxerces-c.a"),
    ]
    extra_link_args += ["-lz"]
    extra_compile_args = [
        "-isystem", os.path.join(pwd, "dependencies/include/system"), "-fPIC", "-std=c++14",
        "-Werror", "-Wall", "-Wextra", "-Wpedantic", "-Wno-self-assign-overloaded",
        "-Wdeprecated", "-Wno-shadow", "-Wuninitialized", "-Wunreachable-code",
        "-Wpessimizing-move", "-Wold-style-cast", "-Wnull-dereference",
        "-Wduplicate-enum", "-Wnon-virtual-dtor", "-Wheader-hygiene",
        "-Wconversion", "-Wfloat-overflow-conversion",
        "-Wno-error=unused-command-line-argument",
        "-DBOOST_ERROR_CODE_HEADER_ONLY", "-DLIBCARLA_WITH_PYTHON_SUPPORT",
        "-stdlib=libstdc++",
    ]
    if is_rss_variant_enabled():
        py_ver = f"{sys.version_info.major}{sys.version_info.minor}"
        dep_lib = os.path.join(pwd, "dependencies/lib")
        extra_compile_args += [
            "-DLIBCARLA_RSS_ENABLED",
            f"-DLIBCARLA_PYTHON_MAJOR_{sys.version_info.major}",
        ]
        extra_link_args += [
            os.path.join(dep_lib, f"libad_rss_map_integration_python{py_ver}.a"),
            os.path.join(dep_lib, "libad_rss_map_integration.a"),
            os.path.join(dep_lib, f"libad_map_access_python{py_ver}.a"),
            os.path.join(dep_lib, "libad_map_access.a"),
            os.path.join(dep_lib, f"libad_rss_python{py_ver}.a"),
            os.path.join(dep_lib, "libad_rss.a"),
            os.path.join(dep_lib, f"libad_physics_python{py_ver}.a"),
            os.path.join(dep_lib, "libad_physics.a"),
            os.path.join(dep_lib, "libad_map_opendrive_reader.a"),
            os.path.join(dep_lib, "libboost_program_options.a"),
            os.path.join(dep_lib, "libodrSpiral.a"),
            os.path.join(dep_lib, "libspdlog.a"),
            "-lrt", "-ltbb",
        ]

    # libproj, libsqlite and python libs are also required for rss_variant, therefore
    # place them after the rss_variant linked libraries
    extra_link_args += [
        os.path.join(pwd, "dependencies/lib/libproj.a"),
        os.path.join(pwd, "dependencies/lib/libsqlite3.a"),
        os.path.join(pwd, "dependencies/lib", pylib),
    ]

    if "TRAVIS" in os.environ and os.environ["TRAVIS"] == "true":
        extra_link_args += ["-ljpeg", "-ltiff"]
        extra_compile_args += ["-DLIBCARLA_IMAGE_WITH_PNG_SUPPORT=false"]
    else:
        extra_link_args += ["-lpng", "-ljpeg", "-ltiff"]
        extra_compile_args += ["-DLIBCARLA_IMAGE_WITH_PNG_SUPPORT=true"]
    extra_link_args += ["-lstdc++"]

    depends = list(walk_fn("source/libcarla"))  # type: ignore[operator]
    depends += list(walk_fn("dependencies"))  # type: ignore[operator]

    def make_extension(name: str, sources: list[str]) -> Extension:
        return Extension(
            name,
            sources=sources,
            include_dirs=include_dirs,
            library_dirs=library_dirs,
            libraries=libraries,
            extra_compile_args=extra_compile_args,
            extra_link_args=extra_link_args,
            language="c++14",
            depends=depends)

    return [make_extension("carla.libcarla", sources)]


def _build_nt_extensions(
    pwd: str,
    include_dirs: list[str],
    library_dirs: list[str],
    libraries: list[str],
    sources: list[str],
    walk_fn: object,
) -> list[Extension]:
    """Build extension list for Windows (NT) platforms."""
    pylib = f"libboost_python{sys.version_info.major}{sys.version_info.minor}"

    extra_link_args = ["shlwapi.lib", "Advapi32.lib", "ole32.lib", "shell32.lib"]

    required_libs = [
        pylib, "libboost_filesystem",
        "rpc.lib", "carla_client.lib",
        "libpng.lib", "zlib.lib",
        "Recast.lib", "Detour.lib", "DetourCrowd.lib",
        "xerces-c_3.lib", "sqlite3.lib",
        "proj.lib", "osm2odr.lib"]

    # Search for files in 'PythonAPI\carla\dependencies\lib' that contains
    # the names listed in required_libs in it's file name
    libs = [x for x in os.listdir("dependencies/lib") if any(d in x for d in required_libs)]

    extra_link_args.extend(os.path.join(pwd, "dependencies/lib", lib) for lib in libs)

    # https://docs.microsoft.com/es-es/cpp/porting/modifying-winver-and-win32-winnt
    extra_compile_args = [
        "/experimental:external", "/external:W0", "/external:I", "dependencies/include/system",
        "/DBOOST_ALL_NO_LIB", "/DBOOST_PYTHON_STATIC_LIB",
        "/DBOOST_ERROR_CODE_HEADER_ONLY", "/D_WIN32_WINNT=0x0600", "/DHAVE_SNPRINTF",
        "/DLIBCARLA_WITH_PYTHON_SUPPORT", "-DLIBCARLA_IMAGE_WITH_PNG_SUPPORT=true", "/MD"]

    depends = list(walk_fn("source/libcarla"))  # type: ignore[operator]
    depends += list(walk_fn("dependencies"))  # type: ignore[operator]

    def make_extension(name: str, sources: list[str]) -> Extension:
        return Extension(
            name,
            sources=sources,
            include_dirs=include_dirs,
            library_dirs=library_dirs,
            libraries=libraries,
            extra_compile_args=extra_compile_args,
            extra_link_args=extra_link_args,
            language="c++14",
            depends=depends)

    return [make_extension("carla.libcarla", sources)]


def get_libcarla_extensions() -> list[Extension]:
    include_dirs = ["dependencies/include"]
    library_dirs = ["dependencies/lib"]
    libraries: list[str] = []
    sources = ["source/libcarla/libcarla.cpp"]

    def walk(folder: str, file_filter: str = "*") -> Iterator[str]:
        for root, _, filenames in os.walk(folder):
            for filename in fnmatch.filter(filenames, file_filter):
                yield os.path.join(root, filename)

    pwd = os.path.dirname(os.path.realpath(__file__))

    if os.name == "posix":
        return _build_posix_extensions(pwd, include_dirs, library_dirs, libraries, sources, walk)
    if os.name == "nt":
        return _build_nt_extensions(pwd, include_dirs, library_dirs, libraries, sources, walk)
    raise NotImplementedError


def get_license() -> str:
    if is_rss_variant_enabled():
        return "LGPL-v2.1-only License"
    return "MIT License"

with open("README.md") as f:
    long_description = f.read()

class CleanADStubFiles(install_lib):
    """
    Removes the ad/ files from the build directory in a normal built
    that else would be copies over.
    """

    CARLA_RSS_STUB_FILE = "__carla_rss.pyi"
    _CARLA_RSS_STUB_FILE_PATH = os.path.join("carla", CARLA_RSS_STUB_FILE)
    CARLA_AD_STUB_DIR = os.path.join("carla", "ad")

    def run(self) -> None:
        if not is_rss_variant_enabled():
            self.prune_rss()
        install_lib.run(self)  # for python2 do not use super here

    def prune_rss(self) -> None:
        """Removes files from an rss build that we do not want to be copied over to a non-rss build."""
        if not self.build_dir:
            return
        shutil.rmtree(os.path.join(self.build_dir, self.CARLA_AD_STUB_DIR), ignore_errors=True)
        with contextlib.suppress(OSError):
            os.remove(os.path.join(self.build_dir, self._CARLA_RSS_STUB_FILE_PATH))
        return

setup(
    name="carla",
    version="0.9.16",
    package_dir={"": "source"},
    # Avoid "Package would be ignored" warning for non-rss build if using ['carla'] here
    packages=["carla", "carla.ad", "carla.ad.rss", "carla.ad.map"] if is_rss_variant_enabled() else ["carla"],
    # For non-rss build do a fine grained include/exclude on the package data.
    package_data={"carla" : [""]},
    exclude_package_data={} if is_rss_variant_enabled() else {"carla":[CleanADStubFiles.CARLA_RSS_STUB_FILE]},
    ext_modules=get_libcarla_extensions(),
    license=get_license(),
    description="Python API for communicating with the CARLA server.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/carla-simulator/carla",
    author="The CARLA team",
    author_email="carla.simulator@gmail.com",
    include_package_data=True,
    cmdclass={"install_lib": CleanADStubFiles},
    )
