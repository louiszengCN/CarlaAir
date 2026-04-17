#!/usr/bin/env python3

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
# de Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from __future__ import annotations

import os
import sys
import tarfile
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Generator

    from docker.models.containers import Container

BLUE = "\033[94m"
GREEN = "\033[92m"
RED = "\033[91m"
ENDC = "\033[0m"
BOLD = "\033[1m"
UNDERLINE = "\033[4m"


class ReadableStream:

    def __init__(self, generator: Generator[bytes, None, None]) -> None:
        self._generator = generator

    def read(self) -> bytes:
        return next(self._generator)


def get_container_name(container: Container) -> str:
    return str(container.attrs["Config"]["Image"])


def exec_command(
    container: Container,
    command: str,
    user: str = "root",
    *,
    silent: bool = False,
    verbose: bool = False,
    ignore_error: bool = True,
) -> object:
    full_cmd = f"bash -c '{command}'"
    if not silent and verbose:
        print(f"[docker] Running: {full_cmd}")

    command_result = container.exec_run(full_cmd, user=user)

    if not silent:
        out = command_result.output.decode().strip()
        if out:
            print(out)
        if verbose and command_result.exit_code:
            print(f"[docker] Command exited with code {command_result.exit_code}", file=sys.stderr)

    if not ignore_error and command_result.exit_code:
        sys.exit(command_result.exit_code)

    return command_result


def get_file_paths(
    container: Container,
    path: str,
    user: str = "root",
    *,
    absolute_path: bool = True,
    hidden_files: bool = True,
    verbose: bool = False,
) -> list[str]:
    command = "ls "
    if hidden_files:
        command += "-a "
    if absolute_path:
        command += "-d "
    result = exec_command(container, command + path, user=user, silent=True)
    if result.exit_code:
        if verbose:
            print(f"[docker] get_file_paths failed for {path}", file=sys.stderr)
        return []
    file_list = [x for x in result.output.decode("utf-8").split("\n") if x]
    if verbose:
        print(f"[docker] Found {len(file_list)} file(s) at {path}")
    return file_list


def extract_files(container: Container, file_list: list[str], out_path: str) -> None:
    for file in file_list:
        tmp_tar = os.path.join(out_path, "result.tar.gz")
        strm, _ = container.get_archive(file)
        with open(tmp_tar, "wb") as f:
            for chunk in strm:
                f.write(chunk)
        with tarfile.open(tmp_tar) as pw_tar:
            pw_tar.extractall(out_path)
        os.remove(tmp_tar)
