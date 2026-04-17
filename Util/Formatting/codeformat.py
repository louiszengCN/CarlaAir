#!/usr/bin/python

#
# Copyright (c) 2017-2020 Intel Corporation
#
# Helper script for code formatting using clang-format-3.9 and autopep

from __future__ import annotations

import argparse
import filecmp
import os
import re
import subprocess
import sys

import sets
from termcolor import cprint

SCRIPT_VERSION = "1.3"


class CodeFormatter:

    def __init__(
        self,
        command: str,
        expected_version: str,
        format_command_arguments: list[str],
        verify_command_arguments: list[str],
        *,
        verify_output_is_diff: bool,
        file_endings: list[str],
        file_description: str,
        install_command: str,
    ) -> None:
        self.command = command
        self.expected_version = expected_version
        self.format_command_arguments = format_command_arguments
        self.verify_command_arguments = verify_command_arguments
        self.verify_output_is_diff = verify_output_is_diff
        self.file_endings = file_endings
        self.file_description = file_description
        self.install_command = install_command

    def verify_formatter_version(self) -> None:
        try:
            version_output = subprocess.check_output([self.command, "--version"]).rstrip("\r\n")
            if self.expected_version != "":
                if version_output.startswith(self.expected_version):
                    return
                cprint("[NOT OK] Found '" + version_output + "'", "red")
                cprint("[NOT OK] Version string does not start with '" + self.expected_version + "'", "red")
            else:
                return
        except OSError:
            cprint("[ERROR] Could not run " + self.command, "red")
            cprint("[INFO] Please check if correct version is installed or install with '" +
                   self.install_command + "'", "blue")
        sys.exit(1)

    def print_input_files(self) -> None:
        if len(self.input_files) > 0:
            for _file_name in self.input_files:
                pass

    def format_file(self, file_name: str) -> bool:
        command_list = [self.command]
        command_list.extend(self.format_command_arguments)
        command_list.append(file_name)
        try:
            subprocess.check_output(command_list, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as e:
            cprint("[ERROR] " + file_name + " (" + e.output.rstrip("\r\n") + ")", "red")
            return True
        return False

    def perform_git_diff(self, file_name: str, verify_output: bytes) -> tuple[bool, bytes | str]:
        try:
            diff_process = subprocess.Popen(
                ["git", "diff", "--color=always", "--exit-code", "--no-index", "--", file_name, "-"],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            diff_output, _ = diff_process.communicate(verify_output)
            if diff_process.returncode == 0:
                diff_output = ""
        except OSError:
            cprint("[ERROR] Failed to run git diff on " + file_name, "red")
            return (True, "")
        return (False, diff_output)

    def verify_file(self, file_name: str, *, print_diff: bool) -> bool:
        command_list = [self.command]
        command_list.extend(self.verify_command_arguments)
        command_list.append(file_name)
        try:
            verify_output = subprocess.check_output(command_list, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as e:
            cprint("[ERROR] " + file_name + " (" + e.output.rstrip("\r\n") + ")", "red")
            return True

        diff_output: bytes | str = ""
        if self.verify_output_is_diff:
            diff_output = verify_output
        else:
            status, diff_output = self.perform_git_diff(file_name, verify_output)
            if status:
                return True

        if diff_output != "":
            cprint("[NOT OK] " + file_name, "red")
            if print_diff:
                pass
            return True

        return False


class CodeFormatterClang(CodeFormatter):
    CLANG_FORMAT_FILE = ".clang-format"
    CHECKED_IN_CLANG_FORMAT_FILE = "clang-format"
    CODE_FORMAT_IGNORE_FILE = ".codeformatignore"

    def __init__(self) -> None:
        CodeFormatter.__init__(self,
                               command="clang-format-3.9",
                               expected_version="clang-format version 3.9",
                               format_command_arguments=["-style=file", "-fallback-style=none", "-i"],
                               verify_command_arguments=["-style=file", "-fallback-style=none"],
                               verify_output_is_diff=False,
                               file_endings=["cpp", "hpp", "c", "h", "cc"],
                               file_description="source and header",
                               install_command="sudo apt-get install clang-format-3.9")
        self.script_path = os.path.dirname(os.path.abspath(__file__))
        self.checked_in_clang_format_file = os.path.join(
            self.script_path, CodeFormatterClang.CHECKED_IN_CLANG_FORMAT_FILE,
        )

    def verify_formatter_version(self) -> None:
        CodeFormatter.verify_formatter_version(self)
        self.verify_clang_format_file_exists_and_matches_checked_in()

    def verify_checked_in_clang_format_file_exists(self) -> None:
        if os.path.exists(self.checked_in_clang_format_file):
            pass
        else:
            cprint("[WARN] Not found " + CodeFormatterClang.CHECKED_IN_CLANG_FORMAT_FILE + " file " +
                   self.checked_in_clang_format_file, "yellow")
            self.confirm_with_user_clang_format_file_cant_be_verified()

    def confirm_with_user_clang_format_file_cant_be_verified(self) -> None:
        if not self.args.yes:
            answer = input("Are you sure your .clang-format file is up-to-date and you want to continue? (y/N)")
            if answer != "y":
                sys.exit(1)

    def verify_clang_format_file_exists_and_matches_checked_in(self) -> None:
        self.verify_checked_in_clang_format_file_exists()
        found_clang_format_files = sets.Set()
        for file_name in self.input_files:
            dir_name = os.path.dirname(os.path.abspath(file_name))
            if not self.find_clang_format_file_starting_from(dir_name, file_name, found_clang_format_files):
                sys.exit(1)

    def find_clang_format_file_starting_from(
        self,
        dir_name: str,
        file_name: str,
        found_clang_format_files: sets.Set,
    ) -> bool:
        clang_format_file = os.path.join(dir_name, CodeFormatterClang.CLANG_FORMAT_FILE)
        if os.path.exists(clang_format_file):
            if clang_format_file not in found_clang_format_files:
                found_clang_format_files.add(clang_format_file)
                if os.path.exists(self.checked_in_clang_format_file) and \
                   not filecmp.cmp(self.checked_in_clang_format_file, clang_format_file):
                    cprint(
                        "[WARN] " + clang_format_file + " does not match "
                        + self.checked_in_clang_format_file,
                        "yellow",
                    )
                    self.confirm_with_user_clang_format_file_cant_be_verified()
                else:
                    pass
            return True
        dir_name_one_level_up = os.path.dirname(dir_name)
        if dir_name_one_level_up == dir_name:
            # dir_name was already root folder -> clang-format file not found
            cprint("[ERROR] Not found " + CodeFormatterClang.CLANG_FORMAT_FILE + " for " +
                   file_name + " in same directory or in any parent directory", "red")
            return False
        return self.find_clang_format_file_starting_from(dir_name_one_level_up, file_name, found_clang_format_files)


class CodeFormatterAutopep(CodeFormatter):

    def __init__(self) -> None:
        CodeFormatter.__init__(self,
                               command="autopep8",
                               expected_version="",
                               format_command_arguments=["--in-place", "--max-line-length=119"],
                               verify_command_arguments=["--diff", "--max-line-length=119"],
                               verify_output_is_diff=True,
                               file_endings=["py"],
                               file_description="python",
                               install_command="sudo apt-get install python-pep8 python-autopep8")


class CodeFormat:

    def __init__(self) -> None:
        self.failure = False
        self.code_formatter_instances: list[CodeFormatter] = []

    def parse_command_line(self) -> None:
        parser = argparse.ArgumentParser(
            description="Helper script for code formatting.")
        parser.add_argument("input", nargs="+",
                            help="files or directories to process")
        parser.add_argument("-v", "--verify", action="store_true",
                            help="do not change file, but only verify the format is correct")
        parser.add_argument("-d", "--diff", action="store_true",
                            help="show diff, implies verify mode")
        parser.add_argument("-e", "--exclude", nargs="+", metavar="exclude",
                            help="exclude files or directories containing words from the exclude list in their names")
        parser.add_argument("-y", "--yes", action="store_true",
                            help="do not ask for confirmation before formatting more than one file")
        parser.add_argument("--version", action="version", version="%(prog)s " + SCRIPT_VERSION)
        self.args = parser.parse_args()
        if self.args.diff:
            self.args.verify = True

    def add_code_formatter(self, code_formatter_instance: CodeFormatter) -> None:
        self.code_formatter_instances.append(code_formatter_instance)

    def scan_for_input_files(self) -> None:
        for formatter_instance in self.code_formatter_instances:
            file_pattern = re.compile(r"^[^.].*\.(" + "|".join(formatter_instance.file_endings) + ")$")
            formatter_instance.input_files = []
            for file_or_directory in self.args.input:
                if os.path.exists(file_or_directory):
                    formatter_instance.input_files.extend(self.scan_file_or_directory(file_or_directory, file_pattern))
                else:
                    cprint("[WARN] Cannot find '" + file_or_directory + "'", "yellow")

    def scan_file_or_directory(self, file_or_directory: str, file_pattern: re.Pattern[str]) -> list[str]:
        file_list = []
        if os.path.isdir(file_or_directory):
            for root, directories, file_names in os.walk(file_or_directory):
                directories[:] = self.filter_directories(root, directories)
                for filename in filter(file_pattern.match, file_names):
                    full_filename = os.path.join(root, filename)
                    if self.is_file_not_excluded(full_filename):
                        file_list.append(full_filename)
        elif self.is_file_not_excluded(file_or_directory) and (
            file_pattern.match(os.path.basename(file_or_directory)) is not None
        ):
            file_list.append(file_or_directory)
        return file_list

    def filter_directories(self, root: str, directories: list[str]) -> list[str]:
        # Exclude hidden directories and all directories that have a CODE_FORMAT_IGNORE_FILE
        directories[:] = [directory for directory in directories if
                          not directory.startswith(".") and
                          not os.path.exists(os.path.join(root, directory, CodeFormatterClang.CODE_FORMAT_IGNORE_FILE))]
        return directories

    def is_file_not_excluded(self, file_name: str) -> bool:
        if self.args.exclude is not None:
            for excluded in self.args.exclude:
                if excluded in file_name:
                    return False
        return not os.path.islink(file_name)

    def confirm_with_user_file_is_outside_git(self, file_name: str) -> None:
        cprint("[WARN] File is not in a Git repo: " + file_name, "yellow")
        answer = input("Are you sure to code format it anyway? (y/Q)")
        if answer != "y":
            sys.exit(1)

    def confirm_with_user_file_is_untracked(self, file_name: str) -> None:
        cprint("[WARN] File is untracked in Git: " + file_name, "yellow")
        answer = input("Are you sure to code format it anyway? (y/Q)")
        if answer != "y":
            sys.exit(1)

    def confirm_with_user_git_repo_is_not_clean(self, git_repo: str) -> None:
        cprint("[WARN] Git repo is not clean: " + git_repo, "yellow")
        answer = input("Are you sure to code format files in it anyway? (y/Q)")
        if answer != "y":
            sys.exit(1)

    def check_input_files_are_in_clean_git_repos_and_are_tracked(self) -> None:
        if self.args.verify or self.args.yes:
            return
        git_repos = sets.Set()
        for formatter_instance in self.code_formatter_instances:
            for file_name in formatter_instance.input_files:
                git_repo = self.get_git_repo_for_file(file_name)
                if git_repo is None:
                    self.confirm_with_user_file_is_outside_git(file_name)
                else:
                    self.git_update_index_refresh(git_repo)
                    if not self.is_tracked_file(file_name):
                        self.confirm_with_user_file_is_untracked(file_name)
                    elif git_repo not in git_repos:
                        git_repos.add(git_repo)
                        if not self.is_clean_git_repo(git_repo):
                            self.confirm_with_user_git_repo_is_not_clean(git_repo)

    def get_git_repo_for_file(self, file_name: str) -> str | None:
        if not self.is_inside_git_repo(file_name):
            return None
        try:
            git_process = subprocess.Popen(["git", "rev-parse", "--show-toplevel"],
                                           stdin=subprocess.PIPE,
                                           stdout=subprocess.PIPE,
                                           stderr=subprocess.PIPE,
                                           cwd=os.path.dirname(file_name))
            git_output, _ = git_process.communicate()
            if git_process.returncode == 0:
                return git_output.rstrip("\r\n")
        except OSError:
            cprint("[ERROR] Failed to run 'git rev-parse --show-toplevel' for " + file_name, "red")
        return None

    def is_inside_git_repo(self, file_name: str) -> bool:
        try:
            git_process = subprocess.Popen(["git", "rev-parse", "--is-inside-work-tree"],
                                           stdin=subprocess.PIPE,
                                           stdout=subprocess.PIPE,
                                           stderr=subprocess.PIPE,
                                           cwd=os.path.dirname(file_name))
            git_output, _ = git_process.communicate()
            if git_process.returncode == 0:
                return git_output.rstrip("\r\n") == "true"
        except OSError:
            cprint("[ERROR] Failed to run 'git rev-parse --is-inside-work-tree' for " + file_name, "red")
        return False

    def is_tracked_file(self, file_name: str) -> bool:
        try:
            git_process = subprocess.Popen(["git", "ls-files", "--error-unmatch", "--", os.path.basename(file_name)],
                                           stdin=subprocess.PIPE,
                                           stdout=subprocess.PIPE,
                                           stderr=subprocess.PIPE,
                                           cwd=os.path.dirname(file_name))
            _, _ = git_process.communicate()
            if git_process.returncode == 0:
                return True
        except OSError:
            cprint("[ERROR] Failed to run 'git ls-files --error-unmatch' for " + file_name, "red")
        return False

    def is_clean_git_repo(self, git_repo: str) -> bool:
        try:
            git_process = subprocess.Popen(["git", "diff-index", "--quiet", "HEAD", "--"],
                                           stdin=subprocess.PIPE,
                                           stdout=subprocess.PIPE,
                                           stderr=subprocess.PIPE,
                                           cwd=git_repo)
            _, _ = git_process.communicate()
            if git_process.returncode == 0:
                return True
        except OSError:
            cprint("[ERROR] Failed to run 'git diff-index --quiet HEAD --' for " + git_repo, "red")
        return False

    def git_update_index_refresh(self, git_repo: str) -> bool:
        try:
            git_process = subprocess.Popen(["git", "update-index", "-q", "--ignore-submodules", "--refresh"],
                                           stdin=subprocess.PIPE,
                                           stdout=subprocess.PIPE,
                                           stderr=subprocess.PIPE,
                                           cwd=git_repo)
            _, _ = git_process.communicate()
            if git_process.returncode == 0:
                return True
        except OSError:
            cprint("[ERROR] Failed to run 'git update-index -q --ignore-submodules --refresh' for " + git_repo, "red")
        return False

    def verify_formatter_version(self) -> None:
        for formatter_instance in self.code_formatter_instances:
            if len(formatter_instance.input_files) > 0:
                formatter_instance.verify_formatter_version()

    def print_mode(self) -> None:
        if self.args.verify:
            cprint("VERIFY MODE", attrs=["bold"])
        else:
            cprint("FORMAT MODE", attrs=["bold"])

    def process_files(self) -> None:
        for formatter_instance in self.code_formatter_instances:
            for file_name in formatter_instance.input_files:
                if self.args.verify:
                    self.failure |= formatter_instance.verify_file(file_name, print_diff=self.args.diff)
                else:
                    self.failure |= formatter_instance.format_file(file_name)

    def number_of_input_files(self) -> int:
        count = 0
        for formatter_instance in self.code_formatter_instances:
            count += len(formatter_instance.input_files)
        return count

    def confirm_with_user(self) -> None:
        if self.number_of_input_files() == 0:
            cprint("[WARN] No input files (or file endings unknown)", "yellow")
        elif (not self.args.verify) and (not self.args.yes) and self.number_of_input_files() > 1:
            for formatter_instance in self.code_formatter_instances:
                formatter_instance.print_input_files()
            answer = input("Are you sure to code format " + str(self.number_of_input_files()) + " files? (y/N)")
            if answer != "y":
                sys.exit(1)


def main() -> None:
    code_format = CodeFormat()
    code_format.parse_command_line()
    code_format.print_mode()

    code_format.add_code_formatter(CodeFormatterClang())
    code_format.add_code_formatter(CodeFormatterAutopep())

    code_format.scan_for_input_files()
    code_format.verify_formatter_version()
    code_format.confirm_with_user()
    code_format.check_input_files_are_in_clean_git_repos_and_are_tracked()
    code_format.process_files()
    if code_format.failure:
        cprint("FAILURE", "red")
        sys.exit(1)
    else:
        cprint("SUCCESS", "green")
        sys.exit(0)

if __name__ == "__main__":
    main()
