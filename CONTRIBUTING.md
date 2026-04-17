# Contributing to CarlaAir

Thank you for your interest in contributing to CarlaAir! We welcome contributions of all kinds — bug reports, feature requests, documentation improvements, and code contributions.

## Getting Started

1. Fork the repository and clone your fork
2. Create a new branch for your changes: `git checkout -b feature/your-feature-name`
3. Make your changes and test them
4. Commit and push to your fork
5. Open a Pull Request

## Commit Messages

We follow the [Conventional Commits](https://www.conventionalcommits.org/) format:

| Type | Description |
|------|-------------|
| `feat` | New feature |
| `fix` | Bug fix |
| `docs` | Documentation only |
| `refactor` | Code refactoring (no feature change) |
| `test` | Adding or updating tests |
| `chore` | Build, CI, or tooling changes |

Example: `feat: add multi-drone spawn support`

## Issues

- **Bug reports**: Use the `[Bug]` prefix in the title and include steps to reproduce, expected vs. actual behavior, and your environment (OS, Python version, etc.)
- **Feature requests**: Use the `[Feature]` prefix and describe the use case

## Pull Requests

- Keep PRs focused — one feature or fix per PR
- Use branch naming: `feature/xxx`, `fix/xxx`, `docs/xxx`
- Include a clear description of what changed and why
- Reference related issues (e.g., `Closes #12`)

## Development Environment

### Prerequisites

| Component | Version |
|-----------|---------|
| Unreal Engine | 5.7+ |
| Python | 3.10–3.12 |
| CMake | 3.20+ |
| Ninja | latest |

### macOS ARM (Apple Silicon — recommended for local dev)

```bash
# Clone and set up
git clone git@github.com:louiszengCN/CarlaAir.git
cd CarlaAir

# Install Homebrew dependencies for building CARLA libs
brew install boost sqlite xerces-c proj ninja cmake

# Build CARLA server dependencies (rpclib, libcarla_server, osm2odr)
bash build_carla_deps_macos.sh

# Set up Python environment (Python 3.12)
bash env_setup/setup_env.sh
conda activate carlaAir
bash env_setup/test_env.sh
```

### Linux (Ubuntu 20.04/22.04)

```bash
git clone git@github.com:louiszengCN/CarlaAir.git
cd CarlaAir
make setup
make LibCarla.server.release
make PythonAPI
```

### Static Analysis

```bash
# Python linting
ruff check .
mypy src/

# Shell script linting
shellcheck Util/BuildTools/*.sh carlaAir.sh

# C++ linting (requires compile_commands.json from UBT)
clang-tidy --config-file=.clang-tidy <file>
```

## Code of Conduct

Please be respectful and constructive in all interactions. We are committed to providing a welcoming and inclusive experience for everyone.

## License

By contributing, you agree that your contributions will be licensed under the MIT License.
