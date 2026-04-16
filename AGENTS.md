# Cosys-AirSim Development Tools

## Package Manager

Uses **uv** for fast, reliable Python package management.

## Pre-Commit Hooks

Configured in `.pre-commit-config.yaml`:

### Python
- **pyupgrade**: Auto-upgrade to Python 3.12+ syntax
- **ruff**: Linting and formatting (replaces flake8, isort, pydocstyle)
- **ruff-format**: Black-compatible formatting

### Docker
- **hadolint**: Dockerfile linting

### General
- trailing-whitespace, end-of-file-fixer
- check-yaml, check-json, check-toml
- check-merge-conflict
- check-added-git-ignore
- debug-statements (Python)

## GitHub Actions CI/CD

### `.github/workflows/python.yml`
- Python lint (ruff)
- Type checking (mypy)
- Security audit (pip-audit)
- Tests on Python 3.12-3.14

### `.github/workflows/lint.yml`
- Pre-commit hooks check
- Hadolint for Dockerfiles

### `.github/workflows/codeql.yml`
- C++ security scanning

### `.github/workflows/build-linux.yml`
- C++ build on Ubuntu 22.04/24.04

### `.github/dependabot.yml`
- Weekly uv and GitHub Actions updates

## Linting Tools

### Install with uv
```bash
# Install uv if not already installed
curl -LsSf https://astral.sh/uv/install.sh | sh

# Install dependencies
uv pip install -e "PythonClient[dev]"

# Or install individually
uv pip install ruff mypy pip-audit pytest
```

### Run linting
```bash
# Run ruff
ruff check PythonClient/
ruff format PythonClient/

# Run type checking
mypy PythonClient/cosysairsim/

# Run security audit
pip-audit

# Run tests
cd PythonClient
pytest tests/ -v
```

### Docker
```bash
hadolint Dockerfile
```

### Shell
```bash
shellcheck -x -e SC1090,SC1091 scripts/*.sh
```

### CMake
```bash
cmake-format -i CMakeLists.txt
```

## Editor Setup

### VS Code (recommended)
```json
{
  "python.linting.ruffEnabled": true,
  "python.formatting.provider": "ruff",
  "python.analysis.typeCheckingMode": "basic",
  "[python]": {
    "editor.formatOnSave": true,
    "editor.codeActionsOnSave.source.fixAll": "explicit"
  }
}
```

### C++, use clangd or compile_commands.json
