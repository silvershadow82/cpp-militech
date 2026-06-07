# Design: clang-format pre-commit hook for DevContainer

**Date:** 2026-06-07  
**Status:** Approved

## Goal

Automatically run clang-format on staged C/C++ files before each commit, blocking the commit if any file is not formatted. Setup must be zero-touch for students — active immediately after a Dev Containers rebuild.

## Approach

Use the `pre-commit` Python framework (already partially wired: `.git/hooks/pre-commit` is the framework runner). Add the missing pieces: the `pre-commit` package in the container and a `.pre-commit-config.yaml` config file.

## Changes

### 1. `.devcontainer/apt-packages.in`
Add `pre-commit` under the build tools section. Ubuntu 24.04 ships it in apt — no pip/venv needed.

### 2. `.pre-commit-config.yaml` (new, repo root)
A single `local` hook with `language: system`. This tells the framework to invoke the system `clang-format` binary directly — no binary download. The hook runs `clang-format --dry-run --Werror` on staged files matching `\.(cpp|hpp|h|c|cxx|cc)$`. Dry-run mode checks only; `--Werror` turns formatting differences into a non-zero exit code, blocking the commit. Students then run `clang-format -i <file>`, re-stage, and commit again. The `.clang-format` style config is picked up automatically from `/.clang-format` (placed by the Dockerfile) as clang-format walks up the directory tree.

### 3. `.devcontainer/devcontainer.json`
Add `"postCreateCommand": "pre-commit install"`. This runs once after the container is created or rebuilt, registering the framework with the git hook. The existing `.git/hooks/pre-commit` script is already the correct framework runner — it just needs this registration step to find `.pre-commit-config.yaml`.

## Student experience

1. Rebuild container → `postCreateCommand` runs `pre-commit install` automatically
2. `git commit` with badly-formatted staged files → commit blocked, output names the failing files
3. Student runs `clang-format -i <file>`, re-stages, commits successfully

## What is NOT changing

- `.git/hooks/pre-commit` — already the correct framework runner, untouched
- `.devcontainer/.clang-format` / `Dockerfile` — style config delivery unchanged
- No auto-formatting on commit (intentional: students learn to format themselves)
