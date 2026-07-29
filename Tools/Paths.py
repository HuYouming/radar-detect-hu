from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def project_path(*parts):
    """Return a path anchored at the repository root."""
    return PROJECT_ROOT.joinpath(*parts)


def resolve_project_path(path):
    """Resolve repository-relative paths without depending on the current directory."""
    resolved = Path(path).expanduser()
    if resolved.is_absolute():
        return resolved
    return PROJECT_ROOT / resolved
