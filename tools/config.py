"""The one runtime-configuration surface for the whole lab.

Three kinds of configuration exist here and they are deliberately kept apart:

===============  ==========================================  ====================
kind             lives in                                    changes when
===============  ==========================================  ====================
hardware         JSON next to the device it describes --     the hardware itself
inventory        ``tools/arm/arm.json``,                     is rewired
                 ``tools/microscope/cameras.json``
runtime          ``config.yaml`` at the repo root            an operator tunes a
controls                                                     run
taught /         ``~/.sdl_lab/robot_arm/*.json``             calibration is
measured state                                               re-run
===============  ==========================================  ====================

**Precedence, highest first** -- this order is a contract, not an implementation
detail. :func:`resolve` is the only function that applies it, and it reports
which layer won so a run can log its own resolved configuration:

1. an explicit call argument / CLI flag
2. the matching key in ``config.yaml``
3. the device's own inventory JSON
4. the code default

A key absent everywhere yields the code default; a key *present but
unparseable* raises rather than silently falling through, because a typo that
quietly selects a permissive safety limit is exactly the failure this layering
exists to prevent.

No third-party YAML dependency: :func:`load_yaml` is the stdlib-only
nested-mapping parser the camera layer has used since the start, lifted here so
every device reads the same file through the same parser.
"""
from __future__ import annotations

import dataclasses
import json
import os
from pathlib import Path
from typing import Any, Union

#: ``tools/config.py`` -> ``tools`` -> the repo root, where the phase scripts,
#: ``config.yaml`` and ``dataset/`` live.
PACKAGE_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_ROOT.parent
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "config.yaml"

#: Live taught/measured state. Never package data -- it is machine-specific and
#: is rewritten by calibration runs.
STATE_DIR = Path.home() / ".sdl_lab" / "robot_arm"


class ConfigError(ValueError):
    """A configuration file is malformed, or a value is out of range."""


# ---------------------------------------------------------------------------
# stdlib-only YAML subset (nested mappings of scalars -- what config.yaml uses)
# ---------------------------------------------------------------------------

def _strip_comment(value: str) -> str:
    in_single = in_double = False
    for index, char in enumerate(value):
        if char == "'" and not in_double:
            in_single = not in_single
        elif char == '"' and not in_single:
            in_double = not in_double
        elif char == "#" and not in_single and not in_double:
            if index == 0 or value[index - 1].isspace():
                return value[:index].rstrip()
    return value.rstrip()


#: YAML spells booleans several ways. The 1.1 set is accepted because an
#: operator writing ``allow_motion: no`` means no -- and ``bool("no")`` is True,
#: which would turn live motion ON from a config that says off.
_TRUE_WORDS = {"true", "yes", "on", "y"}
_FALSE_WORDS = {"false", "no", "off", "n"}


def _parse_scalar(value: str) -> Any:
    value = _strip_comment(value).strip()
    if not value:
        return ""
    if (value.startswith('"') and value.endswith('"')) or (
        value.startswith("'") and value.endswith("'")
    ):
        return value[1:-1]
    lowered = value.lower()
    if lowered in _TRUE_WORDS:
        return True
    if lowered in _FALSE_WORDS:
        return False
    if lowered in {"null", "none", "~"}:
        return None
    try:
        if any(marker in value for marker in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value


def load_yaml(path: Path) -> dict[str, Any]:
    """Parse the nested-mapping YAML subset used by ``config.yaml``."""
    root: dict[str, Any] = {}
    stack: list[tuple[int, dict[str, Any]]] = [(-1, root)]
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw_line.strip() or raw_line.lstrip().startswith("#"):
            continue
        leading = raw_line[: len(raw_line) - len(raw_line.lstrip(" "))]
        if "\t" in leading:
            raise ConfigError(f"{path}:{line_number}: tabs are not valid indentation")
        indent = len(leading)
        line = _strip_comment(raw_line.strip())
        if not line:
            continue
        key, separator, value = line.partition(":")
        if not separator or not key.strip():
            raise ConfigError(f"{path}:{line_number}: expected 'key: value'")
        key = key.strip()
        while stack and stack[-1][0] >= indent:
            stack.pop()
        if not stack:
            raise ConfigError(f"{path}:{line_number}: invalid indentation")
        parent = stack[-1][1]
        if value.strip() == "":
            child: dict[str, Any] = {}
            parent[key] = child
            stack.append((indent, child))
        else:
            parent[key] = _parse_scalar(value)
    return root


# ---------------------------------------------------------------------------
# precedence
# ---------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class Resolved:
    """A single configuration value plus the layer it came from."""

    key: str
    value: Any
    source: str  # "argument" | "config.yaml" | "<device>.json" | "default"

    def __str__(self) -> str:
        return f"{self.key}={self.value!r} ({self.source})"


def resolve(
    key: str,
    *,
    argument: Any = None,
    config: dict[str, Any] | None = None,
    inventory: dict[str, Any] | None = None,
    default: Any = None,
    inventory_name: str = "device.json",
    cast: type | None = None,
) -> Resolved:
    """Apply the precedence contract to one key. See the module docstring.

    ``argument`` wins when it is not None. A present-but-uncastable value is an
    error: falling through to a laxer default is the silent failure this exists
    to prevent.
    """
    layers = (
        ("argument", argument),
        ("config.yaml", (config or {}).get(key)),
        (inventory_name, (inventory or {}).get(key)),
    )
    for source, raw in layers:
        if raw is None:
            continue
        if cast is not None and not isinstance(raw, cast):
            if cast is bool:
                # NEVER bool(x): every non-empty string is truthy, so a typo'd
                # "flase" would read as True and switch on live motion.
                word = str(raw).strip().lower()
                if word in _TRUE_WORDS:
                    raw = True
                elif word in _FALSE_WORDS:
                    raw = False
                else:
                    raise ConfigError(
                        f"{key}: {raw!r} from {source} is not a boolean "
                        f"(use true/false); refusing to guess, because guessing "
                        f"wrong on a gate like this enables motion")
            else:
                try:
                    raw = cast(raw)
                except (TypeError, ValueError) as exc:
                    raise ConfigError(
                        f"{key}: value {raw!r} from {source} is not a valid "
                        f"{cast.__name__} ({exc}); refusing to fall back to the default"
                    ) from exc
        return Resolved(key, raw, source)
    return Resolved(key, default, "default")


def load_inventory(path: Path) -> dict[str, Any]:
    """Read a device's hardware-inventory JSON. Missing file -> empty mapping."""
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ConfigError(f"{path}: unreadable hardware inventory ({exc})") from exc
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: hardware inventory must be a JSON object")
    return data


# ---------------------------------------------------------------------------
# LabConfig
# ---------------------------------------------------------------------------

class LabConfig:
    """Parsed ``config.yaml``, addressed by device section.

    Accepts a path, an already-parsed mapping, or None (the repo-root default),
    so every device API can take a uniform ``config=`` argument::

        arm.move(35, 34, 33, "config.yaml")
        arm.move(35, 34, 33, LabConfig.load())
        arm.move(35, 34, 33)                      # repo-root config.yaml
    """

    def __init__(self, raw: dict[str, Any], path: Path | None = None) -> None:
        if not isinstance(raw, dict):
            raise ConfigError("lab configuration must be a mapping")
        self.raw = raw
        self.path = path

    # -- construction ---------------------------------------------------
    @classmethod
    def load(cls, source: Any = None) -> LabConfig:
        if isinstance(source, LabConfig):
            return source
        if isinstance(source, dict):
            return cls(source, None)
        if source is None:
            env = os.environ.get("SDL_CONFIG")
            path = Path(env).expanduser() if env else DEFAULT_CONFIG_PATH
        else:
            path = Path(source).expanduser()
        if not path.exists():
            if source is not None:
                raise ConfigError(f"config file not found: {path}")
            return cls({}, None)
        return cls(load_yaml(path), path)

    # -- access ---------------------------------------------------------
    def section(self, name: str) -> dict[str, Any]:
        value = self.raw.get(name, {})
        if value is None:
            return {}
        if not isinstance(value, dict):
            raise ConfigError(
                f"config section '{name}' must be a mapping, got {type(value).__name__}"
            )
        return value

    def get(self, section: str, key: str, default: Any = None) -> Any:
        return self.section(section).get(key, default)

    def path_for(self, value: Any, *, default: Path | None = None) -> Path | None:
        """Resolve a relative config path against the REPO ROOT.

        Not against the config file's own directory: the file lives in
        ``configs/``, and a reader seeing ``tools/microscope/cameras.json`` means the
        one at the top of the repo, not ``configs/tools/microscope/cameras.json``.
        Absolute values are taken as-is.
        """
        if value in (None, ""):
            return default
        candidate = Path(str(value)).expanduser()
        if candidate.is_absolute():
            return candidate
        return (PROJECT_ROOT / candidate).resolve()

    def __repr__(self) -> str:
        where = self.path if self.path is not None else "<in-memory>"
        return f"LabConfig({where}, sections={sorted(self.raw)})"


#: Anything a device API accepts as its ``config=`` argument.
ConfigLike = Union[LabConfig, dict, str, Path, None]


def load(source: Any = None) -> LabConfig:
    """Module-level shorthand for :meth:`LabConfig.load`."""
    return LabConfig.load(source)


__all__ = [
    "ConfigError",
    "ConfigLike",
    "DEFAULT_CONFIG_PATH",
    "LabConfig",
    "PACKAGE_ROOT",
    "PROJECT_ROOT",
    "Resolved",
    "STATE_DIR",
    "load",
    "load_inventory",
    "load_yaml",
    "resolve",
]
