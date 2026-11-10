"""OBJECT LIBRARY: one editable source for labels, categories and mobility.

Class-based mobility is a prior, not observed motion. A parked car and a moving
car both belong to potentially_dynamic. This catalog does not measure motion;
tracking.py supplies current stationary-camera estimates. A future moving robot
also needs camera-motion compensation, as described in NAVIGATION_ROADMAP.md.
"""

from dataclasses import dataclass
import json
from pathlib import Path
from .settings import PROJECT_DIRECTORY

OBJECT_LIBRARY_PATH = PROJECT_DIRECTORY / "config" / "outdoor_objects.json"
SUPPORTED_MOTION_GROUPS = {"potentially_dynamic", "usually_static"}


@dataclass(frozen=True)
class ObjectDefinition:
    """Meaning of one prompt label, independent of its model class number."""

    label: str
    category: str
    motion_group: str


def load_object_library(path=OBJECT_LIBRARY_PATH):
    """Return label -> definition; reject duplicate or mistyped additions."""
    data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    definitions = {}
    groups = data.get("motion_groups")
    if not isinstance(groups, dict) or set(groups) != SUPPORTED_MOTION_GROUPS:
        raise ValueError(
            "Object library needs potentially_dynamic and usually_static groups."
        )
    for motion_group, group in groups.items():
        categories = group.get("categories")
        if not isinstance(categories, dict) or not categories:
            raise ValueError(f"{motion_group} needs named categories.")
        for category, labels in categories.items():
            if not isinstance(category, str) or not category.strip():
                raise ValueError("Category names must be nonempty text.")
            if not isinstance(labels, list) or not labels:
                raise ValueError(f"{category} needs a nonempty list of labels.")
            for label in labels:
                if (
                    not isinstance(label, str)
                    or not label.strip()
                    or label != label.strip()
                ):
                    raise ValueError(
                        "Object names must be nonempty text with no outer whitespace."
                    )
                if label in definitions:
                    raise ValueError(f"Duplicate object label: {label}")
                definitions[label] = ObjectDefinition(label, category, motion_group)
    return definitions
