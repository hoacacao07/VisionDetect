"""Editable alternative visual descriptions mapped to stable output labels.

YOLOE can recognize the same category differently under different prompts.
Aliases expand visual descriptions, not physical categories or simulator IDs.
Re-prepare/export models after changing config/detection_prompts.json.
"""

import json
from .settings import PROJECT_DIRECTORY


def load_detection_prompts(object_library):
    """Validate visual aliases and map model prompts to stable public object labels."""
    aliases = json.loads(
        (PROJECT_DIRECTORY / "config/detection_prompts.json").read_text()
    )
    mapping = {label: label for label in object_library}
    for label, prompts in aliases.items():
        if label not in object_library or not isinstance(prompts, list):
            raise ValueError(f"Invalid prompt category: {label}")
        for prompt in prompts:
            if not isinstance(prompt, str) or not prompt.strip() or prompt in mapping:
                raise ValueError(f"Empty or duplicate detection prompt: {prompt}")
            mapping[prompt] = label
    return mapping
