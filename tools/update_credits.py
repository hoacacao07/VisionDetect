"""Record installed third-party packages for reproducible source attribution.

Run after installing/updating packages. Metadata is read locally; this does not
contact package services, open the camera, or change installed dependencies.
"""

from importlib.metadata import distributions
from pathlib import Path
import sys

PROJECT_DIRECTORY = Path(__file__).resolve().parents[1]
DEVELOPMENT_TOOLS = {"black", "beautifulsoup4", "jsbeautifier", "cssbeautifier"}


def package_source(metadata):
    """Prefer the upstream source/homepage declared by the package itself."""
    urls = metadata.get_all("Project-URL") or []
    for preferred in ("source", "repository", "homepage", "home", "documentation"):
        for entry in urls:
            label, separator, url = entry.partition(",")
            if (
                separator
                and preferred in label.lower()
                and url.strip().startswith("http")
            ):
                return url.strip()
    homepage = metadata.get("Home-page", "")
    return homepage if homepage.startswith("http") else ""


if __name__ == "__main__":
    rows = []
    for package in sorted(
        distributions(), key=lambda item: item.metadata["Name"].lower()
    ):
        metadata = package.metadata
        name = metadata["Name"]
        license_name = metadata.get("License-Expression") or metadata.get("License", "")
        license_name = " ".join(license_name.split())
        if not license_name or len(license_name) > 100:
            license_name = "See distributed license files"
        license_name = license_name.replace("|", "/")
        url = package_source(metadata)
        source = (
            f"[Upstream]({url})"
            if url
            else f"[Package page](https://pypi.org/project/{name}/)"
        )
        role = (
            "Development formatter"
            if name.lower() in DEVELOPMENT_TOOLS
            else "Runtime, preparation or dependency"
        )
        rows.append(
            f"| {name} | {package.version} | {role} | {license_name} | {source} |"
        )
    document = (
        "# Installed third-party software credits\n\n"
        "[← Source credits](SOURCE_CREDITS.md) · [Project home](../README.md)\n\n"
        "This inventory includes direct libraries, their installed dependencies, and development tools. "
        "Names, versions, license labels and links are taken from installed package metadata. "
        "Each package retains its distributed license files and notices.\n\n"
        f"Python runtime: {sys.version.split()[0]} — [CPython](https://github.com/python/cpython), "
        "[Python licensing](https://docs.python.org/3/license.html).\n\n"
        "| Package | Installed version | Usage | Declared license | Original source |\n"
        "| --- | --- | --- | --- | --- |\n" + "\n".join(rows) + "\n\n"
        "Regenerate with `python tools/update_credits.py` using the project environment. "
        "Model weights, text encoder, validation imagery and authored additions are credited separately in SOURCE_CREDITS.md.\n"
    )
    (PROJECT_DIRECTORY / "docs" / "THIRD_PARTY_SOFTWARE.md").write_text(
        document, encoding="utf-8"
    )
    print(f"Recorded credits for {len(rows)} installed packages.")
