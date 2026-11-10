"""Stationary-camera perception prototype, organized by responsibility.

Input/health -> recognition/distance -> tracking/prediction -> local outputs.
runtime.py coordinates these modules. Navigation policy and wheel control are
future components; see docs/NAVIGATION_ROADMAP.md for intended extension points.
"""
