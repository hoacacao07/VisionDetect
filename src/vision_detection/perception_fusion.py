"""Merge duplicate prompt observations without inventing new detections."""


def intersection_over_union(first, second):
    """Return overlapping area divided by union area for two pixel rectangles."""
    left, top = max(first[0], second[0]), max(first[1], second[1])
    right, bottom = min(first[2], second[2]), min(first[3], second[3])
    intersection = max(0, right - left) * max(0, bottom - top)
    area = lambda box: max(0, box[2] - box[0]) * max(0, box[3] - box[1])
    return intersection / max(area(first) + area(second) - intersection, 1e-9)


def merge_people(observations, overlap_threshold=0.45):
    """Keep stronger overlapping observations with the same canonical label.

    Confidences from different prompt sets are not calibrated probabilities.
    Selecting the higher score is a simple NMS heuristic, not evidence fusion.
    """
    selected = []
    for observation in sorted(
        observations, key=lambda item: item["confidence"], reverse=True
    ):
        # The broad prompt model can give the exact same human a weaker,
        # conflicting object name. Suppress only near-identical boxes with a
        # substantially stronger person proposal, not nearby/overlapping props.
        if observation["label"] != "person" and any(
            item["label"] == "person"
            and item["confidence"] >= 0.6
            and item["confidence"] >= observation["confidence"] + 0.15
            and intersection_over_union(item["bbox_xyxy"], observation["bbox_xyxy"])
            > 0.75
            for item in selected
        ):
            continue
        if any(
            item["label"] == observation["label"]
            and intersection_over_union(item["bbox_xyxy"], observation["bbox_xyxy"])
            > overlap_threshold
            for item in selected
        ):
            continue
        selected.append(observation)
    return selected
