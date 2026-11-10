import unittest
from vision_detection.object_library import load_object_library
from vision_detection.prompt_library import load_detection_prompts


class PromptTests(unittest.TestCase):
    def test_visual_aliases_keep_stable_navigation_labels(self):
        library = load_object_library()
        prompts = load_detection_prompts(library)
        self.assertEqual(prompts["red box"], "box")
        self.assertEqual(prompts["human"], "person")
        self.assertEqual(prompts["parked car"], "car")
        self.assertEqual(set(prompts.values()), set(library))
