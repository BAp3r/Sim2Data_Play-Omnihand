import importlib.util
import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from isaac_relay import wait_stage_assets


class LoadingFenceTests(unittest.TestCase):
    def test_waits_for_ten_consecutive_idle_updates(self):
        class Context:
            calls=0
            def get_stage(self):return object()
            def get_stage_loading_status(self):
                self.calls+=1
                return ("",0,1 if self.calls in (1,4) else 0)
        class App:
            def update(self):pass
        context=Context()
        self.assertEqual(wait_stage_assets(context,App(),"test")["updates"],14)

    def test_pending_assets_cannot_pass_on_timeout(self):
        class Context:
            def get_stage(self):return object()
            def get_stage_loading_status(self):return ("",0,1)
        class App:
            def update(self):pass
        with self.assertRaisesRegex(RuntimeError,"did not settle"):
            wait_stage_assets(Context(),App(),"pending",max_updates=3)


@unittest.skipUnless(importlib.util.find_spec("numpy"),"numpy unavailable")
class SemanticPixelsTests(unittest.TestCase):
    def test_missing_robot_cannot_pass_with_visible_background_and_box(self):
        import numpy as np
        from isaac_contact_trial import camera_content
        class Annotator:
            def get_data(self):return {"data":np.full((10,10),3),"info":{"idToLabels":{"3":{"class":"cardbox"}}}}
        class Camera:
            _custom_annotators={"semantic_segmentation":Annotator()}
            def get_current_frame(self):return {}
        result=camera_content(Camera(),("left_robot","right_robot","thor","cardbox"))
        self.assertEqual(result["class_pixels"]["cardbox"],100)
        self.assertFalse(result["all_labels_visible"])
        self.assertEqual(result["class_pixels"]["left_robot"],0)
        self.assertIn("cardbox",result["edge_touching_labels"])

    def test_distinct_labels_and_legacy_single_arm_contract(self):
        import numpy as np
        from isaac_contact_trial import camera_content
        class Annotator:
            def get_data(self):return {"data":np.r_[np.full(30,2),np.full(30,3)],"info":{"idToLabels":{"2":{"class":"robot"},"3":{"class":"cardbox"}}}}
        class Camera:
            _custom_annotators={"semantic_segmentation":Annotator()}
            def get_current_frame(self):return {}
        self.assertTrue(camera_content(Camera())["robot_and_box_visible"])
        self.assertFalse(camera_content(Camera(),("left_robot","right_robot","thor","cardbox"))["all_labels_visible"])
