import os
import tempfile
import unittest

from Tools.Paths import PROJECT_ROOT, project_path, resolve_project_path


class ProjectPathsTest(unittest.TestCase):
    def test_repository_paths_do_not_depend_on_current_directory(self):
        original_directory = os.getcwd()
        try:
            with tempfile.TemporaryDirectory() as outside_directory:
                os.chdir(outside_directory)
                self.assertEqual(
                    project_path("configs", "main_config.yaml"),
                    PROJECT_ROOT / "configs" / "main_config.yaml",
                )
                self.assertTrue(
                    resolve_project_path("weight/stage_one.pt").is_file()
                )
        finally:
            os.chdir(original_directory)

    def test_absolute_paths_are_preserved(self):
        absolute_path = PROJECT_ROOT / "RM2026_map.pcd"
        self.assertEqual(resolve_project_path(absolute_path), absolute_path)


if __name__ == "__main__":
    unittest.main()
