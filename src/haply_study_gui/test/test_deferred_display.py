import pygame
import rclpy

from haply_study_gui.study_gui_node import StudyGui


def test_readiness_gated_gui_constructs_before_display_opens():
    rclpy.init(args=["--ros-args", "-p", "require_system_ready:=true"])
    gui = StudyGui()
    try:
        assert gui.screen is None
    finally:
        gui.destroy_node()
        pygame.font.quit()
        pygame.display.quit()
        rclpy.shutdown()
