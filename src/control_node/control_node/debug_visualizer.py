import math
import pygame

def draw_arrow(surface, color, start, end, arrow_size=10):
    """Helper function to draw an arrow using Pygame."""
    pygame.draw.line(surface, color, start, end, 3)
    
    # Calculate the angle for the arrowhead
    angle = math.atan2(end[1] - start[1], end[0] - start[0])
    
    # Calculate arrowhead points
    p1 = (
        end[0] - arrow_size * math.cos(angle - math.pi / 6),
        end[1] - arrow_size * math.sin(angle - math.pi / 6)
    )
    p2 = (
        end[0] - arrow_size * math.cos(angle + math.pi / 6),
        end[1] - arrow_size * math.sin(angle + math.pi / 6)
    )
    
    # Draw the arrowhead
    if start != end:
        pygame.draw.polygon(surface, color, [end, p1, p2])


def run_visualizer(node):
    """Runs the Pygame debugging visualization window."""
    pygame.init()
    width, height = 600, 600
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Control Output Visualizer (DEBUG)")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)

    center = (width // 2, height // 2)
    scale = 20.0 

    running = True
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            screen.fill((30, 30, 30))
            pygame.draw.line(screen, (70, 70, 70), (0, center[1]), (width, center[1]), 1)
            pygame.draw.line(screen, (70, 70, 70), (center[0], 0), (center[0], height), 1)

            current_x = node.latest_control_x
            current_y = node.latest_control_y

            end_x = (center[0] + current_x * scale, center[1])
            end_y = (center[0], center[1] - current_y * scale)
            end_sum = (center[0] + current_x * scale, center[1] - current_y * scale)

            draw_arrow(screen, (255, 50, 50), center, end_x)          # Red X
            draw_arrow(screen, (50, 255, 50), center, end_y)          # Green Y
            draw_arrow(screen, (100, 150, 255), center, end_sum)      # Blue Sum

            if current_x != 0 or current_y != 0:
                pygame.draw.line(screen, (100, 100, 100), end_x, end_sum, 1)
                pygame.draw.line(screen, (100, 100, 100), end_y, end_sum, 1)

            text_x = font.render(f"X Control: {current_x:.2f}", True, (255, 50, 50))
            text_y = font.render(f"Y Control: {current_y:.2f}", True, (50, 255, 50))
            magnitude = math.sqrt(current_x**2 + current_y**2)
            text_sum = font.render(f"Sum Mag: {magnitude:.2f}", True, (100, 150, 255))

            screen.blit(text_x, (10, 10))
            screen.blit(text_y, (10, 35))
            screen.blit(text_sum, (10, 60))

            pygame.display.flip()
            clock.tick(60)

    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()