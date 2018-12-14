from pygame import font
font.init()

SIDE = 10  # Start and goal squares side.
START_COLOR = (0,255,0)  # Start color = green.
GOAL_COLOR = (255,0,0)  # Goal color = red.
START_INIT_POS = (100,100)  # Start initial position.
GOAL_INIT_POS = (30,30)  # Goal initial position.

WIDTH = 418  # Display's width.
HEIGHT = 283  # Display's height.

CAPTION = 'RRT path finder'  # Window's caption.

BG_COLOR = (255,255,255)  # Background color.

EDGE_COLOR = (0,0,0)  # Tree's edge color.
EDGE_WIDTH = 1              # Tree's edge width.
VERTEX_COLOR = (255,128,0)  # Tree's vertex color.
VERTEX_RADIUS = 2           # Tree's vertex radius.

PATH_EDGE_COLOR = (0,0,255) # Path's edge color.
PATH_EDGE_WIDTH = 4         # Path's edge width.
PATH_VERTEX_COLOR = (0,191,255)  # Path's vertex color.
PATH_VERTEX_RADIUS = 4      # Path's vertex radius.

TEXT_X = WIDTH - 200  # 'x' coordinate where to display text info.
TEXT_Y = 15           # 'y' initial coordinate of text info.
TEXT_PADDING = 18     # Text padding between lines.
TEXT_COLOR = (100,0,0)  # Text color.
FONT = font.SysFont('Tahoma',15)  # Text font.

#IMAGE_NAME = "../test_map.png"
IMAGE_NAME ="test_map.pgm"