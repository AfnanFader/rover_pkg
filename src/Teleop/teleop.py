import zmq
import pygame
from pygame.locals import *

RASPBERRY_IP = "172.19.220.138"
TELEOP_PORT = 5600

KEY_CODE_SPACE = 32

TELEOP_RATE = 25

moveBindingsGame = {
    K_UP: "i",
    K_LEFT: "j",
    K_RIGHT: "l",
    K_DOWN: "k",
    K_a : "a",
    K_w : "w",
    K_s : "s",
    K_d : "d",
    K_q : "q",
    K_e : "e"
}

def pygameMain():
    pygame.init()
    window = pygame.display.set_mode((800,500),RESIZABLE)
    pygame.font.init()
    small_font = pygame.font.SysFont('Open Sans', 20)
    end = False

    def writeText(screen, text, x, y, font, color=(62, 107, 153)):
        text = str(text)
        text = font.render(text, True, color)
        screen.blit(text, (x, y))

    def clear():
        window.fill((0, 0, 0))

    def updateScreen(window):
        clear()
        help_str = 'Use arrow keys for Servo.'
        writeText(window, help_str, 20, 50, small_font)
        help_str = 'Use WASD for Motor.'
        writeText(window, help_str, 20, 70, small_font)
        help_2 = 'Q/E - Linear UP/DOWN  : ESC to exit'
        writeText(window, help_2, 20, 100, small_font)

    while not end:
        keys = pygame.key.get_pressed()
        for keycode in moveBindingsGame.keys():
            if keys[keycode]:
                val = moveBindingsGame[keycode]
                sendToServer(socket, val)
    
        updateScreen(window)

        for event in pygame.event.get():
            if event.type == QUIT or event.type == KEYDOWN and event.key in [K_ESCAPE]:
                end = True

        pygame.display.flip()
        pygame.time.Clock().tick(TELEOP_RATE)


def sendToServer(socket, val):
    socket.send_json((val))
    

if __name__ == '__main__':
    context = zmq.Context()
    socket = context.socket(zmq.PAIR)
    socket.connect("tcp://{}:{}".format(RASPBERRY_IP, TELEOP_PORT))

    msg = socket.recv()
    print("Connected To Server")
    try:
        pygameMain()
    except KeyboardInterrupt as e:
        pass
    finally:
        socket.send_json(("p"))
        socket.close()