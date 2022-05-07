#!/usr/bin/env python3
'''
Robot vs. Human Chess Demo
'''

import rospy
from std_msgs.msg import String, Bool, Int32
# Chess Engine
from stockfish import Stockfish

robot_done = False
start_script = False
mode = 'rh'


human_moves = [
    'e2e4',
    'g1f3',
    'd2d4',
    'd1d4',
    'b1c3',
    'd4d3',
    'c1f4',
    'e1c1',
    'c1b1',
    'f4e3',
    'f1e2',
    'd3d2',
    'c3d5',
]
curr_move = 0
start_first_move = False 
class ChessEngine:
    def __init__(self):
        self.engine = Stockfish()
        self.engine.set_skill_level(20)
        self.board_letters = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
        self.piece_dict = {'q':'Queen', 'Q':'Queen', 'k':'King', 'K':'King', \
                           'b':'Bishop', 'B':'Bishop', 'n':'Knight', 'N':'Knight', \
                           'r':'Rook', 'R':'Rook', 'p':'Pawn', 'P':'Pawn'}
        self.prev_move = None
        self.prev_piece = None

    def get_move(self):

        move = self.engine.get_best_move()
        # print(move)
        if self.engine.is_move_correct(move):
            return {'Move':move, 'Centipawn':None, 'Mate':None}

        moves = self.engine.get_top_moves(3)
        if moves is None or len(moves) == 0:
            print("ERROR: NO MOVES FOUND!")
            return None

        for m in moves:
            if self.engine.is_move_correct(m['Move']):
                return m

        print("ERROR: NO MOVES VALID!")
        return None

    def print_board(self):
        board = self.engine.get_board_visual()
        print(board)

    def make_move(self, move):
        self.engine.make_moves_from_current_position([move])

    def disect_board(self):
        board = self.engine.get_board_visual()
        lines = board.splitlines()
        board_array = []
        for i in range(len(lines)):
            if i % 2 != 0:
                l = lines[i].replace(" ", "")
                l = l.split('|')[1:-1]
                board_array.append(l)
        return board_array

    def detect_capture(self, move):
        board = self.disect_board()
        pos1, pos2 = move[:2], move[2:4]

        # check for piece on top of piece capture
        x = abs(int(pos2[1])-8)
        y = self.board_letters.index(pos2[0])
        if board[x][y] != '':
            print("Capture!")
            return pos2 + ",bucket"

        # check for en passant
        if self.prev_move is not None:
            x = abs(int(pos1[1])-8)
            y = self.board_letters.index(pos1[0])
            if board[x][y].lower() == 'p' and self.prev_piece.lower() == 'p':
                # did previous pawn move two spaces
                if abs(int(self.prev_move[3]) - int(self.prev_move[1])) == 2:
                    # did current pawn move behind this pawn
                    if pos2 == (self.prev_move[3]+str(int(self.prev_move[4])-1)) and \
                       board[x][y] == 'p':
                        print("Capture! En Passant!")
                        return self.prev_move[2:4] + ",bucket"
                    if pos2 == (self.prev_move[3]+str(int(self.prev_move[4])+1)) and \
                       board[x][y] == 'P':
                        print("Capture! En Passant!")
                        return self.prev_move[2:4] + ",bucket"

        return None

    def detect_castle(self, move):
        if len(move) == 5:
            return None
        board = self.disect_board()
        pos1, pos2 = move[:2], move[2:4]
        x = abs(int(pos1[1])-8)
        y = self.board_letters.index(pos1[0])
        x2 = abs(int(pos2[1])-8)
        y2 = self.board_letters.index(pos2[0])
        piece = board[x][y]
        if piece == 'K' and abs(y2-y) == 2 and x2==x:
            print("Castling!")
            if move[2] == 'g':
                return 'h1,f1'
            if move[2] == 'c':
                return 'a1,d1'
        elif piece == 'k' and abs(y2-y) == 2 and x2==x:
            print("Castling!")
            if move[2] == 'g':
                return 'h8,f8'
            if move[2] == 'c':
                return 'a8,d8'

        return None

    def detect_promotion(self, move):
        if len(move) == 5:
            print("Promotion!")
            piece = move[4]
            print("Please give me: "+self.piece_dict[piece])
            print("Please indicate you have finished helping the robot :) Simply hit enter.")
            done = input(">>> ")
            return True
        return False

    def save_move(self, move):
        board = self.disect_board()
        self.prev_move = move[0:4]
        pos1, pos2 = move[:2], move[2:4]
        x = abs(int(pos1[1])-8)
        y = self.board_letters.index(pos1[0])
        self.prev_piece = board[x][y]

    def check_human_move(self, move):
        if self.engine.is_move_correct(move):
            m = self.detect_castle(move)
            if m:
                print(""+m)
            else:
                print("")
            m = self.detect_promotion(move)
            if m:
                print(""+m)
            else:
                print("")
            m = self.detect_capture(move)
            if m:
                print(""+m)
            else:
                print("")
        return self.engine.is_move_correct(move)

def publish_move(publisher, data):
    print("Trying to publish move.")
    global robot_done
    while not robot_done:
        pass
    print(data)
    robot_done = False
    publisher.publish(data)

def callback(data):
    print("Callback.")
    global robot_done
    robot_done = True

def robot_turn(chess, arms, which_arm, publisher):

    # tell robot to move to 'playing home position'
    move_string = arms[which_arm] + ',00,playing'
    publish_move(publisher, move_string)

    # get best move based on state of the board
    move = chess.get_move()#"Arm1,A1,A2"
    if move is None:
        print("Winning Robot: "+arms[(which_arm + 1) % 2])
        # tell robot to move to 'not playing home position'
        move_string = arms[which_arm] + ',00,notplaying'
        publish_move(publisher, move_string)
        return True
    from_move = move['Move'][:2]
    to_move = move['Move'][2:4]

    # Check if the move is a castle and move the Knight first
    move_castle = chess.detect_castle(move['Move'])
    if move_castle is not None:
        move_string = arms[which_arm] + "," + move_castle
        publish_move(publisher, move_string)

    # Check if the move captures another pawn
    move_capture = chess.detect_capture(move['Move'])
    if move_capture is not None:
        move_string = arms[which_arm] + "," + move_capture
        publish_move(publisher, move_string)

    # Publish chosen move
    move_string = arms[which_arm] + "," + from_move + "," + to_move
    publish_move(publisher, move_string)
    chess.make_move(move['Move'])

    # Check if the move is a promotion
    if chess.detect_promotion(move['Move']):
        move_string = arms[which_arm] + ",00,promotion"
        publish_move(publisher, move_string)

    chess.save_move(move['Move'])

    # tell robot to move to 'not playing home position'
    move_string = arms[which_arm] + ',00,notplaying'
    publish_move(publisher, move_string)

    return False

def human_turn(chess, arms, which_arm):
    global curr_move
    if curr_move >= len(human_moves)-1:
        return True
    
    quit = False
    while True:
        print("------------------------------------------------------")
        print("YOUR TURN!")

        chess_choice = chess.get_move()#"Arm1,A1,A2"
        if chess_choice is None:
            print("Winning PLAYER: "+arms[(which_arm + 1) % 2])
            quit = True
            break

        rospy.sleep(rospy.Duration(10))
        move = human_moves[curr_move]
        curr_move +=1

        if chess.check_human_move(move):
            chess.make_move(move)
            chess.save_move(move)
            break
        else:
            print("INVALID MOVE! Try again...")
            continue

    return quit


def robot_turn_no_pub(chess, arms, which_arm):

    # get best move based on state of the board
    move = chess.get_move()#"Arm1,A1,A2"
    # print(which_arm)
    print(move['Move'])
    if move is None:
        print("Winning Robot: "+arms[(which_arm + 1) % 2])
        return True
    from_move = move['Move'][:2]
    to_move = move['Move'][2:4]

    # Check if the move is a castle and move the Knight first
    move_castle = chess.detect_castle(move['Move'])
    if move_castle is not None:
        move_string = arms[which_arm] + "," + move_castle

    # Check if the move captures another pawn
    move_capture = chess.detect_capture(move['Move'])
    if move_capture is not None:
        move_string = arms[which_arm] + "," + move_capture

    # Publish chosen move
    move_string = arms[which_arm] + "," + from_move + "," + to_move
    chess.make_move(move['Move'])

    # Check if the move is a promotion
    if chess.detect_promotion(move['Move']):
        move_string = arms[which_arm] + ",00,promotion"

    chess.save_move(move['Move'])
    return False


def chess_RR():
    '''
    Output: "Arm, Starting Position, Ending Position"
            "Arm1,C3,D8" -- comma delimited, no spaces
    '''
    publisher = rospy.Publisher('tea_table/chess_move', String, queue_size=10)
    subscriber = rospy.Subscriber("tea_table/move_finished", Bool, callback)
    rospy.init_node('chess_rr', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    chess = ChessEngine()
    arms = ["arm1", "arm2"]
    which_arm = 0
    quit = False
    global robot_done

    while not rospy.is_shutdown():

        # quit = robot_turn(chess, arms, which_arm, publisher)
        quit = robot_turn_no_pub(chess, arms, which_arm)
        if quit:
            print("GAME OVER")
            break

        # switch to other player's turn
        which_arm = (which_arm + 1) % 2

        rate.sleep()

def chess_RH():
    '''
    Output: "Arm, Starting Position, Ending Position"
            "Arm1,C3,D8" -- comma delimited, no spaces
    '''
    publisher = rospy.Publisher('tea_table/chess_move', String, queue_size=10)
    subscriber = rospy.Subscriber("tea_table/move_finished", Bool, callback)
    start_sub = rospy.Subscriber("/switch_controller", Int32, startScriptCallback)
    rospy.init_node('chess_rh', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    chess = ChessEngine()
    arms = ["human", "arm2"]
    which_arm = 0
    quit = False
    global robot_done
    while not start_script and not robot_done:
        pass

    while not rospy.is_shutdown():

        # HUMAN MOVE
        # chess.print_board()

        if which_arm == 0:
            quit = human_turn(chess, arms, which_arm)

        # ROBOT MOVE
        else:
            robot_turn(chess, arms, which_arm, publisher)

        if quit:
            print("GAME OVER")
            break

        # switch to other player's turn
        which_arm = (which_arm + 1) % 2
        rate.sleep()



def tester():
    chess = ChessEngine()
    arms = ["human", "arm2"]
    which_arm = 0
    quit = False

    # board = chess.disect_board()

    while True:

        chess.print_board()

        # ROBOT MOVE
        if which_arm == 1:
            robot_turn_no_pub(chess, arms, which_arm)

        # HUMAN MOVE
        else:
            quit = human_turn(chess, arms, which_arm)

        if quit:
            print("GAME OVER")
            break

        # switch to other player's turn
        which_arm = (which_arm + 1) % 2

def startScriptCallback(data):
    print("Starting chess script.")
    global start_script
    start_script = True

if __name__ == '__main__':
    
    # global mode
    try:
        if mode == 'rr':
            print("Robot vs. Robot mode engaged.")
            chess_RR()
        elif mode == 'rh':
            # tester()
            print("Robot vs. Human mode engaged.")
            chess_RH()
        else:
            print("Unknown chess mode. Quitting.")
    except rospy.ROSInterruptException:
        print("Error: Failed to start chess demo!")