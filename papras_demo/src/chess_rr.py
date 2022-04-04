#!/usr/bin/env python3
'''
Robot vs. Robot Chess Demo
'''

import rospy
from std_msgs.msg import String, Bool
# Chess Engine
from stockfish import Stockfish

robot_done = True

class ChessEngine:
    def __init__(self):
        self.engine = Stockfish()
        self.board_letters = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
        self.piece_dict = {'q':'Queen', 'Q':'Queen', 'k':'King', 'K':'King', \
                           'b':'Bishop', 'B':'Bishop', 'n':'Knight', 'N':'Knight', \
                           'r':'Rook', 'R':'Rook', 'p':'Pawn', 'P':'Pawn'}
        self.prev_move = None
        self.prev_piece = None

    def get_move(self):
        correct = False
        moves = self.engine.get_top_moves(3)
        if moves is None or len(moves) == 0:
            print("ERROR: NO MOVES FOUND!")
            return None

        for m in moves:
            correct = self.engine.is_move_correct(m['Move'])
            if correct:
                return m

        print("ERROR: NO MOVES FOUND!")
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
        if 'castling' in move:
            print("Castling!")
            if move[2] == 'g':
                return 'h1,f1'
            if move[2] == 'c':
                return 'a1,d1'

        return None

    def detect_promotion(self, move):
        if 'promotion' in move:
            print("Promotion!")
            piece = move[4]
            print("Please give me: "+self.piece_dict[piece])
            return True
        return False

    def save_move(self, move):
        board = self.disect_board()
        self.prev_move = move[0:4]
        pos1, pos2 = move[:2], move[2:4]
        x = abs(int(pos1[1])-8)
        y = self.board_letters.index(pos1[0])
        self.prev_piece = board[x][y]

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

def chess_RR():
    '''
    Output: "Arm, Starting Position, Ending Position"
            "Arm1,C3,D8" -- comma delimited, no spaces
    '''
    publisher = rospy.Publisher('chess_move', String, queue_size=10)
    subscriber = rospy.Subscriber("move_finished", Bool, callback)
    rospy.init_node('chess_rr', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    chess = ChessEngine()
    arms = ["arm1", "arm2"]
    which_arm = 0

    while not rospy.is_shutdown():

        # get best move based on state of the board
        move = chess.get_move()#"Arm1,A1,A2"
        if move is None:
            print("GAME OVER")
            print("Winning robot: "+arms[which_arm])
            break
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

        # switch to other player's turn
        which_arm = (which_arm + 1) % 2
        chess.save_move(move['Move'])

        rate.sleep()

def tester():
    chess = ChessEngine()
    arms = ["Arm1", "Arm2"]
    which_arm = 0

    while True:

        # get best move based on state of the board
        move = chess.get_move()#"Arm1,A1,A2"
        if move is None:
            print("GAME OVER")
            print("Winning robot: "+arms[which_arm])
            break
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
            pass
            # publisher.publish('promotion')

        # switch to other player's turn
        which_arm = (which_arm + 1) % 2
        chess.save_move(move['Move'])

if __name__ == '__main__':
    # tester()
    try:
        chess_RR()
    except rospy.ROSInterruptException:
        print("Failed to launch chess demo RvR!")
