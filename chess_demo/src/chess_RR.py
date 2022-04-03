#!/usr/bin/env python
'''
Robot vs. Robot Chess Demo
'''

import rospy
from std_msgs.msg import String
# Chess Engine
from stockfish import Stockfish

class ChessEngine:
    def __init__(self):
        self.engine = Stockfish()
        self.board_letters = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']

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
        pos1, pos2 = move[:2], move[2:]
        x = int(pos2[1])
        y = self.board_letters.index(pos2[0])
        print(board)
        print(x)
        print(y)
        print(board[x][y])
        if board[x][y] is not '':
            print("CAPTURE")



def chess_RR():
    '''
    Output: "Arm, Starting Position, Ending Position"
            "Arm1,C3,D8" -- comma delimited, no spaces
    '''
    publisher = rospy.Publisher('chess_move', String)
    rospy.init_node('chess_RR', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    chess = ChessEngine()
    # chess.print_board()
    # chess.disect_board()
    # return
    arms = ["Arm1", "Arm2"]
    which_arm = 0

    while not rospy.is_shutdown():
        move = chess.get_move()#"Arm1,A1,A2"
        if move['Mate'] is not None:
            print("GAME OVER")
            print("Winning robot: "+arms[which_arm])
            break
        move_string = arms[which_arm] + "," + move['Move']
        publisher.publish(move_string)
        chess.make_move(move['Move'])
        rate.sleep()

        which_arm = (which_arm + 1) % 2

def tester():
    chess = ChessEngine()
    # chess.print_board()
    arms = ["Arm1", "Arm2"]
    which_arm = 0
    chess.print_board()
    # chess.disect_board()
    chess.detect_capture("a8a1")
    return

    while not rospy.is_shutdown():
        move = chess.get_move()#"Arm1,A1,A2"
        print(move)
        if move is None:
            print("GAME OVER")
            print("Winning robot: "+arms[which_arm])
            break
        move_string = arms[which_arm] + "," + move['Move']
        # print(move_string)
        chess.make_move(move['Move'])

        which_arm = (which_arm + 1) % 2

if __name__ == '__main__':
    tester()

    # try:
    #     chess_RR()
    # except rospy.ROSInterruptException:
    #     print("Failed to launch chess demo RvR!")
