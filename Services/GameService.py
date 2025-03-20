#!/usr/bin/env python3
from random import random

import rospy
import json
from std_srvs.srv import Trigger, TriggerResponse

class GameService():

    def __init__(self):

        self.GRID_SIZE = 3  # Example grid size

        # Define service
        rospy.Service('/get_game_move', Trigger, self.calculate_next_move)
        rospy.loginfo("Game service started.")

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def preveri_zmagovalca(self,grid):
        for i in range(self.GRID_SIZE):
            # Preveri vrstice
            if grid[i][0] == grid[i][1] == grid[i][2] != "":
                znak = grid[i][0]
                return znak
            # Preveri stolpce
            if grid[0][i] == grid[1][i] == grid[2][i] != "":
                znak = grid[0][i]
                return znak

        # Preveri diagonale
        if grid[0][0] == grid[1][1] == grid[2][2] != "":
            znak = grid[0][0]
            return znak
        if grid[0][2] == grid[1][1] == grid[2][0] != "":
            znak = grid[0][2]
            return znak

        return None

    # Funkcija za preverjanje prostih polj
    def preveri_prazna_polja(self,grid):
        prazna_polja = []
        for i in range(self.GRID_SIZE):
            for j in range(self.GRID_SIZE):
                if grid[i][j] == "":
                    prazna_polja.append((i, j))
        if prazna_polja == []:
            # print("Matrika je polna")
            st_praznih_polj = 0
        else:
            st_praznih_polj = len(prazna_polja)
        return prazna_polja, st_praznih_polj

    # Funkcija za preverjanje, ali ima igralec 2 znaka v vrsti, stolpcu ali diagonali
    def preveri_dva_znaka(self,grid, znak):
        for i in range(self.GRID_SIZE):
            # Preveri vrstice
            if grid[i].count(znak) == 2 and grid[i].count("") == 1:
                return (i, grid[i].index(""))

            # Preveri stolpce
            stolpec = [grid[0][i], grid[1][i], grid[2][i]]
            if stolpec.count(znak) == 2 and stolpec.count("") == 1:
                return (stolpec.index(""), i)

        # Preveri diagonale
        diagonal1 = [grid[0][0], grid[1][1], grid[2][2]]
        if diagonal1.count(znak) == 2 and diagonal1.count("") == 1:
            index = diagonal1.index("")
            return (index, index)

        diagonal2 = [grid[0][2], grid[1][1], grid[2][0]]
        if diagonal2.count(znak) == 2 and diagonal2.count("") == 1:
            index = diagonal2.index("")
            return (index, 2 - index)

        return None

    # Funkcija za računalniško potezo
    def poteza_racunalnika(self,grid):
        # Najprej preveri, ali lahko računalnik zmaga
        zmaga_poteza = self.preveri_dva_znaka(grid, "O")
        if zmaga_poteza:
            return {'position': zmaga_poteza, "sign": "O"}

        # Nato preveri, ali mora blokirati igralčevo zmago
        blokira_poteza = self.preveri_dva_znaka(grid, "X")
        if blokira_poteza:
            return {'position': blokira_poteza, "sign": "O"}

        # Če ni nevarnosti, naključno izberemo polje
        prazna_polja = self.preveri_prazna_polja(grid)[0]
        if prazna_polja:
            izbira = random.choice(prazna_polja)
            return {'position': izbira, "sign": "O"}

        return grid

    # Funkcija za preverjanje razlike med številom "X" in "O"
    def preveri_razliko(self,grid):
        stevilo_križcev = sum(row.count("X") for row in grid)
        stevilo_krozcev = sum(row.count("O") for row in grid)

        if abs(stevilo_križcev - stevilo_krozcev) > 1:
            return True
        return False

    # Funkcija, ki bo obravnavala vhodno matriko (od kamere ali kakšnega drugega vira)
    def obdelaj_vhodno_matriko(self,vhodna_matrika):
        print("Prejeta vhodna matrika:")
        for vrstica in vhodna_matrika:
            print(vrstica)

        # Preveri, ali je razlika večja od 1
        if self.preveri_razliko(vhodna_matrika):
            print("Računalnik ne bo izvedel poteze, ker je prekršil pravilo.")
            return vhodna_matrika  # Poteza se ne izvede, ker je razlika večja od 1

        # Računalnik se odzove na prejeto matriko
        nova_matrika = self.poteza_racunalnika(vhodna_matrika)

        return nova_matrika

    def calculate_next_move(self,req):

        try:
            board = json.loads(req.data)  # Convert input string to a 2D list
        except json.JSONDecodeError:
            return Trigger({"success": False, "message": json.dumps({"error": "Invalid JSON"})})

        move = self.poteza_racunalnika(board)

        return Trigger({"success": True, "message": json.dumps({'move': move})})



    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        # this code is run at ctrl + c
        self.ctrl_c = True

if __name__ == '__main__':
    # initialise node
    #rospy.init_node('led_actuator')
    # initialise class
    game_service = GameService()
    try:
        # loop
        rospy.spin()
    except rospy.ROSInterruptException:
        pass