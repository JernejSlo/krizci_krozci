import random
import time
import copy

# Nastavitve
GRID_SIZE = 3
stevec = 0

# Funkcija za preverjanje zmagovalca
def preveri_zmagovalca(grid):
    for i in range(GRID_SIZE):
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
def preveri_prazna_polja(grid):
    prazna_polja = []
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            if grid[i][j] == "":
                prazna_polja.append((i, j))
    if prazna_polja == []:
        # print("Matrika je polna")
        st_praznih_polj = 0
    else:
        st_praznih_polj = len(prazna_polja)
    return prazna_polja,st_praznih_polj

# Funkcija za preverjanje, ali ima igralec 2 znaka v vrsti, stolpcu ali diagonali
def preveri_dva_znaka(grid, znak):
    for i in range(GRID_SIZE):
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
def poteza_racunalnika(grid):
    # Najprej preveri, ali lahko računalnik zmaga
    zmaga_poteza = preveri_dva_znaka(grid, "O")
    if zmaga_poteza:
        grid[zmaga_poteza[0]][zmaga_poteza[1]] = "O"
        if preveri_razliko(grid) == True:
            grid[zmaga_poteza[0]][zmaga_poteza[1]] = ""
        else:
            print("Računalnik je zmagal!")
        return zmaga_poteza
    
    # Nato preveri, ali mora blokirati igralčevo zmago
    blokira_poteza = preveri_dva_znaka(grid, "X")
    if blokira_poteza:
        grid[blokira_poteza[0]][blokira_poteza[1]] = "O"
        if preveri_razliko(grid) == True:
            grid[blokira_poteza[0]][blokira_poteza[1]] = ""
        else:
            print("Računalnik blokira zmago igralca!")

        return grid
    
    # Če ni nevarnosti, naključno izberemo polje
    prazna_polja = preveri_prazna_polja(grid)[0]
    if prazna_polja:
        izbira = random.choice(prazna_polja)
        grid[izbira[0]][izbira[1]] = "O"
        if preveri_razliko(grid) == True:
            grid[izbira[0]][izbira[1]] = ""
        else:
            print("Računalnik je izbral naključno polje!")
    
    return grid

# Funkcija za preverjanje razlike med številom "X" in "O"
def preveri_razliko(grid):
    stevilo_križcev = sum(row.count("X") for row in grid)
    stevilo_krozcev = sum(row.count("O") for row in grid)
    
    if abs(stevilo_križcev - stevilo_krozcev) > 1:
        return True
    return False

# Funkcija, ki bo obravnavala vhodno matriko (od kamere ali kakšnega drugega vira)
def obdelaj_vhodno_matriko(vhodna_matrika):
    print("Prejeta vhodna matrika:")
    for vrstica in vhodna_matrika:
        print(vrstica)
    
    # Preveri, ali je razlika večja od 1
    if preveri_razliko(vhodna_matrika):
        print("Računalnik ne bo izvedel poteze, ker je prekršil pravilo.")
        return vhodna_matrika  # Poteza se ne izvede, ker je razlika večja od 1
    
    # Računalnik se odzove na prejeto matriko
    nova_matrika = poteza_racunalnika(vhodna_matrika)
    
    return nova_matrika

# Testiranje klica funkcije
# Predpostavimo, da je to matrika, ki jo računalnik prejme kot vhod:
matrix = [
    ["", "", ""],
    ["", "", ""],
    ["", "", ""]
]

stanje = 1
matrix_copy = 0

while True:
    # KODA --> slika polje in vrne matriko
    
    if(preveri_prazna_polja(matrix)[1]==9):
        if stanje == 0: #začne računalnik
            matrix = poteza_racunalnika(matrix)
    else:
        if(preveri_razliko(matrix)==True):
            print("Kršenje pravila.")
            break
        if preveri_zmagovalca(matrix) == "O"  or preveri_zmagovalca(matrix) == "X":
            print("Konec igre, zmagovalec je:",preveri_zmagovalca(matrix))
            break
        if(matrix != matrix_copy): 
            matrix = poteza_racunalnika(matrix)
    matrix_copy = copy.deepcopy(matrix)

    # KODA --> robot postavi krogec na zeljeno pozicijo

    print(matrix[0])
    print(matrix[1])
    print(matrix[2])
    time.sleep(5)
        

