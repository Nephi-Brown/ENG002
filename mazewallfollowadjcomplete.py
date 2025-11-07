



matrix = [[0 for _ in range(25)] for _ in range(25)]
for i in range(25):
    for j in range(25):
        if i == 0 or i == 24 or j == 0 or j == 24: 
            matrix[i][j] = 11
            

for i in range(11,14):
    for j in range(11,14):
        matrix[i][j] = -1
        
for i in range(0,25):
    matrix[i][2] = 11
for i in range(0,25):
    matrix[i][4] = 11
for i in range(0,25):
    matrix[i][6] = 11
for i in range(0,25):
    matrix[i][8] = 11
for i in range(0,25):
    matrix[i][10] = 11
for i in range(0,11):
    matrix[i][12] = 11
for i in range(14,25):
    matrix[i][12] = 11
for i in range(0,25):
    matrix[i][14] = 11
for i in range(0,25):
    matrix[i][16] = 11
for i in range(0,25):
    matrix[i][18] = 11
for i in range(0,25):
    matrix[i][20] = 11
for i in range(0,25):
    matrix[i][22] = 11
for i in range(0,25):
    matrix[i][24] = 11
matrix[17][2] = 0
matrix[6][4] = 0
matrix[23][6] = 0
matrix[18][8] = 0

matrix[23][4] = 0
matrix[21][7] = 11
matrix[13][3] = 11
matrix[14][4] = 0
matrix[13][5] = 11
matrix[12][6] = 0
matrix[14][6] = 0
matrix[2][6] = 0
matrix[3][7] = 11
matrix[1][8] = 0
matrix[2][9] = 11
matrix[1][10] = 0
matrix[1][12] = 0
matrix[2][11] = 11
matrix[3][13] = 11
matrix[2][14] = 0
matrix[12][14] = 0








x, y = 1, 1  

def print_matrix():
    for i in range(len(matrix)):
        row = ""
        for j in range(len(matrix[i])):
            if i == x and j == y:
                row += " R "   
            else:
                row += f"{matrix[i][j]:2} "
        print(row)
    print()
    


    
        
def moveup():
    global x, y
    if matrix[x-1][y] != 11:  
        x -= 1
    if matrix[x+1][y] == 0:
        matrix[x+1][y] = 2
    elif matrix[x+1][y] == 2:
        matrix[x+1][y] = 4
    else:
        matrix[x+1][y] = matrix[x+2][y] + 2

def movedown():
    global x, y
    if matrix[x+1][y] != 11:
        x += 1
    if matrix[x-1][y] == 0:
        matrix[x-1][y] = 2
    elif matrix[x-1][y] == 2:
        matrix[x-1][y] = 4
    else:
        matrix[x-1][y] = matrix[x-2][y] + 2

def moveleft():
    global x, y
    if matrix[x][y-1] != 11:
        y -= 1
    if matrix[x][y+1] == 0:
        matrix[x][y+1] = 2
    elif matrix[x][y+1] == 2:
        matrix[x][y+1] = 4
    else:
        matrix[x][y+1] = matrix[x][y+1] + 2

def moveright():
    global x, y
    if matrix[x][y+1] != 11:
        y += 1
    if matrix[x][y-1] == 0:
        matrix[x][y-1] = 2
    elif matrix[x][y-1] == 2:
        matrix[x][y-1] = 4
    else:
        matrix[x][y-1] = matrix[x][y-1] + 2

import time



         
def lookaround():
    global x, y , movepos, cell_left, cell_right, cell_below, cell_above
    
    cell_below = matrix[x+1][y]
    cell_above = matrix[x-1][y]
    cell_right = matrix[x][y+1]
    cell_left = matrix[x][y-1]
    
    movepos = [
    (cell_right, 'right'),
    (cell_below, 'down'),
    (cell_left, 'left'),
    (cell_above, 'up')
]


    movepos.sort()
    
    
     
    
def pathfind():
     global x, y, movepos, cell_left, cell_right, cell_below, cell_above
     lookaround()
     
     value, direction = movepos[0]
     
     
     if direction == 'down':
         movedown()
     elif direction == 'right':
         moveright()
     elif direction == 'up':
         moveup()
     elif direction == 'left':
         moveleft()
        
         
    



while True:
    
    for i in range(50):
        pathfind()
        print_matrix()
        time.sleep(0.5)
        
             
    
    
    
         
    
    
    
    
    
    


    




   











