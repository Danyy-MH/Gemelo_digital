variable_global = [0, 0 ,0]

def prueba1():
    variable_global[0] = 1
    print(variable_global)
    prueba2()

def prueba2():
    variable_global[1] = 2
    print(variable_global)

def main():
    variable_global[2] = 3
    print(variable_global)
    prueba1()

main()