import getch

key = getch()

def test():
    global key
    while True:
        print(key == 13)
        if key == 13:
            print('hi')

if __name__ == '__main__':
    test()