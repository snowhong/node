import os
import time

def atest():
    a=0
    b=[1,2,3,4,5]
    while(1):
        time.sleep(0.1)
        a= a+1
        for i in b:
            if(a<4):
                print a
            elif a>6:
                print 'a>6'
                if a>8:
                    return a
            
        
if __name__ == '__main__':
    atest()
