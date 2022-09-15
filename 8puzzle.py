import copy as cp
import queue
from numpy import equal
import numpy
from random import randint
import time
# State
class State:
    def __init__(self,data =None,parent = None,p = 0,h = 0, action = None):
        self.data = data; # la mang 1 chieu
        self.parent = parent; # parent
        self.p = p; # pathcost cua state: gia tri tu initial state  -> current state
        self.h = h; # gia tri cua ham heuristic h(n): vi tri hien tai cua current state -> goal state
        self.action=action
    def clone(self):
        #tao 1 ban sao cho state 
        clone = cp.deepcopy(self)
        return clone
    def Print(self):
        #in ra state theo 2 chieu
        size = 3
        for i in range(size):
            for j in range (size):
                print(self.data[i*size+j],end =' ')              
            print()
        print()
        
    def Key(self):
        temp = ''
        if self.data == None: 
            return None   # Neu input == null thi key == null
        for x in self.data:
            temp += (str)(x) # Tong (chuoi) cua cac phan tu trong mang 1 chieu
        return temp
    def __lt__(self,other):
        if other == None:
            return False # Neu input == null thi key == null
        return self.p + self.h < other.p + other.h # so sanh gia tri f(n) = pathcost + h(n)
    def __eq__(self,other):
        if other == None:
            return False # Neu input == null thi key == null
        return self.Key()==other.Key() # so sanh gia tri f(n) = pathcost + h(n)
    def checkStateNull(self):
        #kiem tra state null hay khong
        if self.data == None: 
            return True # Neu input == null thi key == null
        return False   
    def FindPos(self,num):
        size = 3
        for i in range (size):
            for j in range (size):
                if self.data[i*size+j] == num: 
                    return i,j          # tim vi tri toa do x: hang, y:cot cua phan tu co gia tri la num
        return None
#Operator
class Action:
    def __init__(self,i):
        self.i = i # signal cho operator
    def swap(self,s,x,y,i):
        sz = 3
        clone = s.clone()
        x_new = x
        y_new = y
        #set up
        if i == 0:
            x_new = x+1
            y_new = y
        #set down
        if i == 1:
            x_new = x-1
            y_new = y
        #set left
        if i == 2:
            x_new = x
            y_new = y+1
        #set right
        if i == 3:
            x_new = x
            y_new = y-1
        clone.data[x * sz + y] = s.data[x_new*sz+y_new]
        clone.data[x_new*sz+y_new] = 0
        return clone
    def Up(self,s):
        #thuc hien action Up
        if s.checkStateNull():
            return None             # Neu state hien tai == null -> khong thuc hien lenh
        x,y = s.FindPos(0)
        if x==2:
            return None # Khong the thuc hien lenh, boi vi vi tri trong o cuoi cung
        return self.swap(s,x,y,self.i)
    def Down(self,s):
        #thuc hien action down
        if s.checkStateNull():
            return None             # Neu state hien tai == null -> khong thuc hien lenh
        x,y = s.FindPos(0)
        if x==0:
            return None # Khong the thuc hien lenh, boi vi vi tri trong o cuoi cung
        return self.swap(s,x,y,self.i)
    def Left(self,s):
        #thuc hien action left
        if s.checkStateNull():
            return None             # Neu state hien tai == null -> khong thuc hien lenh
        x,y = s.FindPos(0)
        if y==2:
            return None # Khong the thuc hien lenh, boi vi vi tri trong o cuoi cung
        return self.swap(s,x,y,self.i)
    def Right(self,s):
        #thuc hien action right
        if s.checkStateNull():
            return None             # Neu state hien tai == null -> khong thuc hien lenh
        x,y = s.FindPos(0)
        if y==0:
            return None # Khong the thuc hien lenh, boi vi vi tri trong o cuoi cung
        return self.swap(s,x,y,self.i)
    def Move(self, s):
        # thuc hien action theo signal
        if self.i == 0:
            return self.Up(s)
        if self.i == 1:
            return self.Down(s)
        if self.i == 2:
            return self.Left(s)
        if self.i == 3:
            return self.Right(s)
        return None # khong thuc hien di chuyen

def checkInPriority(Open, tmp):
    if tmp == None:
        return False 
    return (tmp in Open.queue)
def Equal(Current,Goal):
    # kiem tra xem 2 state giong hay hay khong, cu the muc dich la kiem tra co = goal state hay chua
    if Current == None:
        return False 
    return Current.Key()==Goal.Key()
def Path(state):
    step=''
    if state.parent != None:
        Path(state.parent)
        if state.action.i==0:
            step='up'
        elif state.action.i==1:
            step='down'
        elif state.action.i==2:
            step='left'
        else:
            step='right'
        print(step)
    state.Print()
#ham heusristic thu nhat
def Hx(state, goal):
    size = 3
    h = 0
    for i in range (size):
        for j in range (size):
            x,y=goal.FindPos(state.data[i*size+j]) 
            h +=numpy.abs(x-i)+numpy.abs(y-j)
    return h
#ham heusristic thu 2
def Hx2(state, goal):
    size = 3
    h = 0
    for i in range (size):
        for j in range (size):
            if state.data[i*size+j] != goal.data[i*size+j]:
                h +=1
    return h
#lay ham heusristic tot nhat de chay thuat toan
def Hx_max(state,goal):
    if Hx(state,goal)>=Hx2(state,goal):
        return Hx(state,goal)
    return Hx2(state,goal)
def A_star_Search(problem, goal):
    Open = queue.PriorityQueue()
    Closed = queue.PriorityQueue()
    problem.p = 0
    problem.h = Hx_max(problem,goal)
    Open.put(problem)
    while True:
        if Open.empty() == True:
            print('khong co solution')
            return
        current = Open.get()
        Closed.put(current)
        if Equal(current,goal) == True:
            print('da tim thay solution')
            Path(current)
            return
        # tim tat ca state con
        for i in range (4):
            action = Action(i)
            child = action.Move(current)
            if(child == None):
                continue
            check1 = checkInPriority(Open,child)
            check2 = checkInPriority(Closed,child)
            if not check1 and not check2:
                child.parent = current
                child.action = action
                child.p = current.p+1
                child.h = Hx_max(child,goal)
                Open.put(child)
def init(num):
    #tao problem va goal state
    goal = State()
    size = 3
    goal.data = [1,2,3,4,5,6,7,8,0]
    problem = goal.clone()
    for i in range(num):
        op = Action(randint(0,4))
        tmp = op.Move(problem)
        if tmp != None:
            problem = tmp
    return problem, goal 
problem,goal = init(50)
problem.Print()
goal.Print()
t1 = time.time()
A_star_Search(problem,goal)
t2 = time.time()
print('time: ',(t2-t1))



