from typing import *

T = TypeVar('T')

def cmpLess(a : T, b : T) -> bool:
    return a < b

def cmpGreater(a : T, b : T) -> bool:
    return a > b

class Heap(Generic[T]):
    def __init__(self, content : List[T] = [], cmp : Callable[[T, T], bool] = lambda a,b: a < b):
        self.content  : List[T]                = content
        self.cmp      : Callable[[T, T], bool] = cmp
        self.key_list : Dict[T, bool]          = {}
        
        self.set_keys()
        self.heapify()
    
    def __repr__(self):
        return f"{self.content}"
    
    def set_keys(self) -> None:
        for value in self.content:
            self.key_list[value] = True
    
    def swap(self, idx1 : int, idx2 : int) -> None:
        tmp : T = self.content[idx1]
        self.content[idx1] = self.content[idx2]
        self.content[idx2] = tmp
    
    def bubbleDown(self, idx : int) -> None:
        currVal  : T = self.content[idx]
        leftIdx  : int = idx * 2 + 1  # 0 -> 1, 1 -> 3, 2 -> 5, ...
        rightIdx : int = idx * 2 + 2  # 0 -> 2, 1 -> 4, 2 -> 6, ...
        
        # no children
        if leftIdx >= len(self.content):
            return
        
        # left child only
        elif rightIdx >= len(self.content):
            leftChild : T = self.content[leftIdx]
            
            # if left child has higher priority than root
            if self.cmp(leftChild, currVal):
                # swap root with left child
                self.swap(idx, leftIdx)
                
                # keep bubbling from left child
                self.bubbleDown(leftIdx)
        
        # both children
        else:
            # determine which child has a higher priority
            leftChild  : T = self.content[leftIdx]
            rightChild : T = self.content[rightIdx]
            
            priorityIdx : int = leftIdx if self.cmp(leftChild, rightChild) else rightIdx
            priorityVal : T   = self.content[priorityIdx]
            
            # if either child has a higher priority than root
            if self.cmp(priorityVal, currVal):
                # swap root with priority child
                self.swap(idx, priorityIdx)
                
                # keep bubbling down from priority child
                self.bubbleDown(priorityIdx)
    
    def bubbleUp(self, idx : int) -> None:
        # if root exists for child
        if idx > 0:
            parentIdx : int = (idx - 1) // 2  # 1, 2 -> 0, 3, 4 -> 1, 5, 6 -> 2, ...
            currVal   : T   = self.content[idx]
            parentVal : T   = self.content[parentIdx]
            
            # if parent has lower priority than child
            if self.cmp(currVal, parentVal):
                # swap parent with child
                self.swap(idx, parentIdx)
                
                # continue bubbling up from parent
                self.bubbleUp(parentIdx)
    
    def heapify(self) -> None:
        for i in range(len(self.content)):
            idx = len(self.content) - i - 1
            
            self.bubbleDown(idx)
    
    def empty(self) -> bool:
        return len(self.content) == 0
    
    def top(self) -> T:
        return self.content[0]
    
    def pop(self) -> T:
        ref : T = self.top()  # reference to current root of heap
        
        if len(self.content) > 1:
            # replace root with last element, and remove last element
            self.content[0] = self.content[len(self.content) - 1]
            self.content.pop(len(self.content) - 1)
    
            # bubble down new root into appropriate place
            self.bubbleDown(0)
        else:
            # remove root of heap
            self.content.pop(0)
        
        del self.key_list[ref]
        
        return ref
    
    def push(self, value : T) -> None:
        # add new value to end of heap
        self.content.append(value)
        
        # bubble up new child into appropriate place
        self.bubbleUp(len(self.content) - 1)
        
        self.key_list[value] = True
    
    def contains(self, value : T) -> bool:
        return value in self.key_list
