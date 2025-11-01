import heapq, random, torch
from collections import deque

class TopKBuffer:
    """
    Keep the K *lowest-angle* latent codes.
    Each heap node = (-angle, z0)
    """
    def __init__(self, buffer_size=50, k=10):
        self.top_k  = k
        self.hp = []          # min-heap for top-K items 
        self.heap = deque(maxlen=buffer_size)
        self.counter = 0 
    def push_single(self, angle, z0):
        self.counter += 1
        item = (float(angle), self.counter, z0.detach().cpu())
        if len(self.hp) < self.top_k:
            heapq.heappush(self.hp, item)
        else:
            if item[0] > self.hp[0][0]:     #a better simulation results
                heapq.heapreplace(self.hp, item)
                is_topK = True
            else:
                self.heap.append(item)

    def push(self, items):
        for item in items:
            self.push_single(item[0], item[1])
            
    def sample(self, n):
        all_items = self.hp + list(self.heap)  # self.hp is already a list
        return random.sample(all_items, n)

    def __len__(self):
        return len(self.hp) + len(self.heap)