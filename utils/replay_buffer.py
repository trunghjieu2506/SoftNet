import heapq, random, torch

class TopKBuffer:
    """
    Keep the K *lowest-angle* latent codes.
    Each heap node = (-angle, z0)
    """
    def __init__(self, k=50):
        self.k  = k
        self.hp = []          # max-heap on −angle

    def push(self, angle, z0):
        item = (-float(angle), z0.detach().cpu())
        if len(self.hp) < self.k:
            heapq.heappush(self.hp, item)
        else:
            # hp[0] is *worst* (largest −angle ⇒ smallest +angle)
            if -item[0] < -self.hp[0][0]:      # better angle
                heapq.heapreplace(self.hp, item)

    def sample(self, n):
        return random.sample(self.hp, n)

    def __len__(self):
        return len(self.hp)
