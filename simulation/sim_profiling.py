import time, Sofa, Sofa.Simulation
from idealscene import createScene

def main():
    t0 = time.time()
    root = Sofa.Core.Node("root")
    createScene(root)
    t1 = time.time()
    
    Sofa.Simulation.init(root)
    t2 = time.time()
    
    # N = 10000
    # for _ in range(N):
    #     sim.animate(root, root.dt.value)
    # t3 = time.time()
    
    print(f"scene_build: {t1-t0:.3f}s")
    print(f"init_graph : {t2-t1:.3f}s")
    # print(f"simulate   : {t3-t2:.3f}s  ({N/(t3-t2):.1f} FPS)")

main()