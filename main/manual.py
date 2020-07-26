from interface import Interface, C, Const, Robot
from planGenerator import Generator
from simulation import ManualSimulation


Interface.setup(Const['WINDOW'], 'Plan', font_color=C.BROWN, FPS=30)

Interface.set_robot(Robot((600,600), 0))

g = Generator(Const['WINDOW'])
g.load_data()

Interface.set_plan(g.data[0])

Interface.keep_track_mov(C.YELLOW)

simulation = ManualSimulation()

while Interface.running:
    pressed, events = Interface.run()
    simulation(pressed)
    
simulation.store()