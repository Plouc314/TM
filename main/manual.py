from interface import Interface, C, Const, Robot
from planGenerator import Generator
from simulation import ManualSimulation


Interface.setup(Const['WINDOW'], 'Plan', font_color=C.BROWN, FPS=30)

Interface.set_robot(Robot((600,600), 0))

g = Generator(Const['WINDOW'])
g.load_data()

Interface.set_plan(g.data[4])

Interface.add_dps(g.data[4][0:1], C.BLUE, is_permanent=True)
Interface.add_dps(g.data[4][1:2], C.LIGHT_BLUE, is_permanent=True)
Interface.add_dps(g.data[4][2:3], C.BLUE, is_permanent=True)

Interface.keep_track_mov(C.YELLOW)

simulation = ManualSimulation()

while Interface.running:
    pressed, events = Interface.run()
    simulation(pressed)
    
simulation.store()