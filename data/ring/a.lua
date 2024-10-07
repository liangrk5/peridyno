-- Lua script.
p=tetview:new()
p:load_mesh("C:/Users/90418/Desktop/peridyno/peridyno/data/ring/ring.ele")
rnd=glvCreate(0, 0, 500, 500, "TetView")
p:plot(rnd)
glvWait()
