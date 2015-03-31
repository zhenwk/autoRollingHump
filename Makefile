# Compile snake simulator with or without the visualization.
SnakeSim:
	g++ SnakeSim.cpp -c -o SnakeSim.o

SnakeSimViz:
	g++ SnakeSim.cpp -c -DVISUALIZATION -o SnakeSim.o
	
dSnakeSimViz:
	g++ -g SnakeSim.cpp -c -DVISUALIZATION -o SnakeSim.o

GaitHandler:
	g++ GaitHandler.cpp -c -o GaitHandler.o

GaitHandlerViz:
	g++ GaitHandler.cpp -c -DVISUALIZATION -o GaitHandler.o
	
dGaitHandlerViz:
	g++ -g GaitHandler.cpp -c -DVISUALIZATION -o GaitHandler.o

Obstacle:
	g++ Obstacle.cpp -c -o Obstacle.o

example: SnakeSim GaitHandler
	g++ example.cpp SnakeSim.o GaitHandler.o -o example -lode -lGL -lGLU -lglut

exampleViz: SnakeSimViz GaitHandlerViz
	g++ example.cpp SnakeSim.o GaitHandler.o -o example -lode -lGL -lGLU -lglut -lfreeimage -DVISUALIZATION
	
dexampleViz: dSnakeSimViz dGaitHandlerViz
	g++ -g example.cpp SnakeSim.o GaitHandler.o -o example -lode -lGL -lGLU -lglut -lfreeimage -DVISUALIZATION

clean:
	rm -f SnakeSim.o GaitHandler.o example
	rm -rf latex html
	rm -rf desiredAngles.txt desiredAngles_cp.txt
	rm -rf sensorData.txt sensorData_cp.txt

good-stuff: SnakeSimViz GaitHandlerViz
	g++ good-stuff.cpp SnakeSim.o GaitHandler.o -o run-me -lode -lGL -lGLU -lfreeimage -DVISUALIZATION

doc:
	doxygen
