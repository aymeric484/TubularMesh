INCLUDE = -I ../ThirdParty/gzstream -I ../Embedding/includes -I ../Topology/includes -I ../Algo/includes -I ../Utils/includes -I. -I/opt/local/include -I./Headers

LIB = -L../Release/lib/ -lutils -ltopology -lembedding -lalgo -lgzstream -lGLEW -lglut -lGL -lz -lIL -ILU -lILUT -lGLU

vesselView: Vessel.o MathTools.o Loader.o vesselView.cpp
	g++ -o -DNOTOPOWARNING -O3 -DNDEBUG -Wall -pipe $(INCLUDE) $(LIB) Vessel.o MathTools.o Loader.o vesselView.cpp -o vesselView

Vessel.o: Vessel.cpp Headers/Vessel.h
	g++ -O3 -c $(INCLUDE) Vessel.cpp

MathTools.o: MathTools.cpp Headers/MathTools.h
	g++ -O3 -c $(INCLUDE) MathTools.cpp

Loader.o: Loader.cpp Headers/Loader.h
	g++ -O3 -c $(INCLUDE) Loader.cpp

clean:
	rm vesselView Vessel.o MathTools.o Loader.o
