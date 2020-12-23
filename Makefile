PROGRAM = slerp
GCC = g++
GCCFLAGS = -std=c++17 -Wall
LIBS = -lGL -lGLU -lglut

$(PROGRAM): slerp.o transform.o
	$(GCC) $(GCCFLAGS) -o $@ $^ $(LIBS)
slerp.o: slerp.cpp
	$(GCC) $(GCCFLAGS) -c $< -o $@ 
transform.o: transform.cpp
	$(GCC) $(GCCFLAGS) -c $< -o $@

.PHONY: clean

clean:
	rm *.o $(PROGRAM)