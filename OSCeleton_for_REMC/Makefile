all: osceleton_for_NMP

liblo:
	cd liblo-0.26-modified;./configure;make

osceleton: liblo
	g++ src/OSCeleton.cpp src/viewer.cpp -O3 -pthread -Wno-write-strings -Iliblo-0.26-modified -I/usr/X11/include -I/usr/include/ni -lOpenNI -lstdc++ -L/usr/X11/lib -lGL -lGLU -lglut liblo-0.26-modified/src/.libs/*.o -o osceleton

osceleton_for_NMP: liblo
	g++ src/OSCeleton_for_NMP.cpp src/viewer.cpp -O3 -pthread -Wno-write-strings -Iliblo-0.26-modified -I/usr/X11/include -I/usr/include/ni -lOpenNI -lstdc++ -L/usr/X11/lib -lGL -lGLU -lglut liblo-0.26-modified/src/.libs/*.o -o osceleton_for_NMP


clean:
	rm -f osceleton;
