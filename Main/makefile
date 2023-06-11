angle_table.o:
	g++ -c hash_table/angle_table.cpp -o angle_table.o
angle_id.o:
	g++ -c hash_table/angle_id.cpp -o angle_id.o
unit_vector_table.o:
	g++ -c unit_vector/unit_vector_table.cpp -o unit_vector_table.o
matrix.o:
	g++ -c matrix.cpp -o matrix.o
basic_functions.o:
	g++ -c basic_functions.cpp -o basic_functions.o
star cam: unit_vector_table.o angle_id.o angle_table.o toojpeg.o matrix.o basic_functions.o
	g++ cam.cpp hash_table/hash.cpp angle_table.o angle_id.o unit_vector_table.o matrix.o basic_functions.o -I/usr/local/include/ -lraspicam -L/opt/vc/lib -lraspicam_cv -lmmal -lmmal_core -lmmal_util -lopencv_core -lopencv_highgui -lopencv_imgproc -lwiringPi -lpthread -o cam.exe
