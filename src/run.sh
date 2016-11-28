g++ -Werror `pkg-config --cflags --libs opencv` AStar.cpp DisplayImage.cpp -o visionPlanning
if [ $? -eq 0 ]; then
	echo compile success
    ./visionPlanning 0 ../videos/P1020780.m4v
else
    echo compile failed
fi