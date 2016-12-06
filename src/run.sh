g++ -O2 -Werror `pkg-config --cflags --libs opencv` AStar.cpp DisplayImage.cpp -o visionPlanning
if [ $? -eq 0 ]; then
	echo compile success
    ./visionPlanning 1 /Users/gamjatang1/documents/workspace2/Testing/videos/P1020780.m4v
else
    echo compile failed
fi
