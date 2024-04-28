link_file="config/realCarExp/current_slam"

if [ -L "$link_file" ]; then
    # echo "Symbolic link file $link_file exists. Deleting..."
    rm "$link_file"
    # echo "Symbolic link file deleted."
else
    # echo "Symbolic link file $link_file does not exist."
fi
echo "Createing link .lua: $1"
ls -s ./config/realCarExp/$1.lua ./config/realCarExp/current_slam.lua

wcho "Start slam"
./bin/slam
