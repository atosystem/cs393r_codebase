link_file="/home/dev/cs393r_codebase/config/realCarExp/current_slam.lua"

if [ -L "$link_file" ]; then
    # echo "Symbolic link file $link_file exists. Deleting..."
    rm "$link_file"
    # echo "Symbolic link file deleted."
else
    echo "Symbolic link file $link_file does not exist."
fi
echo "Creating link .lua: $1"
ln -s /home/dev/cs393r_codebase/config/realCarExp/$1.lua   /home/dev/cs393r_codebase/config/realCarExp/current_slam.lua

#ln -s /home/dev/cs393r_codebase/config/realCarExp/offline.lua /home/dev/cs393r_codebase/config/realCarExp/current_slam.lua

echo "Start slam"
./bin/slam