echo "Update Naviation files to robot..."
scp -r ./src/particle_filter amrl_user@10.159.66.168:/home/amrl_user/projects/cs393r_codebase/src
scp -r ./config/particle_filter.lua amrl_user@10.159.66.168:/home/amrl_user/projects/cs393r_codebase/config
echo "Compile on Robot..."
cat ./robot_compile.sh | ssh amrl_user@10.159.66.168
echo "Done"