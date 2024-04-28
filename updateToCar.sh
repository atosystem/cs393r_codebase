echo "Update Naviation files to robot..."

# scp -r ./config/particle_filter.lua amrl_user@10.159.65.41:/home/amrl_user/projects/cs393r_codebase/config
# scp -r ./config/navigation.lua amrl_user@10.159.65.41:/home/amrl_user/projects/cs393r_codebase/config
scp -r ./config/slam.lua amrl_user@10.159.65.41:/home/amrl_user/projects/cs393r_codebase/config

# scp -r ./src/particle_filter amrl_user@10.159.65.41:/home/amrl_user/projects/cs393r_codebase/src
# scp -r ./src/navigation amrl_user@10.159.65.41:/home/amrl_user/projects/cs393r_codebase/src
scp -r ./src/slam amrl_user@10.159.65.41:/home/amrl_user/projects/cs393r_codebase/src
echo "Compile on Robot..."
cat ./robot_compile.sh | ssh amrl_user@10.159.65.41
echo "Done"