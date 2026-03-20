# Сброс

cd ~/sverk-ros2 

sudo systemctl stop sverk-ros2-docker.service

docker compose down

git reset --hard
git pull # login and password

git submodule sync --recursive
git submodule update --init --recursive --remote # login and password
git submodule status

docker volume rm sverk-ros2_shared_fs

docker rmi -f $(docker images -q) 2>/dev/null || true

docker network prune -f

docker builder prune -a -f

# Check space
df -h

docker compose up --build