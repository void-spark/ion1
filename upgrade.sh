#!/bin/bash

ssh root@debian.fritz.box "rm -rf /tmp/esp32 ; mkdir -p -v /tmp/esp32"
scp "$PWD/build/ion1.bin" root@debian.fritz.box:/tmp/esp32
ssh root@debian.fritz.box " \
docker start esp_fw || docker run -d --name esp_fw -v esp_fw_data:/usr/share/nginx/html --publish 8032:80 nginx:alpine ; \
docker cp /tmp/esp32 esp_fw:/usr/share/nginx/html ; \
rm -rf /tmp/esp32 ; \
docker exec -w /usr/share/nginx/html/esp32 -i esp_fw sh -c 'chown -R root:root . ; ls -lah' "

docker run --init -it --rm efrecon/mqtt-client sh -c "pub -h debian.fritz.box -t 'devices/083af25162c4/\$update' -m true ; sub -h debian.fritz.box -t 'devices/083af25162c4/\$state' -C 3 -R -v"
