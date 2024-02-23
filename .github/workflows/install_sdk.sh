cd /home/runner/work
git clone https://github.com/kroshu/kuka-external-control-sdk.git
mkdir -p /home/runner/work/kuka-external-control-sdk/kuka-external-control-sdk/build
cd /home/runner/work/kuka-external-control-sdk/kuka-external-control-sdk/build
cmake ..
make install
