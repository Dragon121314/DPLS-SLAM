echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
# mkdir build

cd build
rm -rf *
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j12

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

# mkdir build
cd build
rm -rf *
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j12

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

# mkdir build
cd build
rm -rf *
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j12

cd ../../../

cd line_descriptor/build
rm -rf *
cmake ..
make -j12

cd ../../

cd yolov8/build
rm -rf *
cmake ..
make -j12

cd ../../




echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."

# mkdir build
cd build
rm -rf *
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j12






#代码改动：

# tracking.cc的PreintegrateIMU()函数计算两帧imu加速度时(i==0) && (i<(n-1)情况
#acc = (mvImuFromLastFrame[i].a+ mvImuFromLastFrame[i+1].a-(mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tini/tab)) *0.5f;
#改为acc = (mvImuFromLastFrame[i+1].a+ mvImuFromLastFrame[i].a-(mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tini/tab)) *0.5f;
#包括陀螺仪及(i>0) && (i==(n-1))情况



