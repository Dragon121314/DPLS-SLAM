#!/bin/bash
pathDatasetEuroc='/home/robot/Datasets/EuRoC' #Example, it is necesary to change it by the dataset path
pathDatasetTUM='/home/robot/Datasets/TUM' 
pathDatasetOpenLORIS='/home/robot/Datasets/OpenLORIS' 
txt=".txt"
cd build
cmake ..
make -j12


# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM1.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg1_room
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM3.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_xyz
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/RealSense_D435i.yaml "$pathDatasetTUM"/HighDynamic

# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM1.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg1_xyz
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM2.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg2_desk
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM3.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_halfsphere
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM3.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_rpy
./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM3.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_static ../DPLS-SLAM
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM3.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_xyz



#===============================================rgbd_dataset_freiburg3_walking_xyz===================================================
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/RealSense_D435i.yaml /home/robot/Datasets/TUM/HighDynamic 
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/RealSense_D435i.yaml "$pathDatasetTUM"/Comp_VO ../Comp_VO
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/RealSense_D435i.yaml "$pathDatasetTUM"/Static_VO ../Static_VO
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/RealSense_D435i.yaml "$pathDatasetTUM"/Dynamic_VO ../Dynamic_VO
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_xyz/groundtruth.txt ../result/fr3_walking_xyz.txt --plot ../result/fr3_walking_xyz.pdf






# #===============================================market1-1===================================================
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/OpenLORIS.yaml "$pathDatasetOpenLORIS"/market1-1 "$pathDatasetOpenLORIS"/market1-1/associate.txt market1-1
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetOpenLORIS"/market1-1/groundtruth.txt market1-1.txt --plot market1-1.pdf



# #===============================================rgbd_dataset_freiburg1_room===================================================
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM1.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg1_room "$pathDatasetTUM"/rgbd_dataset_freiburg1_room/associate.txt ../result/fr1_room
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg1_room/groundtruth.txt ../result/fr1_room.txt --plot ../result/fr1_room.pdf

#===============================================rgbd_dataset_freiburg1_desk2===================================================
# ./rgbd_tum
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM1.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg1_desk2 ../Examples/RGB-D/associations/fr1_desk2.txt
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg1_desk2/groundtruth.txt CameraTrajectory.txt --plot ../Examples/RGB-D/fr1_desk2_rgbd.pdf
# evo_ape tum "$pathDatasetTUM"/rgbd_dataset_freiburg1_desk2/groundtruth.txt CameraTrajectory.txt -a -s


# ===============================================rgbd_dataset_freiburg1_xyz===================================================
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM1.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg1_xyz ../Examples/RGB-D/associations/fr1_xyz.txt
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg1_xyz/groundtruth.txt CameraTrajectory.txt --plot ../Examples/RGB-D/fr1_xyz_rgbd.pdf



# # # #===============================================rgbd_dataset_freiburg2_large_no_loop===================================================
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM2.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg2_large_no_loop "$pathDatasetTUM"/rgbd_dataset_freiburg2_large_no_loop/associate.txt ../result/dataset-fr2_large_no_loop_rgb
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg2_large_no_loop/groundtruth.txt ../result/dataset-fr2_large_no_loop_rgb.txt --plot ../result/fr2_large_no_loop_rgbd.pdf

# # #===============================================rgbd_dataset_freiburg2_large_with_loop===================================================
# # ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM2.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg2_large_with_loop "$pathDatasetTUM"/rgbd_dataset_freiburg2_large_with_loop/associate.txt ../result/dataset-fr2_large_with_loop_rgb
# # python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg2_large_with_loop/groundtruth.txt ../result/dataset-fr2_large_with_loop_rgb.txt --plot ../result/fr2_large_with_loop_rgbd.pdf


# # evo_ape tum /home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_rpy/groundtruth.txt result/fr3_walking_rpy.txt -a -s
# # evo_rpe tum /home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_rpy/groundtruth.txt result/fr3_walking_rpy.txt -a -s
# # evo_traj tum /result/fr3_walking_rpy.txt --ref /home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_rpy/groundtruth.txt -p -a

# # evo_ape tum /home/robot/Datasets/TUM/rgbd_dataset_freiburg3_sitting_rpy/groundtruth.txt result/fr3_sitting_rpy.txt -a -s
# # evo_rpe tum /home/robot/Datasets/TUM/rgbd_dataset_freiburg3_sitting_rpy/groundtruth.txt result/fr3_sitting_rpy.txt -a -s
# # evo_traj tum /result/fr3_sitting_rpy.txt --ref /home/robot/Datasets/TUM/rgbd_dataset_freiburg3_sitting_rpy/groundtruth.txt -p -a

# # evo_ape tum /home/robot/Datasets/TUM/rgbd_dataset_freiburg3_sitting_static/groundtruth.txt result/fr3_sitting_static.txt -a -s
# # evo_rpe tum /home/robot/Datasets/TUM/rgbd_dataset_freiburg3_sitting_static/groundtruth.txt result/fr3_sitting_static.txt -a -s
# # evo_traj tum /result/fr3_sitting_static.txt --ref /home/robot/Datasets/TUM/rgbd_dataset_freiburg3_sitting_static/groundtruth.txt -p -a


# #===============================================rgbd_dataset_freiburg3_sitting_xyz===================================================
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM3.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_xyz "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_xyz/associate.txt ../result/fr3_sitting_xyz
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_xyz/groundtruth.txt ../result/fr3_sitting_xyz.txt --plot ../result/fr3_sitting_xyz.pdf

# # #===============================================rgbd_dataset_freiburg3_sitting_rpy===================================================
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM3.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_rpy "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_rpy/associate.txt ../result/fr3_sitting_rpy
# # python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_rpy/groundtruth.txt ../result/fr3_sitting_rpy.txt --plot ../result/fr3_sitting_rpy.pdf

# #===============================================rgbd_dataset_freiburg3_sitting_halfsphere===================================================
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM3.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_halfsphere "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_halfsphere/associate.txt ../result/fr3_sitting_halfsphere
# # python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_halfsphere/groundtruth.txt ../result/fr3_sitting_halfsphere.txt --plot ../result/fr3_sitting_halfsphere.pdf

# #===============================================rgbd_dataset_freiburg3_sitting_static===================================================
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM3.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_static "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_static/associate.txt ../result/fr3_sitting_static
# # python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_static/groundtruth.txt ../result/fr3_sitting_static.txt --plot ../result/fr3_sitting_static.pdf

# # evo_ape tum /home/robot/Datasets/TUM/rgbd_dataset_freiburg3_sitting_xyz/groundtruth.txt result/fr3_sitting_xyz.txt -a -s



# #=======================================================evaluation=============================================
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_static/groundtruth.txt ../result/fr3_sitting_static.txt --plot ../result/fr3_sitting_static.pdf
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_xyz/groundtruth.txt ../result/fr3_sitting_xyz.txt --plot ../result/fr3_sitting_xyz.pdf
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_halfsphere/groundtruth.txt ../result/fr3_sitting_halfsphere.txt --plot ../result/fr3_sitting_halfsphere.pdf
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg3_sitting_rpy/groundtruth.txt ../result/fr3_sitting_rpy.txt --plot ../result/fr3_sitting_rpy.pdf




# #===============================================rgbd_dataset_freiburg3_walking_xyz===================================================
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM3.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_xyz "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_xyz/associate.txt ../result/fr3_walking_xyz
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_xyz/groundtruth.txt ../result/fr3_walking_xyz.txt --plot ../result/fr3_walking_xyz.pdf

# # #===============================================rgbd_dataset_freiburg3_walking_rpy===================================================
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM3.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_rpy "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_rpy/associate.txt ../result/fr3_walking_rpy
# # python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_rpy/groundtruth.txt ../result/fr3_walking_rpy.txt --plot ../result/fr3_walking_rpy.pdf

# #===============================================rgbd_dataset_freiburg3_walking_halfsphere===================================================
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM3.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_halfsphere "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_halfsphere/associate.txt ../result/fr3_walking_halfsphere
# # python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_halfsphere/groundtruth.txt ../result/fr3_walking_halfsphere.txt --plot ../result/fr3_walking_halfsphere.pdf

# #===============================================rgbd_dataset_freiburg3_walking_static===================================================
# ./rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM3.yaml "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_static "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_static/associate.txt ../result/fr3_walking_static
# # python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_static/groundtruth.txt ../result/fr3_walking_static.txt --plot ../result/fr3_walking_static.pdf




# #=======================================================evaluation=============================================
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_halfsphere/groundtruth.txt ../result/fr3_walking_halfsphere.txt --plot ../result/fr3_walking_halfsphere.pdf
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_rpy/groundtruth.txt ../result/fr3_walking_rpy.txt --plot ../result/fr3_walking_rpy.pdf
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_static/groundtruth.txt ../result/fr3_walking_static.txt --plot ../result/fr3_walking_static.pdf
# python ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM"/rgbd_dataset_freiburg3_walking_xyz/groundtruth.txt ../result/fr3_walking_xyz.txt --plot ../result/fr3_walking_xyz.pdf


# evo_ape tum /home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_halfsphere/groundtruth.txt ../result/fr3_walking_halfsphere.txt -a -s
# evo_ape tum /home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_rpy/groundtruth.txt ../result/fr3_walking_rpy.txt -a -s
# evo_ape tum /home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_static/groundtruth.txt ../result/fr3_walking_static.txt -a -s
# evo_ape tum /home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/groundtruth.txt ../result/fr3_walking_xyz.txt -a 