/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ORB_SLAM3_OPTIMIZABLETYPES_H
#define ORB_SLAM3_OPTIMIZABLETYPES_H

#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include <Thirdparty/g2o/g2o/types/types_six_dof_expmap.h>
#include <Thirdparty/g2o/g2o/types/sim3.h>

#include <Eigen/Geometry>
#include <include/CameraModels/GeometricCamera.h>
typedef Eigen::Matrix<double,6,1> vector6d;


namespace ORB_SLAM3
{
// 左目纯位姿优化的边，左目点的重投影误差相对于左目位姿
class EdgeSE3ProjectXYZOnlyPose : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPose() {}

    bool read(std::istream &is);

    bool write(std::ostream &os) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs - pCamera->project(v1->estimate().map(Xw));
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        return (v1->estimate().map(Xw))(2) > 0.0;
    }

    virtual void linearizeOplus();

    Eigen::Vector3d Xw;
    GeometricCamera *pCamera;
};

// 两个相机中的右目上的重投影误差与左目位姿的边
class EdgeSE3ProjectXYZOnlyPoseToBody : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPoseToBody() {}

    bool read(std::istream &is);

    bool write(std::ostream &os) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs - pCamera->project((mTrl * v1->estimate()).map(Xw));
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        return ((mTrl * v1->estimate()).map(Xw))(2) > 0.0;
    }

    virtual void linearizeOplus();

    Eigen::Vector3d Xw;
    GeometricCamera *pCamera;

    g2o::SE3Quat mTrl;//两个相机时左相机到右相机的位姿变换
};


// 左目纯位姿优化的边，左目点的重投影误差相对于左目位姿以及三维点
class EdgeSE3ProjectXYZ : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZ();

    bool read(std::istream &is);

    bool write(std::ostream &os) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs - pCamera->project(v1->estimate().map(v2->estimate()));
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        return ((v1->estimate().map(v2->estimate()))(2) > 0.0);
    }

    virtual void linearizeOplus();

    GeometricCamera *pCamera;
};

// 两个相机中的右目上的重投影误差与左目位姿以及三维点的边
class EdgeSE3ProjectXYZToBody : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZToBody();

    bool read(std::istream &is);

    bool write(std::ostream &os) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs - pCamera->project((mTrl * v1->estimate()).map(v2->estimate()));
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        return ((mTrl * v1->estimate()).map(v2->estimate()))(2) > 0.0;
    }

    virtual void linearizeOplus();

    GeometricCamera *pCamera;
    g2o::SE3Quat mTrl;
};

// sim3节点
class VertexSim3Expmap : public g2o::BaseVertex<7, g2o::Sim3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexSim3Expmap();
    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

    // 原始值
    virtual void setToOriginImpl()
    {
        _estimate = g2o::Sim3();
    }

    // 更新
    virtual void oplusImpl(const double *update_)
    {
        Eigen::Map<g2o::Vector7d> update(const_cast<double *>(update_));

        if (_fix_scale)
            update[6] = 0;

        g2o::Sim3 s(update);
        setEstimate(s * estimate());
    }

    GeometricCamera *pCamera1, *pCamera2;

    bool _fix_scale;
};

// sim3边
class EdgeSim3ProjectXYZ : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, ORB_SLAM3::VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3ProjectXYZ();
    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

    void computeError()
    {
        const ORB_SLAM3::VertexSim3Expmap *v1 = static_cast<const ORB_SLAM3::VertexSim3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);

        Eigen::Vector2d obs(_measurement);
        _error = obs - v1->pCamera1->project(v1->estimate().map(v2->estimate()));
    }

    // 自动求导，没错g2o也有自动求导
    // virtual void linearizeOplus();
};

// sim3反投的边
class EdgeInverseSim3ProjectXYZ : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeInverseSim3ProjectXYZ();
    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;

    void computeError()
    {
        const ORB_SLAM3::VertexSim3Expmap *v1 = static_cast<const ORB_SLAM3::VertexSim3Expmap *>(_vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);

        Eigen::Vector2d obs(_measurement);
        _error = obs - v1->pCamera2->project((v1->estimate().inverse().map(v2->estimate())));
    }

    // 自动求导，没错g2o也有自动求导
    // virtual void linearizeOplus();
};



//直线重投影误差边
class  EdgeLineSE3ProjectXYZOnlyPose: public  g2o::BaseUnaryEdge<2, Eigen::Vector3d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeLineSE3ProjectXYZOnlyPose(){}

    bool read(std::istream& is){
        for (int i=0; i<3; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool write(std::ostream& os)const {
        for (int i=0; i<3; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }


    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector3d obs(_measurement);
        Eigen::Matrix<double,2,3> Matrix23d; 
        Matrix23d(0,0) = cam_project(v1->estimate().map(Xw_s))(0); 
        Matrix23d(0,1) = cam_project(v1->estimate().map(Xw_s))(1);
        Matrix23d(0,2) = 1.0;
        Matrix23d(1,0) = cam_project(v1->estimate().map(Xw_e))(0); 
        Matrix23d(1,1) = cam_project(v1->estimate().map(Xw_e))(1);
        Matrix23d(1,2) = 1.0;
        _error = Matrix23d * obs;
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        return (v1->estimate().map(Xw_s))(2)>0.0 && (v1->estimate().map(Xw_e))(2)>0.0;
    }


    virtual void linearizeOplus(){
        g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans_s = vi->estimate().map(Xw_s);
        Eigen::Vector3d xyz_trans_e = vi->estimate().map(Xw_e);

        double x_s = xyz_trans_s[0];
        double y_s = xyz_trans_s[1];
        double invz_s = 1.0/xyz_trans_s[2];
        double invz_s_2 = invz_s*invz_s;

        double x_e = xyz_trans_e[0];
        double y_e = xyz_trans_e[1];
        double invz_e = 1.0/xyz_trans_e[2];
        double invz_e_2 = invz_e*invz_e;

        double l0 = obs_temp(0);
        double l1 = obs_temp(1);

        _jacobianOplusXi(0,0) = -fx*x_s*y_s*invz_s_2*l0-fy*(1+y_s*y_s*invz_s_2)*l1;
        _jacobianOplusXi(0,1) = fx*(1+x_s*x_s*invz_s_2)*l0+fy*x_s*y_s*invz_s_2*l1;
        _jacobianOplusXi(0,2) = -fx*y_s*invz_s*l0+fy*x_s*invz_s*l1; 
        _jacobianOplusXi(0,3) = fx*invz_s*l0;
        _jacobianOplusXi(0,4) = fy*invz_s*l1;
        _jacobianOplusXi(0,5) = (-fx*x_s*l0-fy*y_s*l1)*invz_s_2;

        _jacobianOplusXi(1,0) = -fx*x_e*y_e*invz_e_2*l0-fy*(1+y_e*y_e*invz_e_2)*l1;
        _jacobianOplusXi(1,1) = fx*(1+x_e*x_e*invz_e_2)*l0+fy*x_e*y_e*invz_e_2*l1;
        _jacobianOplusXi(1,2) = -fx*y_e*invz_e*l0+fy*x_e*invz_e*l1;  
        _jacobianOplusXi(1,3) = fx*invz_e*l0;
        _jacobianOplusXi(1,4) = fy*invz_e*l1;
        _jacobianOplusXi(1,5) = (-fx*x_e*l0-fy*y_e*l1)*invz_e_2;
    }

    Eigen::Vector2d cam_project(const Eigen::Vector3d & trans_xyz) const
    {
        const float invz = 1.0f/trans_xyz[2];
        Eigen::Vector2d res;
        res[0] = trans_xyz[0]*invz*fx + cx;
        res[1] = trans_xyz[1]*invz*fy + cy;
        return res;
    }

    Eigen::Vector3d Xw_s;
    Eigen::Vector3d Xw_e;
    Eigen::Vector3d obs_temp;
    double fx, fy, cx, cy;
};


class  EdgeLineSE3ProjectXYZ: public  g2o::BaseBinaryEdge<2, Eigen::Vector3d, g2o::VertexSBALineXYZ, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeLineSE3ProjectXYZ();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBALineXYZ* v2 = static_cast<const g2o::VertexSBALineXYZ*>(_vertices[0]);
        Eigen::Vector3d obs(_measurement);
        Eigen::Matrix<double,2,3> Matrix23d; 
        Matrix23d(0,0) = cam_project(v1->estimate().map(v2->estimate().head(3)))(0); 
        Matrix23d(0,1) = cam_project(v1->estimate().map(v2->estimate().head(3)))(1);
        Matrix23d(0,2) = 1.0;
        Matrix23d(1,0) = cam_project(v1->estimate().map(v2->estimate().tail(3)))(0); 
        Matrix23d(1,1) = cam_project(v1->estimate().map(v2->estimate().tail(3)))(1);
        Matrix23d(1,2) = 1.0;
        _error = Matrix23d * obs;


    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBALineXYZ* v2 = static_cast<const g2o::VertexSBALineXYZ*>(_vertices[0]);
        return (v1->estimate().map(v2->estimate().head(3)))(2)>0.0 && (v1->estimate().map(v2->estimate().tail(3)))(2)>0.0;
    }


    virtual void linearizeOplus();

    Eigen::Vector2d cam_project(const Eigen::Vector3d & trans_xyz) const;

    Eigen::Vector3d obs_temp;
    double fx, fy, cx, cy;
};



}

#endif // ORB_SLAM3_OPTIMIZABLETYPES_H
