#pragma once
/// @file LegMotion.h
/// @brief  Calculate leg motion
/// @date 2020.10.1
/// @author Shunya Hara

#include <Arduino.h>
#include <Vector3.h>

class LegMotion{
private:
    /* data */
    
    double min_rps;
    unsigned long pretime;
    double nowrad;
public:
    double getRad(double rps);

    /// @brief constractor
    /// @param min_rps [rps] モーションを生成する最小のrpsこの速度以下だと原点を返す
    LegMotion(double min_rps_);

    
    /// @brief 足の座標を計算する (円)
    /// @param rps [rps] 歩行モーションの角速度
    /// @param radius 生成する歩行モーションの半径
    /// @param direction 歩行モーションで進む方向[deg] 正面に進む場合 0[deg] 反時計回り正z
    /// @param phase [deg] 足の位相
    /// @return 計算結果の座標
    Vector3 getLegPositon(double rps,double radius,double direction,double phase=0);

    /// @brief 足の座標を計算する (半円)
    /// @param rps [rps] 歩行モーションの角速度
    /// @param radius 生成する歩行モーションの半径
    /// @param groundtime 地面に足が触れている時間の割合
    /// @param direction 歩行モーションで進む方向[deg] 正面に進む場合 0[deg] 反時計回り正z
    /// @param phase 0~360[deg] 足の位相
    /// @return 計算結果の座標
    Vector3 getLegPositon2(double rps,double radius,double groundtime,double direction,double phase=0.0);
};