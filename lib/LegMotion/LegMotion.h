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
    double getRad(double rps);
    double min_rps;
public:
    /// @brief constractor
    /// @param min_rps [rps] モーションを生成する最小のrpsこの速度以下だと原点を返す
    LegMotion(double min_rps_);

    
    /// @brief 足の座標を計算する
    /// @param rps [rps] 歩行モーションの角速度
    /// @param radius 生成する歩行モーションの半径
    /// @param direction 歩行モーションで進む方向[deg] 正面に進む場合 0[deg] 反時計回り正z
    /// @param phase [deg] 足の位相
    /// @return 計算結果の座標
    Vector3 getLegPositon(double rps,double radius,double direction,double phase=0);
};