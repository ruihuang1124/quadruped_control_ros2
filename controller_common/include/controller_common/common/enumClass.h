//
// Created by tlab-uav on 24-9-6.
//

#ifndef ENUMCLASS_H
#define ENUMCLASS_H

enum class FSMStateName
{
    // EXIT,
    INVALID,
    PASSIVE,
    PASSIVEADJUSTABLELEG,
    FIXEDDOWN,
    FIXEDDOWNADJUSTABLELEG,
    FIXEDSTANDADJUSTABLELEG,
    FIXEDSTAND,
    FREESTAND,
    TROTTING,

    SWINGTEST,
    BALANCETEST,

    OCS2,
    RL,
    RLPOLICY2,
    QMFIXEDDOWN,
    QMFIXEDSTAND,
    QMPASSIVE,
    QMRL,

    QWFIXEDDOWN,
    QWFIXEDSTAND,
    QWPASSIVE,
    QWRL
};

enum class FSMMode
{
    NORMAL,
    CHANGE
};

enum class FrameType
{
    BODY,
    HIP,
    GLOBAL
};

enum class WaveStatus
{
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL
};

#endif //ENUMCLASS_H
