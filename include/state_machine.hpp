#pragma once

#include <algorithm>

#include "basic_controller.hpp"

enum class STATES
{
    DAMPING = 0,
    SIT = 1,
    STAND = 2,
    CTRL = 3
};


class SimpleStateMachine
{
public:
    SimpleStateMachine() : state(STATES::DAMPING) {}

    bool Stop()
    {
        state = STATES::DAMPING;
        return true;
    }

    bool Stand()
    {
        // // Gate invoking standing by already being in the DAMPING state
        // if (state == STATES::DAMPING)
        // {
        //     state = STATES::STAND;
        //     standing_count = 0;
        //     return true;
        // }
        // else // if state is not DAMPING
        // {
        //     return false;
        // }

        state = STATES::STAND;
        standing_count = 0;
        return true;
    }

    bool Ctrl() // gate entering ctrl mode by already being in the standing state
    {
        // if (state == STATES::STAND)
        // {
        //     state = STATES::CTRL;
        //     return true;
        // }
        // else
        // {
        //     return false;
        // }
        state = STATES::CTRL;
        return true;
    }

    void Standing(BasicUserController& ctrl)
    {
        // Create intermediate stnding goals over "standing_duration" iterations
        standing_percent = (float)standing_count / standing_duration;
        for (int i = 0; i < 12; i++)
        {
            ctrl.jpos_des.at(i) = (1 - standing_percent)*ctrl.start_pos.at(i) + standing_percent*ctrl.stand_pos.at(i);
        }
        standing_count++;
        standing_count = standing_count > standing_duration? standing_duration : standing_count;
    }

    STATES state;
protected:
    int standing_count = 0;
    float standing_percent = 0.0;
    int standing_duration = 100; // 200 intermediate standing timesteps

};


class PACTStateMachine: public SimpleStateMachine
{
public:
    bool Sit()
    {
        // Gate sitting by already being in the DAMPING state or standing state
        if(state == STATES::DAMPING || state == STATES::STAND )
        {
            state = STATES::SIT;
            sitting_count = 0;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool Stand()
    {
        // // Gate sitting by already being in the sitting state
        // if (state == STATES::SIT)
        // {
        //     state = STATES::STAND;
        //     standing_count = 0;
        //     return true;
        // }
        // else // if state is not SIT
        // {
        //     return false;
        // }
        state = STATES::STAND;
        standing_count = 0;
        return true;
    }

    void Sitting(BasicUserController& ctrl)
    {
        // Create intermediate sitting goals over "sitting_duration" iterations
        sitting_percent = (float)sitting_count / sitting_duration;
        for (int i = 0; i < 12; i++)
        {
            ctrl.jpos_des.at(i) = (1 - sitting_percent)*ctrl.start_pos.at(i) + sitting_percent*ctrl.sit_pos.at(i);
        }
        sitting_count++;
        sitting_count = sitting_count > sitting_duration? sitting_duration : sitting_count;
    }
protected:
    int sitting_count = 0;
    float sitting_percent = 0.0;
    int sitting_duration = 200;
};