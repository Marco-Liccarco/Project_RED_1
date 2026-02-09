void Control::GoToLimitSwitchHomingHW(std::vector<unsigned int>& nodeVec)
{
    const long SPEED_FAST = 2000;
    const long SPEED_SLOW = SPEED_FAST / 10;
    const long ACCELERATION = 5000;

    // imposto dei soft stop per decidere la direzione più vicina al limit switch
    const long MECH_MIN_POS = -100000;
    const long MECH_MAX_POS = +100000;

    for (unsigned int i = 0; i < nodeVec.size(); i++)
    {
        unsigned int id = nodeVec[i];
        long currentPos = 0;

        VCS_GetPositionIs(nodeMap[id].handle, id, &currentPos, errorCode);

        // calcolo della distanza dai finecorsa fisici
        long distToNeg = std::abs(currentPos - MECH_MIN_POS);
        long distToPos = std::abs(MECH_MAX_POS - currentPos);

        string homingMethod;

        if (distToNeg < distToPos)
            homingMethod = HM_NEGATIVE_LIMIT_SWITCH;
        else
            homingMethod = HM_POSITIVE_LIMIT_SWITCH;

        if (!VCS_SetOperationMode(nodeMap[id].handle, id, 6, errorCode))
        {
            cerr << "Node " << id << " failed to set Homing Mode" << endl;
        }

        long homeOffset = 0;

        if (!VCS_SetHomingParameter(
                nodeMap[id].handle,
                id,
                homingMethod,
                homeOffset,
                SPEED_FAST,   // ricerca veloce
                SPEED_SLOW,   // avvicinamento lento (1/10 della velocità massima consentita)
                ACCELERATION,
                errorCode))
        {
            cerr << "Node " << id << " failed to set homing parameters" << endl;
        }

        if (!VCS_ActivateHomingMode(nodeMap[id].handle, id, errorCode))
        {
            cerr << "Node " << id << " failed to activate Homing Mode" << endl;
        }

        if (!VCS_StartHoming(nodeMap[id].handle, id, errorCode))
        {
            cerr << "Node " << id << " failed to start homing" << endl;
        }

        bool homingAttained = false;
        do
        {
            VCS_GetHomingState(nodeMap[id].handle, id, &homingAttained, errorCode);
        }
        while (!homingAttained);

        cerr << "Node " << id << " homing completed on hardware limit switch" << endl;
    }
}
