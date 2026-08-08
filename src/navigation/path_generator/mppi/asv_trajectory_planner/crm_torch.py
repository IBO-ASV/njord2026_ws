import numpy as np
import torch

def _EntryCriteria(own, oth):
    alpha13crit = 20 # overtake
    alpha14crit = 15 # headon
    alpha15crit = 10 # crosssing

    turn = np.arctan2( (oth[0] - own[0]), (oth[1] - own[1]) )


    # if dcpa <= R_detect and -5 <= tcpa <= 30:
    if True:
        rel_owncog = (own[3]-np.degrees(turn))%360
        rel_othcog = (oth[3]-np.degrees(turn))%360

        alpha_360 = (-(rel_othcog-180))%360
        if alpha_360>180:
            alpha_180 = alpha_360-360
        else:
            alpha_180 = alpha_360

        beta_360 = (-rel_owncog)%360
        if beta_360>180:
            beta_180 = beta_360-360
        else:
            beta_180 = beta_360

        # print('alpha_360 : ', alpha_360, '\nbeta_360 : ', beta_360)

        if (beta_360>112.5) and (beta_360<247.5) and (abs(alpha_180)<alpha13crit):
            # rule13/17 stand-on
            return 0

        elif (alpha_360>112.5) and (alpha_360<247.5) and (abs(beta_180)<alpha13crit):
            # rule13/16 give-way
            return 1

        elif (abs(beta_180)<alpha14crit) and (abs(alpha_180)<alpha14crit):
            # rule14
            return 2

        elif (beta_360>0) and (beta_360<112.5) and (alpha_180>-112.50) and (alpha_180<alpha15crit):
            # rule15/16 vessel is crossing give-way
            return 3

        elif (alpha_360>0) and (alpha_360<112.5) and (beta_180>-112.50) and (beta_180<alpha15crit):
            # rule15/17 vessel is crossing stand-on
            return 4
        else:
            #Safe
            return 5
    else:
        return 5

_EntryCriteria_GIVE_WAY_No = [1, 2, 3]
_EntryCriteria_STAND_ON_No = [ 0, 4  ]
_EntryCriteria_SAFE_No     = [   5   ]

tcpa_threshold1 = 8
tcpa_threshold2 = 7
dcpa_threshold1 = 10
dcpa_threshold2 = 9
def CPAgain(own, oth):
    # [x,y,spd,hdg,loa]
    rel_posi = (
        np.array(oth[:2]) - np.array(own[:2])
    )@np.array(
        [
            [ np.cos(np.radians(own[3])), np.sin(np.radians(own[3])) ],
            [-np.sin(np.radians(own[3])), np.cos(np.radians(own[3])) ],
        ]
    )

    relv = np.array(
        [
            oth[2]*np.sin(np.radians(oth[3]-own[3])) - 0,
            oth[2]*np.cos(np.radians(oth[3]-own[3])) - own[2],
        ]
    )
    tcpa = -(rel_posi@relv)/(relv@relv)
    dcpa = abs(rel_posi[0]*relv[1] - rel_posi[1]*relv[0])/np.sqrt(relv@relv)

    # print(tcpa, dcpa)
    # print(
    #     (tcpa-tcpa_threshold1)/(tcpa_threshold2-tcpa_threshold1),
    #     (dcpa-dcpa_threshold1)/(dcpa_threshold2-dcpa_threshold1),
    # )
    return np.where(
        tcpa>=0,
        np.clip(
            (tcpa-tcpa_threshold1)/(tcpa_threshold2-tcpa_threshold1), 0, 1
        ),
        np.clip(
            abs(tcpa)/tcpa_threshold2, 0, 1
        )
    )*np.clip(
        (dcpa-dcpa_threshold1)/(dcpa_threshold2-dcpa_threshold1), 0, 1
    )

RIGHT_AX_GAIN = 3.2
LEFT_AX_GAIN  = 1.6
FORE_AX_GAIN  = 6.4
AFT_AX_GAIN   = 1.6
def timedomaincrm(X, Y, T, own, others, turn=None, ax_gains=None):
    # ax_gains: optional (right, left, fore, aft) bumper axis gains in LOA
    # multiples. None keeps the historical module-level constants, so existing
    # callers are behaviorally unchanged.
    if ax_gains is None:
        right_ax_gain = RIGHT_AX_GAIN
        left_ax_gain = LEFT_AX_GAIN
        fore_ax_gain = FORE_AX_GAIN
        aft_ax_gain = AFT_AX_GAIN
    else:
        right_ax_gain, left_ax_gain, fore_ax_gain, aft_ax_gain = ax_gains

    relx = X - own[0]
    rely = Y - own[1]
    # dt = np.sqrt( relx**2 + rely**2 )/own[2]
    dt=T
    if turn is None:
        turn = torch.where(
            dt==0, torch.deg2rad(torch.ones_like(relx)*own[3]),
            torch.arctan2(relx,rely)
        )

    crms = torch.zeros_like(X)
    for oth in others:

        Vtx = oth[2]*np.sin(np.deg2rad(oth[3]))
        Vty = oth[2]*np.cos(np.deg2rad(oth[3]))

        relvx = Vtx
        relvy = Vty-own[2]

        EntryNo = _EntryCriteria(own, oth)

        pred_x = oth[0] + Vtx*dt
        pred_y = oth[1] + Vty*dt

        pred_relx = (pred_x-X)*torch.cos(turn) - (pred_y-Y)*torch.sin(turn)
        pred_rely = (pred_x-X)*torch.sin(turn) + (pred_y-Y)*torch.cos(turn)

        _s = torch.clip(
            1 - torch.sqrt(
                torch.where(
                    pred_relx>0,
                    (pred_relx/own[4]/right_ax_gain),
                    (pred_relx/own[4]/left_ax_gain),
                )**2 + torch.where(
                    pred_rely>0, (pred_rely/own[4]/fore_ax_gain),
                    (pred_rely/own[4]/aft_ax_gain),
                )**2
            ), 0, 1,
        )


        # CRM(相手船のバンパーへ侵入する領域)
        if (EntryNo in  _EntryCriteria_GIVE_WAY_No):
            pred_x_oth = X - pred_x
            pred_y_oth = Y - pred_y

            pred_relx_oth = pred_x_oth*np.cos(np.deg2rad(oth[3]))\
                - pred_y_oth*np.sin(np.deg2rad(oth[3]))
            pred_rely_oth = pred_x_oth*np.sin(np.deg2rad(oth[3]))\
                + pred_y_oth*np.cos(np.deg2rad(oth[3]))

            _s = 1 - (
                1-_s
            )*(
                1 - torch.clip(
                    1 - torch.sqrt(
                        torch.where(
                            pred_relx_oth>0,
                            (pred_relx_oth/oth[4]/right_ax_gain),
                            (pred_relx_oth/oth[4]/left_ax_gain),
                        )**2 + torch.where(
                            pred_rely_oth>0,
                            (pred_rely_oth/oth[4]/fore_ax_gain),
                            (pred_rely_oth/oth[4]/aft_ax_gain),
                        )**2
                    ), 0, 1,
                )
            )

        gain = CPAgain(own, oth)
        # print(gain)
        # crms = 1 - (1-crms)*(1-_s*gain)
        crms = 1 - (1-crms)*(1-_s)

    return crms

def timedomaincrm_numpy(X, Y, T, own, others,):
    relx = X - own[0]
    rely = Y - own[1]
    # dt = np.sqrt( relx**2 + rely**2 )/own[2]
    dt=T
    turn = np.where(
        dt==0, np.deg2rad(np.ones_like(relx)*own[3]),
        np.arctan2(relx,rely)
    )

    crms = np.zeros_like(X)
    for oth in others:

        Vtx = oth[2]*np.sin(np.deg2rad(oth[3]))
        Vty = oth[2]*np.cos(np.deg2rad(oth[3]))

        relvx = Vtx
        relvy = Vty-own[2]

        EntryNo = _EntryCriteria(own, oth)

        pred_x = oth[0] + Vtx*dt
        pred_y = oth[1] + Vty*dt

        pred_relx = (pred_x-X)*np.cos(turn) - (pred_y-Y)*np.sin(turn)
        pred_rely = (pred_x-X)*np.sin(turn) + (pred_y-Y)*np.cos(turn)

        _s = np.clip(
            1 - np.sqrt(
                np.where(
                    pred_relx>0,
                    (pred_relx/own[4]/RIGHT_AX_GAIN),
                    (pred_relx/own[4]/LEFT_AX_GAIN),
                )**2 + np.where(
                    pred_rely>0, (pred_rely/own[4]/FORE_AX_GAIN),
                    (pred_rely/own[4]/AFT_AX_GAIN),
                )**2
            ), 0, 1,
        )


        # CRM(相手船のバンパーへ侵入する領域)
        if (EntryNo in  _EntryCriteria_GIVE_WAY_No):
            pred_x_oth = X - pred_x
            pred_y_oth = Y - pred_y

            pred_relx_oth = pred_x_oth*np.cos(np.deg2rad(oth[3]))\
                - pred_y_oth*np.sin(np.deg2rad(oth[3]))
            pred_rely_oth = pred_x_oth*np.sin(np.deg2rad(oth[3]))\
                + pred_y_oth*np.cos(np.deg2rad(oth[3]))

            _s = 1 - (
                1-_s
            )*(
                1 - np.clip(
                    1 - np.sqrt(
                        np.where(
                            pred_relx_oth>0,
                            (pred_relx_oth/oth[4]/RIGHT_AX_GAIN),
                            (pred_relx_oth/oth[4]/LEFT_AX_GAIN),
                        )**2 + np.where(
                            pred_rely_oth>0,
                            (pred_rely_oth/oth[4]/FORE_AX_GAIN),
                            (pred_rely_oth/oth[4]/AFT_AX_GAIN),
                        )**2
                    ), 0, 1,
                )
            )

        crms = 1 - (1-crms)*(1-_s)

    return crms