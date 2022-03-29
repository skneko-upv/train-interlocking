#define TRACK_SEGMENT_COUNT 2
#define SIGNAL_COUNT        2

#include "common.pml"

/*
 *         A :]
 *   <001> ===*=================||==================*=== <001>
 *                   TC01               TC02        [: AR
 */
#define TC01        0
#define TC02        1
#define A           0
#define AR          1
mtype:route_spec = { R_A_001, R_AR_001 };

#define T_001   (!segment_locked[TC01] && !segment_locked[TC02])

active proctype controller() {
    bool segment_locked[TRACK_SEGMENT_COUNT];
    controller_init();

    pid train;
    byte segment;

end_ctl_loop:
    do
    :: tr_ctl ??[ train, REQUEST_ROUTE, R_A_001 ] && T_001 ->
        segment_locked[TC01] = true;
        segment_locked[TC02] = true;
        commit_route(R_A_001);
        change_signal(A, GREEN)
    :: tr_ctl ??[ train, REQUEST_ROUTE, R_AR_001 ] && T_001 ->
        segment_locked[TC01] = true;
        segment_locked[TC02] = true;
        commit_route(R_AR_001);
        change_signal(AR, GREEN)
    :: seg_ctl ? segment, TRAIN_LEFT -> 
        if
        :: segment == TC01 -> change_signal(A, RED)
        :: segment == TC02 -> change_signal(AR, RED)
        fi;
        segment_locked[segment] = false
    od
}

active[3] proctype train() {
    byte segment = OUTSIDE;

    do
    :: true ->
        mtype:route_spec route; 
        byte initial_segment;
        if
        :: route = R_A_001; initial_segment = TC01
        :: route = R_AR_001; initial_segment = TC02
        fi

        train_init(initial_segment);

        segment = initial_segment;
        if
        :: route == R_A_001 ->
            signal[A] == GREEN;
            do
            :: segment == TC01 -> 
                atomic {
                    segment = TC02;
                    move_segments(TC01, TC02)
                }
            :: segment == TC02 ->
                atomic {
                    segment = OUTSIDE;
                    release_segment(TC02)
                }
            :: segment == OUTSIDE -> break
            od
        :: route == R_AR_001 ->
            signal[AR] == GREEN;
            do
            :: segment == TC02 -> 
                atomic {
                    segment = TC01;
                    move_segments(TC02, TC01)
                }
            :: segment == TC01 ->
                atomic {
                    segment = OUTSIDE;
                    release_segment(TC01)
                }
            :: segment == OUTSIDE -> break
            od
        fi
    od
}

/*** Verification ***/

/* Liveness */

ltl all_trains_leave { []( (train[2]:segment != OUTSIDE) -> <>( train[2]:segment == OUTSIDE ) ) }

#define obeys_signal(t, r, seg, sig)     (train[t]:segment == seg && train[t]:route == r -> ((train[t]:segment == seg) U (signal[sig] == GREEN)))
ltl trains_obey_signals { []( obeys_signal(2, R_A_001, TC01, A) && obeys_signal(2, R_AR_001, TC02, AR) ) }

/* Safety */

#define collision_between(x, y)     (train[x]:segment != OUTSIDE && train[y]:segment != OUTSIDE && train[x]:segment == train[y]:segment)
ltl no_collision { []!( collision_between(1, 2) || collision_between(1, 3) || collision_between(2, 3) ) }

/* 
 * Deadlock freedom:
 *  spin -a $THIS_FILE
 *  gcc -DMEMLIM=1024 -O2 -DXUSAFE -DSAFETY -DNOCLAIM -w -o pan pan.c
 *  ./pan -m100000
 *
 * Claims verification:
 *  spin -a $THIS_FILE
 *  gcc -DMEMLIM=1024 -O2 -DXUSAFE -w -o pan pan.c
 *  ./pan -m100000 -a -N all_trains_leave
 *  ./pan -m100000 -a -N trains_obey_signals
 *  ./pan -m100000 -a -N no_collision
 */