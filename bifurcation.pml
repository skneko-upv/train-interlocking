#define TRACK_SEGMENT_COUNT 3
#define SIGNAL_COUNT        3
#define POINTS_COUNT        1

#include "common.pml"

/*
 *         A :]      TC01   TP01        TC02        [: AR
 *   <001> ===*==============+======================*=== <001>
 *                            \
 *                             \
 *                              ====================*=== <002>
 *                                       TC03       [: B
 */
#define TC01        0
#define TC02        1
#define TC03        2
#define TP01        0
#define A           0
#define AR          1
#define B           2
mtype:route_spec = { R_A_001, R_AR_001, R_A_002, R_B_001 };

#define T_001       (!segment_locked[TC01] && !segment_locked[TC02])
#define T_001_002   (!segment_locked[TC01] && !segment_locked[TC03])

active proctype controller() {
    bool segment_locked[TRACK_SEGMENT_COUNT];
    controller_init();

    pid train;
    byte segment;

end_ctl_loop:
    do
    :: tr_ctl ??[ train, REQUEST_ROUTE, R_A_001 ] && T_001 ->
        turn_points(TP01, LEFT);
        segment_locked[TC01] = true;
        segment_locked[TC02] = true;
        commit_route(R_A_001);
        change_signal(A, GREEN)
    :: tr_ctl ??[ train, REQUEST_ROUTE, R_AR_001 ] && T_001 ->
        turn_points(TP01, LEFT);
        segment_locked[TC01] = true;
        segment_locked[TC02] = true;
        commit_route(R_AR_001);
        change_signal(AR, GREEN)
    :: tr_ctl ??[ train, REQUEST_ROUTE, R_A_002 ] && T_001_002 ->
        turn_points(TP01, RIGHT);
        segment_locked[TC01] = true;
        segment_locked[TC03] = true;
        commit_route(R_A_002);
        change_signal(A, GREEN)
    :: tr_ctl ??[ train, REQUEST_ROUTE, R_B_001 ] && T_001_002 ->
        turn_points(TP01, RIGHT);
        segment_locked[TC01] = true;
        segment_locked[TC03] = true;
        commit_route(R_B_001);
        change_signal(B, GREEN)
    :: seg_ctl ? segment, TRAIN_LEFT -> 
        if
        :: segment == TC01 -> change_signal(A, RED)
        :: segment == TC02 -> change_signal(AR, RED)
        :: segment == TC03 -> change_signal(B, RED)
        fi;
        segment_locked[segment] = false
    od
}

active[2] proctype train() {
    byte segment = OUTSIDE;

    do
    :: true ->
        mtype:route_spec route; 
        byte initial_segment;
        if
        :: route = R_A_001; initial_segment = TC01
        :: route = R_AR_001; initial_segment = TC02
        :: route = R_A_002; initial_segment = TC01
        :: route = R_B_001; initial_segment = TC03
        fi

        train_init(initial_segment);

        segment = initial_segment;
        if
        :: route == R_A_001 || route == R_A_002 ->
            signal[A] == GREEN;
            do
            :: segment == TC01 && points[TP01] == LEFT -> 
                atomic {
                    segment = TC02;
                    move_segments(TC01, TC02)
                }
            :: segment == TC01 && points[TP01] == RIGHT -> 
                atomic {
                    segment = TC03;
                    move_segments(TC01, TC03)
                }
            :: segment == TC02 ->
                atomic {
                    segment = OUTSIDE;
                    release_segment(TC02)
                }
            :: segment == TC03 ->
                atomic {
                    segment = OUTSIDE;
                    release_segment(TC03)
                }
            :: segment == OUTSIDE -> break
            od
        :: route == R_AR_001 ->
            signal[AR] == GREEN;
            do
            :: segment == TC02 && points[TP01] == LEFT -> 
                atomic {
                    segment = TC01;
                    move_segments(TC02, TC01)
                }
            :: segment == TC02 && points[TP01] == RIGHT ->
                printf("Train %d DERAILED!", _pid);
                break
            :: segment == TC01 ->
                atomic {
                    segment = OUTSIDE;
                    release_segment(TC01)
                }
            :: segment == OUTSIDE -> break
            od
        :: route == R_B_001 ->
            signal[B] == GREEN;
            do
            :: segment == TC03 && points[TP01] == RIGHT ->
                atomic {
                    segment = TC01;
                    move_segments(TC03, TC01)
                }
            :: segment == TC03 && points[TP01] == LEFT ->
                printf("Train %d DERAILED!", _pid);
                break
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
ltl trains_obey_signals { []( 
    obeys_signal(2, R_A_001,  TC01, A) && 
    obeys_signal(2, R_AR_001, TC02, AR) && 
    obeys_signal(2, R_A_002,  TC01, A) && 
    obeys_signal(2, R_B_001,  TC03, B) 
) }

#define points_dir(t, r, from, to, p, dir)  (train[t]:route == r && train[t]:segment == from -> (points[p] == dir U train[t]:segment == to))
ltl no_unexpected_points_direction { [](
    points_dir(2, R_A_001,  TC01, TC02, TP01, LEFT) &&
    points_dir(2, R_A_002,  TC01, TC03, TP01, RIGHT) &&
    points_dir(2, R_AR_001, TC02, TC01, TP01, LEFT) &&
    points_dir(2, R_B_001,  TC03, TC01, TP01, RIGHT)
) }

/* Safety */

#define collision_between(x, y)  (train[x]:segment != OUTSIDE && train[y]:segment != OUTSIDE && train[x]:segment == train[y]:segment)
ltl no_collision { []!( collision_between(1, 2) ) }

/* 
 * Deadlock freedom:
 *  spin -a $THIS_FILE
 *  gcc -DMEMLIM=10240 -O2 -DXUSAFE -DSAFETY -DNOCLAIM -DCOLLAPSE -w -o pan pan.c
 *  ./pan -m1000000
 *
 * Claims verification:
 *  spin -a $THIS_FILE
 *  gcc -DMEMLIM=10240 -O2 -DXUSAFE -DCOLLAPSE -w -o pan pan.c
 *  ./pan -m10000000 -a -N trains_obey_signals
 *  ./pan -m10000000 -a -N no_unexpected_points_direction
 *  ./pan -m10000000 -a -N no_collision
 *  gcc -DMEMLIM=1024 -O2 -DNFAIR=4 -DXUSAFE -DCOLLAPSE -DNOREDUCE -w -o pan pan.c
 *  ./pan -m1000000 -a -f -N all_trains_leave
 */