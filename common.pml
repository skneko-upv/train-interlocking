/*
 * Type definitions 
 */

mtype:rel_direction2 = { LEFT, RIGHT };

mtype:track_segment_sensor_state = { IDLE, PRESSED, RELEASE };

mtype:signal_aspect = { GREEN, RED };

mtype:route_spec = { NULL_ROUTE };

#define OUTSIDE 255

/*
 * Channels and messages
 */

mtype:tr_ctl_msg = { 
    REQUEST_ROUTE
};
mtype:ctl_tr_msg = { 
    COMMIT_ROUTE
};
chan tr_ctl = [1] of { pid, mtype:tr_ctl_msg, mtype:route_spec };
chan ctl_tr = [0] of { pid, mtype:ctl_tr_msg, mtype:route_spec };

mtype:seg_ctl_msg = {
    READY,
    TRAIN_LEFT
};
chan seg_ctl = [0] of { byte, mtype:seg_ctl_msg };

#ifdef SIGNAL_COUNT
chan ctl_sig = [0] of { byte, mtype:signal_aspect };

mtype:sig_ctl_msg = {
    CHANGED
};
chan sig_ctl = [0] of { byte, mtype:sig_ctl_msg };
#endif

#ifdef POINTS_COUNT
chan ctl_pt = [0] of { byte, mtype:rel_direction2 };

mtype:pt_ctl_msg = {
    SECURED
};
chan pt_ctl = [0] of { byte, mtype:pt_ctl_msg };
#endif

/*
 * Shared state
 */

mtype:track_segment_sensor_state train_on_segment[TRACK_SEGMENT_COUNT];

#ifdef SIGNAL_COUNT
mtype:signal_aspect signal[SIGNAL_COUNT];
#endif

#ifdef POINTS_COUNT
mtype:rel_direction2 points[POINTS_COUNT];
#endif

/*
 * Inline functions
 */

inline controller_init() {
    byte i;

#ifdef SIGNAL_COUNT
    for (i : 0 .. (SIGNAL_COUNT - 1)) {
        run signal_ctl(i);
    };
#endif
#ifdef POINTS_COUNT
    for (i : 0 .. (POINTS_COUNT - 1)) {
        run points_ctl(i);
    };
#endif

    for (i : 0 .. (TRACK_SEGMENT_COUNT - 1)) {
        run track_segment_sensor(i);
    }
#ifdef SIGNAL_COUNT
    for (i : 0 .. (SIGNAL_COUNT - 1)) {
        change_signal(i, RED)
    };
#endif
    for (i : 0 .. (TRACK_SEGMENT_COUNT - 1)) {
        segment_locked[i] = false;
        seg_ctl ? eval(i), READY
    };
}

inline train_init(initial_segment) {
    tr_ctl ! _pid, REQUEST_ROUTE, route;
    ctl_tr ?? eval(_pid), COMMIT_ROUTE, route;
    train_on_segment[initial_segment] = PRESSED;
}

inline commit_route(route) {
    tr_ctl ?? train, REQUEST_ROUTE, route;
    ctl_tr ! train, COMMIT_ROUTE, route
}

inline acquire_segment(i) {
    train_on_segment[i] = PRESSED
}

inline release_segment(i) {
    train_on_segment[i] = RELEASE
}

inline move_segments(from, to) {
    atomic {
        acquire_segment(to);
        release_segment(from)
    }
}

inline turn_points(id, direction) {
    ctl_pt ! id, direction;
    pt_ctl ?? id, SECURED
}

inline change_signal(id, aspect) {
    ctl_sig ! id, aspect;
    sig_ctl ?? eval(id), CHANGED
}

/*
 * Process definitions
 */

proctype track_segment_sensor(byte i) {
    train_on_segment[i] = IDLE;

    seg_ctl ! i, READY;

end_seg_loop:
    do
    :: train_on_segment[i] == RELEASE ->
        train_on_segment[i] = IDLE;
        seg_ctl ! i, TRAIN_LEFT
    od
}

#ifdef SIGNAL_COUNT
proctype signal_ctl(byte i) {
    mtype:signal_aspect aspect;

end_sig_loop:
    do
    :: ctl_sig ?? eval(i), aspect ->
        signal[i] = aspect;
        sig_ctl ! i, CHANGED
    od
}
#endif

#ifdef POINTS_COUNT
proctype points_ctl(byte i) {
    mtype:rel_direction2 direction;

end_pt_loop:
    do
    :: ctl_pt ?? eval(i), direction ->
        points[i] = direction;
        pt_ctl ! i, SECURED
    od
}
#endif