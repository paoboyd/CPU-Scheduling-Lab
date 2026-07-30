#include "oslabs.h"

/* helper - checks if a PCB has no process running */
int is_null_pcb(struct PCB p) {
    return p.process_id == 0;
}

/* Priority-based Preemptive Scheduling */

struct PCB handle_process_arrival_pp(struct PCB ready_queue[QUEUEMAX], int *queue_cnt,
                                      struct PCB current_process, struct PCB new_process,
                                      int timestamp) {

    /* nothing running yet, so the new process just starts */
    if (is_null_pcb(current_process)) {
        new_process.execution_starttime = timestamp;
        new_process.execution_endtime = timestamp + new_process.total_bursttime;
        new_process.remaining_bursttime = new_process.total_bursttime;
        return new_process;
    }

    /* smaller priority number = higher priority */
    if (new_process.process_priority >= current_process.process_priority) {
        /* new process is not higher priority, so it just waits in line */
        new_process.execution_starttime = 0;
        new_process.execution_endtime = 0;
        new_process.remaining_bursttime = new_process.total_bursttime;

        ready_queue[*queue_cnt] = new_process;
        (*queue_cnt)++;

        return current_process;
    }

    /* new process wins, preempt whatever is running now */
    current_process.execution_endtime = 0;
    current_process.remaining_bursttime = current_process.remaining_bursttime -
                                           (timestamp - current_process.execution_starttime);

    ready_queue[*queue_cnt] = current_process;
    (*queue_cnt)++;

    new_process.execution_starttime = timestamp;
    new_process.execution_endtime = timestamp + new_process.total_bursttime;
    new_process.remaining_bursttime = new_process.total_bursttime;

    return new_process;
}

struct PCB handle_process_completion_pp(struct PCB ready_queue[QUEUEMAX], int *queue_cnt,
                                         int timestamp) {

    struct PCB next_process = NULLPCB;

    if (*queue_cnt == 0) {
        return next_process;
    }

    /* find the highest priority process in the queue (lowest priority number) */
    int best_index = 0;
    for (int i = 1; i < *queue_cnt; i++) {
        if (ready_queue[i].process_priority < ready_queue[best_index].process_priority) {
            best_index = i;
        }
    }

    next_process = ready_queue[best_index];

    /* remove it from the queue by shifting everything after it left */
    for (int i = best_index; i < *queue_cnt - 1; i++) {
        ready_queue[i] = ready_queue[i + 1];
    }
    (*queue_cnt)--;

    next_process.execution_starttime = timestamp;
    next_process.execution_endtime = timestamp + next_process.remaining_bursttime;

    return next_process;
}

/* Shortest-Remaining-Time-Next Preemptive Scheduling */

struct PCB handle_process_arrival_srtp(struct PCB ready_queue[QUEUEMAX], int *queue_cnt,
                                        struct PCB current_process, struct PCB new_process,
                                        int time_stamp) {

    if (is_null_pcb(current_process)) {
        new_process.execution_starttime = time_stamp;
        new_process.execution_endtime = time_stamp + new_process.total_bursttime;
        new_process.remaining_bursttime = new_process.total_bursttime;
        return new_process;
    }

    if (new_process.total_bursttime >= current_process.remaining_bursttime) {
        /* new process would take just as long or longer, so it waits */
        new_process.execution_starttime = 0;
        new_process.execution_endtime = 0;
        new_process.remaining_bursttime = new_process.total_bursttime;

        ready_queue[*queue_cnt] = new_process;
        (*queue_cnt)++;

        return current_process;
    }

    /* new process is shorter, it preempts the running process */
    current_process.remaining_bursttime = current_process.remaining_bursttime -
                                           (time_stamp - current_process.execution_starttime);
    current_process.execution_starttime = 0;
    current_process.execution_endtime = 0;

    ready_queue[*queue_cnt] = current_process;
    (*queue_cnt)++;

    new_process.execution_starttime = time_stamp;
    new_process.execution_endtime = time_stamp + new_process.total_bursttime;
    new_process.remaining_bursttime = new_process.total_bursttime;

    return new_process;
}

struct PCB handle_process_completion_srtp(struct PCB ready_queue[QUEUEMAX], int *queue_cnt,
                                           int timestamp) {

    struct PCB next_process = NULLPCB;

    if (*queue_cnt == 0) {
        return next_process;
    }

    /* find the process with the smallest remaining burst time */
    int best_index = 0;
    for (int i = 1; i < *queue_cnt; i++) {
        if (ready_queue[i].remaining_bursttime < ready_queue[best_index].remaining_bursttime) {
            best_index = i;
        }
    }

    next_process = ready_queue[best_index];

    for (int i = best_index; i < *queue_cnt - 1; i++) {
        ready_queue[i] = ready_queue[i + 1];
    }
    (*queue_cnt)--;

    next_process.execution_starttime = timestamp;
    next_process.execution_endtime = timestamp + next_process.remaining_bursttime;

    return next_process;
}

/* ---------------- Round-Robin Scheduling ---------------- */

struct PCB handle_process_arrival_rr(struct PCB ready_queue[QUEUEMAX], int *queue_cnt,
                                      struct PCB current_process, struct PCB new_process,
                                      int timestamp, int time_quantum) {

    if (is_null_pcb(current_process)) {
        new_process.execution_starttime = timestamp;
        new_process.remaining_bursttime = new_process.total_bursttime;

        int slice = time_quantum < new_process.total_bursttime ? time_quantum : new_process.total_bursttime;
        new_process.execution_endtime = timestamp + slice;

        return new_process;
    }

    /* RR never preempts on arrival - the new process just joins the line */
    new_process.execution_starttime = 0;
    new_process.execution_endtime = 0;
    new_process.remaining_bursttime = new_process.total_bursttime;

    ready_queue[*queue_cnt] = new_process;
    (*queue_cnt)++;

    return current_process;
}

struct PCB handle_process_completion_rr(struct PCB ready_queue[QUEUEMAX], int *queue_cnt,
                                         int time_stamp, int time_quantum) {

    struct PCB next_process = NULLPCB;

    if (*queue_cnt == 0) {
        return next_process;
    }

    /* pick the process that's been waiting the longest (earliest arrival) */
    int best_index = 0;
    for (int i = 1; i < *queue_cnt; i++) {
        if (ready_queue[i].arrival_timestamp < ready_queue[best_index].arrival_timestamp) {
            best_index = i;
        }
    }

    next_process = ready_queue[best_index];

    for (int i = best_index; i < *queue_cnt - 1; i++) {
        ready_queue[i] = ready_queue[i + 1];
    }
    (*queue_cnt)--;

    next_process.execution_starttime = time_stamp;

    int slice = time_quantum < next_process.remaining_bursttime ? time_quantum : next_process.remaining_bursttime;
    next_process.execution_endtime = time_stamp + slice;

    return next_process;
}
