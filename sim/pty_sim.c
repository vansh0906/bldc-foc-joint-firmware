/* Run N simulated joints in real time behind a pseudo-terminal, so host software (jointctl,
 * the ROS 2 bridge) talks to them exactly as it would to a USB-RS485 adapter.
 *
 *   ./build/pty_sim --joints 2 --link /tmp/jointbus      (prints the pty path, Ctrl-C to stop)
 *
 * Each joint is calibrated at start-up (fast-forwarded) unless --no-cal is given. */
#include <errno.h>
#include <fcntl.h>
#ifdef __APPLE__
#include <util.h>
#else
#include <pty.h>
#endif
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "sim.h"

static volatile sig_atomic_t stop;
static void on_sig(int s) { (void)s; stop = 1; }

int main(int argc, char **argv)
{
    int nj = 1, cal = 1;
    unsigned seed = 1;
    const char *link = NULL;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--joints") && i + 1 < argc) nj = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--seed") && i + 1 < argc) seed = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--link") && i + 1 < argc) link = argv[++i];
        else if (!strcmp(argv[i], "--no-cal")) cal = 0;
        else { fprintf(stderr, "usage: %s [--joints N] [--seed S] [--link PATH] [--no-cal]\n", argv[0]); return 2; }
    }
    sim_t *s = sim_new(nj, seed);
    if (!s) { fprintf(stderr, "1..%d joints\n", SIM_MAX_J); return 2; }
    sim_run(s, 0.05);
    if (cal) {
        for (int j = 0; j < nj; j++) {
            sim_local(s, j, "cal");
            sim_run(s, 6.0);
            sim_local(s, j, "save");
        }
        fprintf(stderr, "calibrated %d joint(s)\n", nj);
    }
    int master, slave;
    char name[256];
    struct termios tio;
    memset(&tio, 0, sizeof tio);
    cfmakeraw(&tio);
    if (openpty(&master, &slave, name, &tio, NULL) < 0) { perror("openpty"); return 1; }
    fcntl(master, F_SETFL, O_NONBLOCK);
    if (link) {
        unlink(link);
        if (symlink(name, link) < 0) perror("symlink");
    }
    printf("%s\n", link ? link : name);
    fflush(stdout);
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    uint8_t buf[4096];
    double rx_t[4096];
    while (!stop) {
        ssize_t n = read(master, buf, sizeof buf);
        if (n > 0) sim_master_send(s, buf, (int)n, 0.0);
        sim_run(s, 0.001);
        int m = sim_master_recv(s, buf, rx_t, (int)sizeof buf);
        if (m > 0 && write(master, buf, (size_t)m) < 0 && errno != EAGAIN) break;
        next.tv_nsec += 1000000;
        if (next.tv_nsec >= 1000000000) { next.tv_nsec -= 1000000000; next.tv_sec++; }
        struct timespec now, d;                  /* sleep until the next 1 ms tick (Linux + macOS) */
        clock_gettime(CLOCK_MONOTONIC, &now);
        long ns = (next.tv_sec - now.tv_sec) * 1000000000L + (next.tv_nsec - now.tv_nsec);
        if (ns > 0) { d.tv_sec = ns / 1000000000L; d.tv_nsec = ns % 1000000000L; nanosleep(&d, NULL); }
    }
    if (link) unlink(link);
    close(slave);
    close(master);
    sim_free(s);
    return 0;
}
