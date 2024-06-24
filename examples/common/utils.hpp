#if defined(__linux__) || defined(__APPLE__)
#ifdef __linux__
#include <termio.h>
#else
#include <termios.h>
#endif
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#define gets_s gets

int getch(void) {
    struct termios tm, tm_old;
    int            fd = 0, ch;

    if(tcgetattr(fd, &tm) < 0) {  // Save the current terminal settings
        return -1;
    }

    tm_old = tm;
    cfmakeraw(&tm);                        // Change the terminal settings to raw mode, in which all input data is processed in bytes
    if(tcsetattr(fd, TCSANOW, &tm) < 0) {  // Settings after changes on settings
        return -1;
    }

    ch = getchar();
    if(tcsetattr(fd, TCSANOW, &tm_old) < 0) {  // Change the settings to what they were originally
        return -1;
    }

    return ch;
}

int kbhit(void) {
    struct termios oldt, newt;
    int            ch;
    int            oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if(ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

#include <sys/time.h>
long long get_current_timestamp_ms() {
    struct timeval te;
    gettimeofday(&te, NULL);                                          // 获取当前时间
    long long milliseconds = te.tv_sec * 1000LL + te.tv_usec / 1000;  // 计算毫秒
    return milliseconds;
}
#else
#include <conio.h>
#include <windows.h>
long long get_current_timestamp_ms() {
    FILETIME      ft;
    LARGE_INTEGER li;
    GetSystemTimeAsFileTime(&ft);
    li.LowPart             = ft.dwLowDateTime;
    li.HighPart            = ft.dwHighDateTime;
    long long milliseconds = li.QuadPart / 10000LL;
    return milliseconds;
}
#endif

#define ESC 27

// helper function to check for errors and exit if there is one.
#define CHECK_OB_ERROR_EXIT(error)                                \
    if(*error) {                                                  \
        const char *error_message = ob_error_get_message(*error); \
        fprintf(stderr, "Error: %s\n", error_message);            \
        ob_delete_error(*error);                                  \
        *error = NULL;                                            \
        exit(-1);                                                 \
    }                                                             \
    *error = NULL;
