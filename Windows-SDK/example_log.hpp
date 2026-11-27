// Run log for the examples: `-l` copies everything printed to stdout and
// stderr, by the example and by the SDK, into a log file in the current
// directory, while still showing it on the console.
//
// stdout/stderr are redirected at the file-descriptor level into a pipe, and a
// thread writes what comes out of the pipe to both the console and the file,
// so output from SDK threads is captured too. The log stops when the
// ExampleLog object goes out of scope.
//
// This file is identical in Linux-SDK, MAC-SDK and Windows-SDK.

#ifndef INSIGHT9_EXAMPLE_LOG_HPP
#define INSIGHT9_EXAMPLE_LOG_HPP

#include <errno.h>
#include <stdio.h>
#include <time.h>

#include <string>
#include <thread>

#if defined(_WIN32)
#include <fcntl.h>
#include <io.h>
#define EXLOG_DUP   _dup
#define EXLOG_DUP2  _dup2
#define EXLOG_CLOSE _close
#define EXLOG_READ  _read
#define EXLOG_WRITE _write
#else
#include <unistd.h>
#define EXLOG_DUP   dup
#define EXLOG_DUP2  dup2
#define EXLOG_CLOSE close
#define EXLOG_READ  read
#define EXLOG_WRITE write
#endif

class ExampleLog {
public:
    ExampleLog() = default;
    ExampleLog(const ExampleLog &) = delete;
    ExampleLog &operator=(const ExampleLog &) = delete;
    ~ExampleLog() { stop(); }

    // Start logging to "<prefix>_YYYYMMDD_HHMMSS.log" in the current directory.
    bool start(const char *prefix) {
        if (file_) return true;

        char stamp[32];
        time_t now = time(NULL);
        struct tm tm_now;
#if defined(_WIN32)
        localtime_s(&tm_now, &now);
#else
        localtime_r(&now, &tm_now);
#endif
        strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm_now);
        path_ = std::string(prefix) + "_" + stamp + ".log";

        file_ = fopen(path_.c_str(), "wb");
        if (!file_) {
            fprintf(stderr, "Cannot create log file %s\n", path_.c_str());
            return false;
        }
        char header[64];
        strftime(header, sizeof(header), "==== log started %Y-%m-%d %H:%M:%S ====\n", &tm_now);
        fputs(header, file_);
        fflush(file_);

        int fds[2];
#if defined(_WIN32)
        if (_pipe(fds, 64 * 1024, _O_BINARY) != 0) {
#else
        if (pipe(fds) != 0) {
#endif
            fprintf(stderr, "Cannot create the log pipe\n");
            fclose(file_);
            file_ = NULL;
            return false;
        }

        fflush(stdout);
        fflush(stderr);
        console_ = EXLOG_DUP(1);        // the real console, for the copy thread
        saved_err_ = EXLOG_DUP(2);
        EXLOG_DUP2(fds[1], 1);
        EXLOG_DUP2(fds[1], 2);
        EXLOG_CLOSE(fds[1]);
        read_fd_ = fds[0];
        // A pipe makes stdout fully buffered; flush every write instead so
        // the console and the log stay live and in order with stderr.
        setvbuf(stdout, NULL, _IONBF, 0);

        thread_ = std::thread([this] { copy_loop(); });
        printf("Saving the run log to %s\n", path_.c_str());
        return true;
    }

    // Restore stdout/stderr and close the log. Safe to call more than once.
    void stop() {
        if (!file_) return;
        fflush(stdout);
        fflush(stderr);
        // Pointing 1 and 2 back at the console closes the last write end of
        // the pipe, so the copy thread sees end-of-file and exits.
        EXLOG_DUP2(console_, 1);
        EXLOG_DUP2(saved_err_, 2);
        if (thread_.joinable()) thread_.join();
        EXLOG_CLOSE(read_fd_);
        EXLOG_CLOSE(console_);
        EXLOG_CLOSE(saved_err_);
        fclose(file_);
        file_ = NULL;
        printf("Run log saved to %s\n", path_.c_str());
    }

private:
    void copy_loop() {
        char buf[4096];
        for (;;) {
            int n = (int)EXLOG_READ(read_fd_, buf, sizeof(buf));
            if (n < 0 && errno == EINTR) continue;   // e.g. Ctrl+C arrived
            if (n <= 0) break;
            int off = 0;
            while (off < n) {   // the console may take a write in pieces
                int w = (int)EXLOG_WRITE(console_, buf + off, (unsigned)(n - off));
                if (w <= 0) break;
                off += w;
            }
            fwrite(buf, 1, (size_t)n, file_);
            fflush(file_);
        }
    }

    FILE *file_ = NULL;
    std::string path_;
    int read_fd_ = -1;
    int console_ = -1;
    int saved_err_ = -1;
    std::thread thread_;
};

#endif  // INSIGHT9_EXAMPLE_LOG_HPP
