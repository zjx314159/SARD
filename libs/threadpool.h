#pragma once
#ifndef SHARABLENETWORKEXP_THREADPOOL_H
#define SHARABLENETWORKEXP_THREADPOOL_H

#include <vector>
#include <queue>
#include <atomic>
#include <future>
#include <stdexcept>

namespace std {
#define  THREADPOOL_MAX_NUM 50

//#define  THREADPOOL_AUTO_GROW
    class threadpool {
        using Task = function<void()>;
        vector<thread> _pool;
        queue<Task> _tasks;
        mutex _lock;
        condition_variable _task_cv;
        atomic<bool> _run{true};
        atomic<int> _idlThrNum{0};

    public:
        inline threadpool(unsigned short size = 4) { addThread(size); }

        inline ~threadpool() {
            _run = false;
            _task_cv.notify_all();
            for (thread &thread : _pool) {
                //thread.detach();
                if (thread.joinable())
                    thread.join();
            }
        }

    public:
        template<class F, class... Args>
        auto commit(F &&f, Args &&... args) -> future<decltype(f(args...))> {
            if (!_run)    // stoped ??
                throw runtime_error("commit on ThreadPool is stopped.");

            using RetType = decltype(f(args...));
            auto task = make_shared<packaged_task<RetType()>>(
                    bind(forward<F>(f), forward<Args>(args)...)
            );
            future<RetType> future = task->get_future();
            {    // 添加任务到队列
                lock_guard<mutex> lock{_lock};
                _tasks.emplace([task]() {
                    (*task)();
                });
            }
#ifdef THREADPOOL_AUTO_GROW
            if (_idlThrNum < 1 && _pool.size() < THREADPOOL_MAX_NUM)
            addThread(1);
#endif
            _task_cv.notify_one();

            return future;
        }

        int idlCount() { return _idlThrNum; }

        int thrCount() { return _pool.size(); }

#ifndef THREADPOOL_AUTO_GROW
    private:
#endif

        void addThread(unsigned short size) {
            for (; _pool.size() < THREADPOOL_MAX_NUM && size > 0; --size) {
                _pool.emplace_back([this] {
                    while (_run) {
                        Task task;
                        {
                            unique_lock<mutex> lock{_lock};
                            _task_cv.wait(lock, [this] {
                                return !_run || !_tasks.empty();
                            });
                            if (!_run && _tasks.empty())
                                return;
                            task = move(_tasks.front());
                            _tasks.pop();
                        }
                        _idlThrNum--;
                        task();
                        _idlThrNum++;
                    }
                });
                _idlThrNum++;
            }
        }
    };

}
#endif
