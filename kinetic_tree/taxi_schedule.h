#ifndef SHARABLENETWORKEXP_TAXI_SCHEDULE_H
#define SHARABLENETWORKEXP_TAXI_SCHEDULE_H

#include <vector>
#include "../global.h"
#include "../metric.h"
#include "../util.h"

using namespace std;
typedef int ReqID;
typedef int WorkerID;
struct Request;
class MyKineticTree;

class KineticTreeNode {
public:
    // Structure Properties
    KineticTreeNode *parent;
    vector<KineticTreeNode *> children;
    int stub_node = 0; // 0-> not stub node; 1-> stub node; -1: root node;
    bool is_pick_up = false;
    ReqID request_id; // only work for pickup or drop-off node
    WorkerID worker_id; // only work for root
    int passengers; // passenger count in current node

    double time; // time from parent to here
    double slack_time; // allowed detour time

    int best_child; // mark the best path


    KineticTreeNode();

    KineticTreeNode(const KineticTreeNode &node);

    KineticTreeNode(ReqID r_id, double slack_time, bool is_pick_up, int passenger,
                    KineticTreeNode *parent);

    ~KineticTreeNode();

    bool insertNodes(vector<KineticTreeNode *> nodes, double depth);

    double bestTime();

    void updateSlackTime(double depth);

    NodeID getVertexId() const;

    bool valueDriver();

    void bestPath(vector<int> &path);

private:
    bool copyNodes(const vector<KineticTreeNode *> &nodes, double delay);

    KineticTreeNode *constructNode(KineticTreeNode *node, double detour);

    KineticTreeNode *constructNodeForCopyNodes(KineticTreeNode *node, double delay);

    bool valueDriver(double increased_slack);
};

class MyKineticTree {
public:
    KineticTreeNode *root;
    KineticTreeNode *root_tmp; // For value-push-cancel

    MyKineticTree(); // create an empty Kinetic Tree with a stub node as root
    MyKineticTree(const MyKineticTree &tree);

    ~MyKineticTree();

    double value(int r_id);

    bool push();

    void cancel();

    void assignDriver(int w_id); // Please ensure that you have called valueDriver before setting.

    pair<bool, int> step(int &drop_id) const;

    double valueDriver(int w_id, bool save);

    vector<int> bestPath();
};


typedef int ReqID;
typedef int WorkerID;

class TaxiSchedule {
public:
    queue<NodeID> cur_path; // 当前行驶路径 (路网顶点队列)
    set<int> requests; // 当前所维护的请求集合

//    NodeID cur_pos; // 当前所处路网位置
//    double cur_offset; // cur_pos偏移出的距离
    MyKineticTree *tree; // 行程中所维护的KineticTree

    TaxiSchedule();

    TaxiSchedule(const TaxiSchedule &schedule);

    ~TaxiSchedule();

    double value(int r_id); // 尝试插入请求，并返回插入后最优行程的长度，负值表示无法插入
    void cancel() const; // 取消插入请求
    void push(); // 物化上一次的value()结果
    void push_check(int &SWITCH_CNT); // 物化上一次的value()结果并check是否发生switch

    double valueDriver(int w_id); // 尝试将给定工人设置为根节点（当且仅当当前行程未分配司机时有效）
    // 撤销上一次valueDriver的结果（当且仅当当前行程未分配司机时有效）
    bool pushDriver(WorkerID w_id);

    void update(int time); // 根据给定时间更新行程(path queue && cur_pos && cur_offset && tree && requests)

    vector<int> bestPath(); // 获取最佳路径
private:
    ReqID tmp_req;
};

#endif //SHARABLENETWORKEXP_TAXI_SCHEDULE_H
