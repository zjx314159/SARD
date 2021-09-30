#include "taxi_schedule.h"

/**
 * Working for inserting request pairs.
 * @param depth total distance from top to current node
 * @param nodes (pickup node, drop-off node)
 */
bool KineticTreeNode::insertNodes(vector<KineticTreeNode *> nodes, double depth) {
    if (nodes.empty()) return true;
    double detour_time = depth + sssp->shortestDistance(getVertexId(), nodes[0]->getVertexId());
    KineticTreeNode *copy_node = constructNode(nodes[0], detour_time);
    if (!copy_node) return false;
    bool fail = false;
    for (auto &child:children) {
        double delay = detour_time + sssp->shortestDistance(child->getVertexId(), copy_node->getVertexId())
                       - (depth + child->time);
        if (copy_node->copyNodes({child}, delay)) continue;
        fail = true;
    }

    if ((!fail || !copy_node->children.empty()) && nodes.size() >= 2) {
//        copy_node->updateSlackTime(detour_time);
        vector<KineticTreeNode *> nodes_copy{nodes[1]};
        if (!copy_node->insertNodes(nodes_copy, detour_time))
            fail = true;
    }

    // Call for other edges recursively in Kinetic Tree.
    for (int it = 0; it < children.size(); it++) {
        KineticTreeNode *child = children[it];
        if (!(child->insertNodes(nodes, depth + child->time))) {
            delete child;
            children.erase(children.begin() + it);
            it--;
        }
    }
    if (!fail) children.push_back(copy_node);
    else {
        delete copy_node;
        return !children.empty();
    }
    return true;
}

/**
 * Creating a stub node
 */
KineticTreeNode::KineticTreeNode() {
    time = 0;
    parent = nullptr;
    slack_time = 100000;
    best_child = 0;
    request_id = -1;
    worker_id = -1;
    stub_node = 1;
    passengers = 0;
}

KineticTreeNode::KineticTreeNode(const KineticTreeNode &node) {
    stub_node = node.stub_node;
    request_id = node.request_id;
    slack_time = node.slack_time;
    best_child = node.best_child;
    passengers = node.passengers;
    time = node.time;
    is_pick_up = node.is_pick_up;
    worker_id = node.worker_id;
    parent = node.parent;
    for (auto child: node.children) {
        auto *child_copy = new KineticTreeNode(*child);
        child_copy->parent = this;
        children.push_back(child_copy);
    }
}

/**
 * Creating a node
 */
KineticTreeNode::KineticTreeNode(ReqID r_id, double slack_time, bool is_pick_up, int passenger,
                                 KineticTreeNode *parent) {
    this->slack_time = slack_time;
    this->passengers = passenger;
    this->parent = parent;
    this->request_id = r_id;
    this->worker_id = -1;
    this->best_child = -1;
    this->is_pick_up = is_pick_up;
    this->time = parent ? sssp->shortestDistance(parent->getVertexId(), getVertexId()) : 0;
}

KineticTreeNode::~KineticTreeNode() {
    // for (auto child:children) delete child;
    for (int i = 0; i < children.size(); ++i) {
        delete children[i];
    }
}

/**
 * Find the best schedule and calculate total driving time
 * @return total driving time
 */
double KineticTreeNode::bestTime() {
    if (children.empty()) return 0; // leaf node, no travel time
    double best_time = numeric_limits<double>::max();
    for (int i = 0; i < children.size(); i++) {
        double child_best_time = children[i]->bestTime() + children[i]->time;
        if (child_best_time > best_time) continue;
        best_time = child_best_time;
        best_child = i;
    }
    return best_time;
}

/**
 * Updating the slack time of this node and children recursively
 */
void KineticTreeNode::updateSlackTime(double depth) {
    if (stub_node != 0) {
        // if current node is a stub node or root, feel free to set it with a random value.
        slack_time = 1000000;
        return;
    }
    // initial current nodes slack_time with maximum tolerable value.
    slack_time = (is_pick_up ? R[request_id].ddl - R[request_id].len : R[request_id].ddl) - CURRENT_TIME - depth;
    if (children.empty()) return;
    // current node's slack_time constrained by children's maximum tolerable time.
    double max_child_slack = numeric_limits<double>::min();
    for (auto &child:children) {
        child->updateSlackTime(depth + child->time);
        max_child_slack = max(max_child_slack, child->slack_time);
    }
    slack_time = min(slack_time, max_child_slack);
}

NodeID KineticTreeNode::getVertexId() const {
    if (stub_node == 1) return -1;
    return stub_node == -1 ? W[worker_id].pid : (is_pick_up ? R[request_id].s : R[request_id].e);
}

/**
 * Copies `nodes` to be underneath this node
 * Note that this function only return false when no children exist.
 */
bool KineticTreeNode::copyNodes(const vector<KineticTreeNode *> &nodes, double detour) {
    bool fail = false;
    for (auto &node:nodes) {
        auto copy = constructNodeForCopyNodes(node, detour);
        if (copy && copy->copyNodes(node->children, detour)) {
            children.push_back(copy);
            continue;
        }
        delete copy;
        fail = true;
    }
    return !fail || !children.empty();
}

KineticTreeNode *KineticTreeNode::constructNodeForCopyNodes(KineticTreeNode *node, double delay) {
    int passenger = passengers + (node->is_pick_up ? 1 : -1) * R[node->request_id].com;
    if (delay > node->slack_time || passenger > CAPACITY_CONSTRAINT) return nullptr;
    return new KineticTreeNode(node->request_id, node->slack_time - delay,
                               node->is_pick_up, passenger, this);
}

KineticTreeNode *KineticTreeNode::constructNode(KineticTreeNode *node, double detour) {
    double slack = (node->is_pick_up ? R[node->request_id].ddl - R[node->request_id].len : R[node->request_id].ddl) -
                   CURRENT_TIME - detour;
    int passenger = passengers + (node->is_pick_up ? 1 : -1) * R[node->request_id].com;
    if (slack < 0 || passenger > CAPACITY_CONSTRAINT) return nullptr; // can't arrive `node` on time or can not pickup
    return new KineticTreeNode(node->request_id, slack, node->is_pick_up, passenger, this);
}

bool KineticTreeNode::valueDriver() {
    bool fail = false;
    for (int it = 0; it < children.size(); it++) {
        auto &child = children[it];
        child->time = sssp->shortestDistance(getVertexId(), child->getVertexId());
        if (child->valueDriver(child->time))continue;
        fail = true;
        delete child;
        children.erase(children.begin() + it);
        it--;
    }
    return !fail || !children.empty();
}

bool KineticTreeNode::valueDriver(double increased_slack) {
    if (slack_time < increased_slack) return false;
    slack_time -= increased_slack;
    bool fail = false;
    for (int it = 0; it < children.size(); it++) {
        auto &child = children[it];
        if (child->valueDriver(increased_slack))continue;
        fail = true;
        delete child;
        children.erase(children.begin() + it);
        it--;
    }
    return !fail || !children.empty();
}

void KineticTreeNode::bestPath(vector<int> &path) {
    path.push_back(request_id);
    if (children.empty()) return;
    children[best_child]->bestPath(path);
}

MyKineticTree::MyKineticTree() {
    this->root = new KineticTreeNode();
    this->root_tmp = nullptr;
}


MyKineticTree::MyKineticTree(const MyKineticTree &tree) {
    root = new KineticTreeNode(*tree.root);
    if (tree.root_tmp != nullptr)
        root_tmp = new KineticTreeNode(*tree.root_tmp);
    else root_tmp = nullptr;
}

MyKineticTree::~MyKineticTree() {
    delete root;
    if (root_tmp != nullptr) delete root_tmp;
    root_tmp = nullptr;
}

double MyKineticTree::value(int r_id) {
    // Operation done on root_tmp
    if (root_tmp != nullptr) {
//        cout << "Size:" << root_tmp->children.size() << endl;
        delete root_tmp;
        root_tmp = nullptr;
    }
    root_tmp = new KineticTreeNode(*root);
    double buffer_time = R[r_id].ddl - sssp->shortestDistance(R[r_id].s, R[r_id].e) - CURRENT_TIME;
    auto *pick = new KineticTreeNode(r_id, buffer_time, true, 1, nullptr);
    auto *drop = new KineticTreeNode(r_id, buffer_time, false, 0, nullptr);
    bool copy_success = root_tmp->insertNodes({pick, drop}, 0);
    delete pick;
    delete drop;
    if (!copy_success) return -1;
    root_tmp->updateSlackTime(0);
    double time = root_tmp->bestTime();
    return time;
}

bool MyKineticTree::push() {
    if (root_tmp == nullptr)return false;
    delete root;
    root = root_tmp;
    root_tmp = nullptr;
    return true;
}

void MyKineticTree::cancel() {
    if (root_tmp == nullptr)return;
    delete root_tmp;
    root_tmp = nullptr;
}

double MyKineticTree::valueDriver(int w_id, bool save) {
    auto *tmp = new KineticTreeNode(*root);
    tmp->stub_node = -1;
    tmp->worker_id = w_id;
    double best_time = tmp->valueDriver() ? tmp->bestTime() : -1;
    if (save) {
        if (root_tmp != nullptr) delete root_tmp;
        root_tmp = tmp;
    } else delete tmp;
    return best_time;
}

void MyKineticTree::assignDriver(int w_id) {
    if (root_tmp != nullptr) {
        delete root_tmp;
        root_tmp = nullptr;
    }
    valueDriver(w_id, true);
    delete root;
    root = root_tmp;
    root_tmp = nullptr;
}

/**
 * linearStep down along with best path and remove other subtrees
 * @return is_drop_off and cur_passengers_on_car
 */
pair<bool, int> MyKineticTree::step(int &drop_id) const {
    if (root->children.empty()) return {false, 0};
    int best_child = root->best_child;
    //delete every child except the best option from the root node
    for (int i = root->children.size() - 1; i >= 0; i--) {
        if (i != best_child) {
            delete root->children[i];
        }
    }
    KineticTreeNode *removed_node = root->children[best_child];
    vector<KineticTreeNode *>().swap(root->children);
    double len = sssp->shortestDistance(W[root->worker_id].pid, removed_node->getVertexId());
    for (auto &child : removed_node->children) {
        root->children.push_back(child);
        child->parent = root;
        child->time = len;
    }
    bool drop_passenger = !removed_node->is_pick_up;
    int passenger_cnt = removed_node->passengers;
    drop_id = drop_passenger ? removed_node->request_id : -1;
    removed_node->children.clear();
    delete removed_node;
    root->updateSlackTime(0);
    root->bestTime();
    return {drop_passenger, passenger_cnt};
}

vector<int> MyKineticTree::bestPath() {
    vector<int> res;
    root->bestPath(res);
    return res;
}

void TaxiSchedule::push() {
    if (!tree->push()) return;
    this->requests.insert(tmp_req);
}

bool checkSwitched(const vector<int> &a, const vector<int> &b, const set<int> &inserted) {
    for (uint i = 0, j = 0; i < a.size() && j < b.size(); i++, j++) {
        while (inserted.find(a[i]) != inserted.end()) i++;
        while (inserted.find(b[j]) != inserted.end())j++;
        if (i < a.size() && j < b.size() && a[i] != b[j]) return true;
    }
    return false;
}

void TaxiSchedule::push_check(int &SWITCH_CNT) {
    auto originPath = bestPath();
    vector<int> updatedPath;
    tree->root_tmp->bestPath(updatedPath);
    if (checkSwitched(originPath, updatedPath, {tmp_req}))
        SWITCH_CNT++;
    if (!tree->push()) return;
    this->requests.insert(tmp_req);
}

void TaxiSchedule::cancel() const {
    tree->cancel();
}

double TaxiSchedule::value(int r_id) {
    this->tmp_req = r_id;
    return tree->value(r_id);
}

double TaxiSchedule::valueDriver(int w_id) {
    if (W[w_id].path == nullptr) return tree->valueDriver(w_id, false);
    auto *schedule_tmp = new TaxiSchedule(*W[w_id].path);
    for (int r_id:requests) {
        if (schedule_tmp->value(r_id) < 0) {
            delete schedule_tmp;
            return -1;
        }
        schedule_tmp->push();
    }
    double best_time = schedule_tmp->tree->root->bestTime();
    delete schedule_tmp;
    return best_time;
}

/**
 * push requests into driver `w_id`
 * @param w_id worker/driver id
 */
bool TaxiSchedule::pushDriver(WorkerID w_id) {
//    if (W[w_id].path == nullptr) {
//        tree->assignDriver(w_id);
//        W[w_id].path = new TaxiSchedule();
//        delete W[w_id].path->tree;
//        W[w_id].path->tree = new MyKineticTree(*tree);
//        W[w_id].path->requests = requests;
//    } else {
//        bool fail = false;
//        auto *schedule_tmp = new TaxiSchedule(*W[w_id].path);
//        for (int r_id:requests) {
//            if (schedule_tmp->value(r_id) < 0) {
//                fail = true;
//                break;
//            }
//            schedule_tmp->push();
//        }
//        if (!fail) {
//            auto originPath = W[w_id].path->bestPath();
//            auto updatedPath = schedule_tmp->bestPath();
//            set<int> inserted;
//            set_difference(schedule_tmp->requests.begin(), schedule_tmp->requests.end(),
//                           W[w_id].path->requests.begin(), W[w_id].path->requests.end(),
//                           inserter(inserted, inserted.begin()));
//            delete W[w_id].path;
//            W[w_id].path = schedule_tmp;
//        } else {
//            delete schedule_tmp;
//            return false;
//        }
//    }

    tree->assignDriver(w_id);
    // Update path for schedule
//    vector<NodeID> path;
//    NodeID next_node = tree->root->children[tree->root->best_child]->getVertexId();
//    sssp->shortestPathByNode(W[w_id].pid, next_node, path);
//    queue<int> empty;
//    swap(empty, cur_path);
//    for (auto node:path) cur_path.push(node);
    return true;
}

/**
 * @return real drive distance
 */
void TaxiSchedule::update(int time) {
    updateGrid(W[tree->root->worker_id].pid, tree->root->worker_id, -1);
    WorkerID w_id = tree->root->worker_id;
    bool finished = false;
    double offset = time;
    while (!finished) {
        while (!cur_path.empty()) {
            double len = sssp->shortestDistance(W[w_id].pid, cur_path.front());
            // Note that driver maybe accelerated in this turn to avoid offset computation
            if (offset < 0) {
                finished = true;
                break;
            }
            W[w_id].pid = cur_path.front();
            cur_path.pop();
            offset -= len;
//            TOTAL_DRIVE_DISTANCE += len;
        }
        if (cur_path.empty()) {
            int drop_id = -1;
            pair<bool, int> res = tree->step(drop_id);
            W[w_id].num = res.second;
            if (res.first) requests.erase(drop_id);
            if (tree->root->children.empty()) break;
            vector<NodeID> path;
            sssp->shortestPathByNode(W[w_id].pid, tree->root->children[tree->root->best_child]->getVertexId(), path);
            for (NodeID i:path) cur_path.push(i);
        }
    }
    updateGrid(W[w_id].pid, tree->root->worker_id, 1);
}


TaxiSchedule::TaxiSchedule() {
    tree = new MyKineticTree();
    tmp_req = -1;
}

TaxiSchedule::TaxiSchedule(const TaxiSchedule &schedule) {
    cur_path = schedule.cur_path;
    requests.insert(schedule.requests.begin(), schedule.requests.end());
    tmp_req = schedule.tmp_req;
    tree = new MyKineticTree(*schedule.tree);
}

TaxiSchedule::~TaxiSchedule() {
    delete tree;
}

vector<int> TaxiSchedule::bestPath() {
    return tree->bestPath();
}