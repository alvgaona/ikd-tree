#pragma once
#include <cmath>
#include <memory>
#include <utility>
#include <vector>
#include <pcl/point_types.h>

namespace ikd_tree {

struct ikdTree_PointType {
    float x, y, z;
    ikdTree_PointType(float px = 0.0f, float py = 0.0f, float pz = 0.0f) {
        x = px;
        y = py;
        z = pz;
    }
};

struct BoxPointType {
    float vertex_min[3];
    float vertex_max[3];
};

enum operation_set { ADD_POINT, DELETE_POINT, DELETE_BOX, ADD_BOX, DOWNSAMPLE_DELETE, PUSH_DOWN };

enum delete_point_storage_set { NOT_RECORD, DELETE_POINTS_REC, MULTI_THREAD_REC };

// Fixed-capacity ring buffer used by the rebuild logger. The default
// matches upstream (hku-mars/ikd-Tree) so this fork is a drop-in
// replacement; pass `rebuild_log_capacity` to KdTree() to shrink it for
// memory-constrained deployments. Overflow silently overwrites the oldest
// entry.
template <typename T> class MANUAL_Q {
  public:
    static constexpr int kDefaultCapacity = 1000000;

    explicit MANUAL_Q(int capacity = kDefaultCapacity) : cap_(capacity), q(new T[capacity]) {}
    ~MANUAL_Q() { delete[] q; }
    MANUAL_Q(const MANUAL_Q &) = delete;
    MANUAL_Q &operator=(const MANUAL_Q &) = delete;

    void clear() {
        head = 0;
        tail = 0;
        counter = 0;
        is_empty = true;
    }
    void pop() {
        if (counter == 0)
            return;
        head = (head + 1) % cap_;
        if (--counter == 0)
            is_empty = true;
    }
    void push(T op) {
        q[tail] = std::move(op);
        ++counter;
        is_empty = false;
        tail = (tail + 1) % cap_;
    }
    T front() { return q[head]; }
    T back() { return q[tail]; }
    bool empty() { return is_empty; }
    int size() { return counter; }
    int capacity() const { return cap_; }

  private:
    int cap_;
    int head = 0, tail = 0, counter = 0;
    T *q;
    bool is_empty = true;
};

template <typename PointType> class KdTree {
  public:
    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    using Ptr = std::shared_ptr<KdTree<PointType>>;

  private:
    // Forward-declared private types; full definitions live in ikd_tree.cpp so
    // pthread.h is not part of the public header surface.
    struct KD_TREE_NODE;
    struct Threads;

    struct Operation_Logger_Type {
        PointType point;
        BoxPointType boxpoint;
        bool tree_deleted, tree_downsample_deleted;
        operation_set op;
    };

    struct PointType_CMP {
        PointType point;
        float dist = 0.0;
        PointType_CMP(PointType p = PointType(), float d = INFINITY) {
            this->point = p;
            this->dist = d;
        };
        bool operator<(const PointType_CMP &a) const {
            if (fabs(dist - a.dist) < 1e-10)
                return point.x < a.point.x;
            else
                return dist < a.dist;
        }
    };

    class MANUAL_HEAP {
      public:
        MANUAL_HEAP(int max_capacity = 100) {
            cap = max_capacity;
            heap = new PointType_CMP[max_capacity];
            heap_size = 0;
        }

        ~MANUAL_HEAP() { delete[] heap; }

        void pop() {
            if (heap_size == 0)
                return;
            heap[0] = heap[heap_size - 1];
            heap_size--;
            MoveDown(0);
            return;
        }

        PointType_CMP top() { return heap[0]; }

        void push(PointType_CMP point) {
            if (heap_size >= cap)
                return;
            heap[heap_size] = point;
            FloatUp(heap_size);
            heap_size++;
            return;
        }

        int size() { return heap_size; }

        void clear() { heap_size = 0; }

      private:
        int heap_size = 0;
        int cap = 0;
        PointType_CMP *heap;
        void MoveDown(int heap_index) {
            int l = heap_index * 2 + 1;
            PointType_CMP tmp = heap[heap_index];
            while (l < heap_size) {
                if (l + 1 < heap_size && heap[l] < heap[l + 1])
                    l++;
                if (tmp < heap[l]) {
                    heap[heap_index] = heap[l];
                    heap_index = l;
                    l = heap_index * 2 + 1;
                } else
                    break;
            }
            heap[heap_index] = tmp;
            return;
        }

        void FloatUp(int heap_index) {
            int ancestor = (heap_index - 1) / 2;
            PointType_CMP tmp = heap[heap_index];
            while (heap_index > 0) {
                if (heap[ancestor] < tmp) {
                    heap[heap_index] = heap[ancestor];
                    heap_index = ancestor;
                    ancestor = (heap_index - 1) / 2;
                } else
                    break;
            }
            heap[heap_index] = tmp;
            return;
        }
    };

    // Multi-thread Tree Rebuild
    bool termination_flag = false;
    bool rebuild_flag = false;
    std::unique_ptr<Threads> threads_;
    MANUAL_Q<Operation_Logger_Type> Rebuild_Logger;
    PointVector Rebuild_PCL_Storage;
    KD_TREE_NODE **Rebuild_Ptr = nullptr;
    int search_mutex_counter = 0;
    static void *multi_thread_ptr(void *arg);
    void multi_thread_rebuild();
    void start_thread();
    void stop_thread();
    void run_operation(KD_TREE_NODE **root, const Operation_Logger_Type &operation);
    // KD Tree Functions and augmented variables
    int Treesize_tmp = 0, Validnum_tmp = 0;
    float alpha_bal_tmp = 0.5, alpha_del_tmp = 0.0;
    float delete_criterion_param = 0.5f;
    float balance_criterion_param = 0.7f;
    float downsample_size = 0.2f;
    bool Delete_Storage_Disabled = false;
    KD_TREE_NODE *STATIC_ROOT_NODE = nullptr;
    PointVector Points_deleted;
    PointVector Downsample_Storage;
    PointVector Multithread_Points_deleted;
    void InitTreeNode(KD_TREE_NODE *root);
    void Test_Lock_States(KD_TREE_NODE *root);
    void BuildTree(KD_TREE_NODE **root, int l, int r, PointVector &Storage);
    void Rebuild(KD_TREE_NODE **root);
    int Delete_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild, bool is_downsample);
    void Delete_by_point(KD_TREE_NODE **root, const PointType &point, bool allow_rebuild);
    void Add_by_point(KD_TREE_NODE **root, const PointType &point, bool allow_rebuild, int father_axis);
    void Add_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild);
    void Search(KD_TREE_NODE *root, int k_nearest, const PointType &point, MANUAL_HEAP &q, double max_dist);
    void Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector &Storage);
    void Search_by_radius(KD_TREE_NODE *root, const PointType &point, float radius, PointVector &Storage);
    bool Criterion_Check(KD_TREE_NODE *root);
    void Push_Down(KD_TREE_NODE *root);
    void Update(KD_TREE_NODE *root);
    void delete_tree_nodes(KD_TREE_NODE **root);
    void downsample(KD_TREE_NODE **root);
    inline bool same_point(const PointType &a, const PointType &b);
    inline float calc_dist(const PointType &a, const PointType &b);
    inline float calc_box_dist(KD_TREE_NODE *node, const PointType &point);
    static inline bool point_cmp_x(PointType a, PointType b);
    static inline bool point_cmp_y(PointType a, PointType b);
    static inline bool point_cmp_z(PointType a, PointType b);

    void flatten(KD_TREE_NODE *root, PointVector &Storage, delete_point_storage_set storage_type);

    PointVector PCL_Storage;
    KD_TREE_NODE *Root_Node = nullptr;

  public:
    KdTree(float delete_param = 0.5, float balance_param = 0.7, float box_length = 0.2,
           int rebuild_log_capacity = MANUAL_Q<int>::kDefaultCapacity);
    ~KdTree();
    void set_delete_criterion_param(float delete_param);
    void set_balance_criterion_param(float balance_param);
    void set_downsample_param(float box_length);
    void initialize(float delete_param = 0.5, float balance_param = 0.7, float box_length = 0.2);
    int size();
    int validnum();
    void root_alpha(float &alpha_bal, float &alpha_del);
    void build(PointVector point_cloud);
    void nearest_search(const PointType &point, int k_nearest, PointVector &Nearest_Points,
                        std::vector<float> &Point_Distance, double max_dist = INFINITY);
    void box_search(const BoxPointType &Box_of_Point, PointVector &Storage);
    void radius_search(const PointType &point, const float radius, PointVector &Storage);
    int add_points(PointVector &PointToAdd, bool downsample_on);
    void add_point_boxes(std::vector<BoxPointType> &BoxPoints);
    int delete_points(PointVector &PointToDel);
    int delete_point_boxes(std::vector<BoxPointType> &BoxPoints);
    void flatten(PointVector &Storage);
    void acquire_removed_points(PointVector &removed_points);
    BoxPointType tree_range();

  private:
    int max_queue_size = 0;
};

} // namespace ikd_tree
