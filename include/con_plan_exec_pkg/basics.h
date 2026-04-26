#include <vector>


using namespace std;


struct s2
{
  int x;
  int y;
};

typedef struct s2 position;

typedef std::vector<position> pos_vec_t;

struct s3
{
  int x;
  int y;
  double theta;	
  	
};

typedef struct s3 position_theta;

typedef std::vector<position_theta> pos_theta_vec_t;


struct d
{
  unsigned int length_x;
  unsigned int length_y;
};

typedef struct d dimension_t;


struct c
{
  float max_cost;
  float min_cost;
  float min2_cost;
  float min_cost_diff;
  float min2_min_cost_diff;
};

typedef struct c prim_cost_t;


struct plan
{
  int id;
  int time_instance;
  int location_x;
  int location_y;
  double theta;
};

typedef struct plan motionPlan;
typedef std::vector<motionPlan> plan_vec_t;
typedef vector<plan_vec_t> plan_mat;


struct status
{
  int id;
  int status;
  int x;
  int y;
  double theta;	 	
};

typedef struct status robot_status;
typedef std::vector<robot_status> robot_status_vec_t;


typedef std::vector<int> Cluster;

//std::vector< std::vector<double> > rd;
typedef vector<vector<double> > Rewards;
bool isObstacle (pos_vec_t , unsigned int , unsigned int );

struct loc
{
    int x;
    int y;
    int theta;
};


struct node     //For stacks and queues
{
    int index;
    int is_robot;
};

typedef vector<bool> bool_vec;
typedef vector<int> int_vec;
typedef vector<uint> uint_vec;
typedef vector<float> float_vec;
typedef vector<double> double_vec;
typedef vector<struct loc> loc_vec;

typedef vector<bool_vec> bool_mat;
typedef vector<int_vec> int_mat;
typedef vector<float_vec> float_mat;
typedef vector<double_vec> double_mat;
typedef vector<loc_vec> loc_mat;