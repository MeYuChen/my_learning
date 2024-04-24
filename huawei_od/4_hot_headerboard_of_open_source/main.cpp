#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <vector>

class Project {
 private:
  /* data */
 public:
  Project(int hot, std::string name) : hot_(hot), name_(name) {}
  int hot_;
  std::string name_;
};

bool compare(const Project& a, const Project& b) {
  if (a.hot_ != b.hot_) {
    return a.hot_ > b.hot_;
  }
  return a.name_ < b.name_;
}

int main() {
  int project_num;
  std::cin >> project_num;
  std::vector<int> weights;
  for (int i = 0; i < 5; ++i) {
    int weight = 0;
    std::cin >> weight;
    weights.emplace_back(weight);
  }
  std::vector<Project> projects;
  for (int i = 0; i < project_num; ++i) {
    std::string name;
    std::cin >> name;
    int hot_val = 0;
    for (int i = 0; i < 5; ++i) {
      int type;
      std::cin >> type;
      type = type * weights[i];
      hot_val += type;
    }
    Project proj(hot_val, name);
    projects.emplace_back(proj);
  }

  std::sort(projects.begin(), projects.end(), compare);
  for (const auto& proj : projects) {
    std::cout << proj.name_ << std::endl;
  }
}

// #include <iostream>
// #include <vector>
// #include <algorithm>
// #include <string>

// class Project {
// public:
//     std::string name;
//     int hot;

//     Project(std::string name, int hot) : name(name), hot(hot) {}
// };

// bool compare(const Project &a, const Project &b) {
//     if (a.hot != b.hot) {
//         return a.hot > b.hot;
//     }
//     return a.name < b.name;
// }

// int main() {
//     int n;
//     std::cin >> n;

//     std::vector<int> weights(5);
//     for (int i = 0; i < 5; i++) {
//         std::cin >> weights[i];
//     }

//     std::vector<Project> projects;
//     for (int i = 0; i < n; i++) {
//         std::string name;
//         std::cin >> name;

//         std::vector<int> statistics(5);
//         for (int j = 0; j < 5; j++) {
//             std::cin >> statistics[j];
//         }

//         int hot = 0;
//         for (int j = 0; j < 5; j++) {
//             hot += statistics[j] * weights[j];
//         }

//         projects.push_back(Project(name, hot));
//     }

//     std::sort(projects.begin(), projects.end(), compare);

//     for (const auto &p : projects) {
//         std::cout << p.name << std::endl;
//     }

//     return 0;
// }
