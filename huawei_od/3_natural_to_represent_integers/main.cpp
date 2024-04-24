#include <iostream>
#include <vector>

int sum(const std::vector<int>& v) {
  int res = 0;
  for (int i : v) res += i;
  return res;
}

bool isCon(const std::vector<int>& v) {
  if (v.size() > 1) {
    for (int i = v.size() - 1; i > 1; --i) {
      if (v[i] != v[i - 1] - 1) return false;
    }
  }
  return true;
}

void print(const std::vector<int>& v, int target) {
  std::cout << target << " = ";
  if (v.size() <= 1) {
    std::cout << v[0] << std::endl;
    return;
  }
  for (int i = 0; i < v.size() - 1; ++i) {
    std::cout << v[i] << " + ";
  }
  std::cout << v[v.size() - 1] << std::endl;
}

int main() {
  int target;
  std::vector<int> v;
  std::cin >> target;
  int sumcount = 0;
  for (int i = target; i > 0; --i) {
    for (int j = i; j > 0; --j) {
      v.emplace_back(j);
      if (sum(v) == target && isCon(v)) {  // 连续且等于目标
        print(v, target);
        v.clear();
        sumcount++;
        break;
      }
      if (sum(v) > target) {
        v.clear();
        break;
      }
    }
  }
  std::cout << "Result: " << sumcount << std::endl;
  return 0;
}

// //
// #include <iostream>
// #include <vector>

// using namespace std;

// bool isCon(vector<int> v) {
//   for (int i = v.size() - 1; i > 0; --i) {
//     if (v[i] != v[i - 1] - 1) return false;
//   }
//   return true;
// }

// int sum(vector<int> v) {
//   int sum = 0;
//   for (int i : v) sum += i;
//   return sum;
// }

// void print(vector<int> v, int n) {
//   cout << n << "=";
//   if (v.size() == 1) {
//     cout << v[0] << endl;
//     return;
//   }

//   int i;
//   for (i = v.size() - 1; i > 0; --i) cout << v[i] << "+";
//   cout << v[i] << endl;
// }

// int main() {
//   int n;
//   int sumcount = 0;
//   vector<int> v;
//   cin >> n;

//   for (int i = n; i >= 1; --i) {
//     for (int j = i; j >= 1; --j) {
//       v.push_back(j);
//       if (sum(v) == n && isCon(v)) {
//         print(v, n);
//         ++sumcount;
//         v.clear();
//         break;
//       }
//       if (sum(v) > n) {
//         v.clear();
//         break;
//       }
//     }
//   }
//   cout << "Result:" << sumcount;
// }
