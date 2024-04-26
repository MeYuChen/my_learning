// 题目描述
// 某个产品的RESTful
// API集合部署在服务器集群的多个节点上，近期对客户端访问日志进行了采集，需要统计各个API的访问频次，根据热点信息在服务器节点之间做负载均衡，现在需要实现热点信息统计查询功能。

// RESTful API是由多个层级构成，层级之间使用 / 连接，如 /A/B/C/D
// 这个地址，A属于第一级，B属于第二级，C属于第三级，D属于第四级。

// 现在负载均衡模块需要知道给定层级上某个名字出现的频次，未出现过用0表示，实现这个功能。

// 输入描述
// 第一行为N，表示访问历史日志的条数，0 ＜ N ≤ 100。

// 接下来N行，每一行为一个RESTful API的URL地址，约束地址中仅包含英文字母和连接符
// / ，最大层级为10，每层级字符串最大长度为10。

// 最后一行为层级L和要查询的关键字。

// 输出描述
// 输出给定层级上，关键字出现的频次，使用完全匹配方式（大小写敏感）。
// 输入
// 5
// /huawei/computing/no/one
// /huawei/computing
// /huawei
// /huawei/cloud/no/one
// /huawei/wireless/no/one
// 2 computing
// 输出
// 2

#include <iostream>
#include <sstream>
#include <vector>

int main() {
  int n;
  std::cin >> n;
  std::cin.ignore();  // 忽略换行符
  std::vector<std::vector<std::string>> list;
  for (int i = 0; i < n; i++) {
    std::string str;
    getline(std::cin, str);
    std::stringstream ss(str);
    std::string world;
    std::vector<std::string> worlds;
    while (getline(ss, world, '/')) {
      worlds.emplace_back(world);
    }
    list.emplace_back(worlds);
  }

  int L;
  std::string world;
  std::cin >> L >> world;
  int ans = 0;
  for (std::vector<std::string> str : list) {
    if (str.size() <= L) continue;
    if (str[L] == world) ans++;
  }
  std::cout << ans << std::endl;
  return 0;
}

// #include <iostream>
// #include <sstream>
// #include <vector>

// using namespace std;

// int main() {
//   int n;
//   cin >> n;
//   cin.ignore();

//   vector<vector<string>> lst;
//   for (int i = 0; i < n; i++) {
//     string line;
//     getline(cin, line);
//     stringstream ss(line);
//     string word;
//     vector<string> words;
//     while (getline(ss, word, '/')) {
//       words.push_back(word);
//     }
//     lst.push_back(words);
//   }

//   int L;
//   string word;
//   cin >> L >> word;
//   cout << L << word;
//   int ans = 0;

//   for (const vector<string>& words : lst) {
//     if (words.size() <= L) {
//       continue;
//     }
//     if (words[L] == word) {
//       ans++;
//     }
//   }

//   cout << ans << endl;

//   return 0;
// }
