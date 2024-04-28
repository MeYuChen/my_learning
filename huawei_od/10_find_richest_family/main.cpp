// 在一棵树中，每个节点代表一个家庭成员，节点的数字表示其个人的财富值，一个节点及其直接相连的子节点被定义为一个小家庭。
// 现给你一棵树，请计算出最富裕的小家庭的财富和。
// 输入描述：第一行为一个数N，表示成员总数，成员编号1-N，1<=N<=1000
// 第二行为N个空格分隔的数，表示编号1-N的成员的财富值。0<=财富值<=1000000
// 接下来N-1行，每行两个空格分隔的整数(N1,N2)，表示N1是N2的父节点。
// 输出描述：
// 最富裕的小家庭的财富和
// 补充说明：
// 示例1
// 输入：
// 4
// 100 200 300 500
// 1 2
// 1 3
// 2 4
// 输出：
// 700
// 说明：
// 成员1,2,3组成的小家庭财富值为600
// 成员2,4组成的小家庭财富值为700
// 示例2
// 输入：
// 4
// 100 200 300 500
// 1 2
// 1 3
// 1 4
// 输出：
// 1100
// 说明：
// 成员1,2,3,4组成的小家庭财富值为1100

// 解题思路
// 这个问题可以通过深度优先搜索（DFS）来解决。以下是一个可能的解题思路：

// 构建树的数据结构：
// 根据输入的父节点信息构建树的数据结构，可以使用邻接表或者树的节点类来表示。
// #include <cstddef>
// #include <iostream>
// using namespace std;
// #include <cmath>
// #include <map>
// #include <vector>

// int main() {
//   int N;
//   cin >> N;
//   vector<int> val(N + 1, 0);
//   int i = 0;
//   while (i < N) {
//     cin >> val[i + 1];
//     i++;
//   }
//   i = 0;
//   map<int, vector<int>> mp;
//   while (i < N - 1) {
//     int x1, x2;
//     cin >> x1 >> x2;
//     if (mp.find(x1) != mp.end()) {
//       mp[x1].push_back(x2);
//     } else {
//       vector<int> v;
//       v.push_back(x2);
//       mp.insert(make_pair(x1, v));
//     }
//     i++;
//   }
//   int res = 0;
//   auto it = mp.begin();
//   while (it != mp.end()) {
//     int acc = val[it->first];
//     for (int i = 0; i < (it->second).size(); i++) {
//       acc += val[(it->second)[i]];
//     }
//     if (res < acc) res = acc;
//     it++;
//   }
//   cout << res << endl;

//   return 0;
// }