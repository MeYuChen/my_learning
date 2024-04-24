

// 思路：
// 将黑白棋的位置处理成字符串，由于横纵向坐标不同所以字符串唯一，使用unorder_set能够保证数据不会重复

#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>
int Edge = 18;
std::vector<std::string> transform(const std::vector<std::string>& strs) {
  std::vector<std::string> chess(strs.size() / 2);
  for (int i = 0; i < strs.size();) {
    chess[i / 2] = strs[i] + "_" + strs[i + 1];
    i += 2;
  }
  return chess;
}

int counting(const std::vector<std::string>& str1,
             const std::vector<std::string>& str2) {
  std::unordered_set<std::string> count;
  for (std::string location : str1) {
    count.insert(location);
    int pos = location.find("_");
    int x = std::stoi(location.substr(0, pos));
    int y = std::stoi(location.substr(pos + 1));
    if (x > 0)
      count.insert(std::to_string(x - 1) + "_" + location.substr(pos + 1));
    if (x < Edge)
      count.insert(std::to_string(x + 1) + "_" + location.substr(pos + 1));
    if (y > 0)
      count.insert(location.substr(0, pos) + "_" + std::to_string(y - 1));
    if (y < Edge)
      count.insert(location.substr(0, pos) + "_" + std::to_string(y + 1));
  }
  int res = count.size() - str1.size();
  for (std::string str : str2) {
    if (count.count(str)) {
      res--;
    }
  }
  return res;
}

int main() {
  // 输入两行数据
  std::vector<std::string> white;
  std::vector<std::string> black;
  std::string input;
  std::getline(std::cin, input);
  int pos = 0;
  while ((pos = input.find(" ")) != std::string::npos) {
    black.emplace_back(input.substr(0, pos));
    input.erase(0, pos + 1);
  }
  black.emplace_back(input.substr(0, pos));

  std::getline(std::cin, input);
  pos = 0;
  while ((pos = input.find(" ")) != std::string::npos) {
    white.emplace_back(input.substr(0, pos));
    input.erase(0, pos + 1);
  }
  white.emplace_back(input.substr(0, pos));

  std::vector<std::string> black_chess = transform(black);
  std::vector<std::string> white_chess = transform(white);
  std::cout << counting(black_chess, white_chess) << " "
            << counting(white_chess, black_chess) << std::endl;
}

// #include <iostream>
// #include <string>
// #include <unordered_set>
// #include <vector>

// using namespace std;

// int maxSide = 18;

// int counting(vector<string>& alias, vector<string>& enemy) {
//   //  unordered_set不允许有重复的数据插入
//   unordered_set<string> count;
//   for (string a : alias) {
//     count.insert(a);
//     size_t pos = a.find("_");
//     int x = stoi(a.substr(0, pos));
//     int y = stoi(a.substr(pos + 1));
//     if (x > 0) {
//       count.insert(to_string(x - 1) + "_" + a.substr(pos + 1));
//     }
//     if (x < maxSide) {
//       count.insert(to_string(x + 1) + "_" + a.substr(pos + 1));
//     }
//     if (y > 0) {
//       count.insert(a.substr(0, pos) + "_" + to_string(y - 1));
//     }
//     if (y < maxSide) {
//       count.insert(a.substr(0, pos) + "_" + to_string(y + 1));
//     }
//   }
//   //   移除棋子自身占据的位置
//   int res = count.size() - alias.size();
//   // 移除 敌方棋子占据的位置
//   for (string e : enemy) {
//     if (count.count(e)) {
//       res--;
//     }
//   }
//   return res;
// }

// vector<string> transform(vector<string>& locs) {
//   vector<string> chess(locs.size() / 2);
//   for (int i = 0; i < locs.size();) {
//     chess[i / 2] = locs[i] + "_" + locs[i + 1];
//     i += 2;
//   }
//   return chess;
// }

// int main() {
//   vector<string> locBlacks;
//   vector<string> locWhites;
//   string input;
//   getline(cin, input);
//   size_t pos = 0;
//   while ((pos = input.find(" ")) != string::npos) {
//     locBlacks.push_back(input.substr(0, pos));
//     input.erase(0, pos + 1);
//   }
//   locBlacks.push_back(input);

//   getline(cin, input);
//   pos = 0;
//   while ((pos = input.find(" ")) != string::npos) {
//     locWhites.push_back(input.substr(0, pos));
//     input.erase(0, pos + 1);
//   }
//   locWhites.push_back(input);

//   vector<string> blacks = transform(locBlacks);
//   vector<string> whites = transform(locWhites);

//   cout << counting(blacks, whites) << " " << counting(whites, blacks) <<
//   endl;

//   return 0;
// }
