// 题目描述与示例
// 题目描述

// 小明在玩一个游戏，游戏规则如下：在游戏开始前，小明站在坐标轴原点处（坐标值为
// 0）给定一组指令和一个幸运数，每个指令都是一个整数，小明按照指定的要求前进或者后退指定的步数。

// 前进代表朝坐标轴的正方向走，后退代表朝坐标轴的负方向走，幸运数为一个整数，如果某个指令正好和幸运数相等，则小明行进步数加
// 1 。

// 例如：

// 幸运数为 3 ，指令数组为[2, 3, 0, -5]

// 指令为 2 ，表示前进 2 步

// 指令为 3 ，正好好和幸运数相等，前进 3+1=4 步

// 指令为 0 ，表示原地不动，既不前进，也不后退

// 指令为 -5 ，表示后退 5 步。

// 请你计算小明在整个游戏过程中，小明所处的最大坐标值。
// 输入描述

// 第一行输入 1 个数字，代表指令的总个数 n(1≤n≤100）

// 第二行输入 1 个数字，代表幸运数 m(−100≤m≤100)

// 第三行输入 n 个指令，每个指令值的取值范围为： − 100 ≤ 指令值 ≤ 100
// 输出描述

// 输出在整个游戏过程中，小明所处的最大坐标值。异常情况下输出：12345
// 示例一
// 输入

// 2
// 1
// -5 1

//     1
//     2
//     3

// 输出

// 0

//     1

// 说明

// 总共 2 个指令，幸运数为 1
// ，依照指令行进，依次如下游戏开始前，站在坐标轴原点，此时坐标值为 0 ；

// 指令为 −5 ，后退 5 步 ，此时坐标值为 −5 ；指令为 1 ，正好等于幸运数，前进
// 1+1=2 步，此时坐标值为 −3；

// 整个游戏过程中，小明所处的坐标值依次为[0,−5,−3]，最大坐标值为 0 。
// 示例二
// 输入

// 5
// -5
// -5 1 6 0 -7

//     1
//     2
//     3

// 输出

// 1

//     1

// 说明

// 总共 5 个指令，幸运数为 −5，依照指令行进，依次如下：

// 游戏开始前，站在坐标轴原点，此时坐标值为 0 ，

// 指令为 −5，正好等于幸运数，后退 5+1=6 步，此时坐标值为 −6；

// 指令为 1 ，前进 1 步此时坐标值为 −5 ；

// 指令为 6 ，前进 6 步此时坐标值为 1 ；

// 指令为 0 ，既不前进也不后退，此时坐标值为 1 ：

// 指令为 −7，后退 7 步，此时坐标值为 −6。

// 整个游戏过程中，小明所处的坐标值依次为 [0,−6,−5,1,1,−6]，最大坐标值为 1 。
// 解题思路

// 直接按照题意进行模拟即可。

// 需要注意的地方有两点：

//     题目提示了可能输入会有异常，使用try-except语句进行排除。
//     当幸运数字m是负数时，此时行进的方向是往后走，步数增加1指的是往后多退一步。

#include <iostream>

int main() {
  int command_num;
  std::cin >> command_num;
  if (command_num > 100 || command_num < 1) {
    std::cout << 12345 << std::endl;
    return 0;
  }

  int luck_val;
  std::cin.ignore();
  std::cin >> luck_val;
  if (luck_val < -100 || luck_val > 100) {
    std::cout << luck_val << std::endl;
    std::cout << 12345 << std::endl;
    return 0;
  }
  int command;
  int curr = 0;
  std::cin.ignore();
  int max_location = 0;
  for (int i = 0; i < command_num; ++i) {
    std::cin >> command;
    if (command < -100 || command > 100) {
      std::cout << 12345 << std::endl;
      return 0;
    }
    curr += command;
    if (command == luck_val) {
      if (command > 0) {
        curr += 1;
      } else {
        curr += -1;
      }
    }
    if (curr > max_location) max_location = curr;
  }
  std::cout << max_location << std::endl;
  return 0;
}

// #include <iostream>
// #include <vector>
// using namespace std;

// int main() {
//   int n, lucky;
//   cin >> n;
//   cin >> lucky;

//   if (n < 1 || n > 100 || lucky < -100 || lucky > 100) {
//     cout << 12345;
//     return 0;
//   }

//   int i = 0;
//   int pos = 0;
//   int ans = 0;
//   while (i < n) {
//     int a;
//     cin >> a;
//     if (a < -100 || a > 100) {
//       cout << 12345;
//       return 0;
//     }

//     if (a == lucky) {
//       if (a > 0)
//         a += 1;
//       else
//         a -= 1;
//     }

//     pos += a;
//     ans = max(ans, pos);

//     i++;
//   }

//   cout << ans;
//   return 0;
// }