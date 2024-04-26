// ■ 题目描述

// 【素数之积】

// RSA加密算法在网络安全世界中无处不在，它利用了极大整数因数分解的困难度，数据越大，安全系数越高。

// 给定一个32位正整数，请对其进行因数分解，找出是哪两个素数的乘积。

// 输入描述

// 一个正整数num

// 0 < num <= 2147483647

// 输出描述

// 如果成功找到，以单个空格分割，从小到大输出两个素数，分解失败，请输出-1 -1。

// 示例1 输入输出示例仅供调试，后台判题数据一般不包含示例

// 输入

// 15

// 输出

// 3 5

// 说明

// 因数分解后，找到两个素数3和5，使得3*5=15，按从小到大排列后，输出3 5。

// 示例2 输入输出示例仅供调试，后台判题数据一般不包含示例

// 输入

// 27

// 输出

// -1 -1

// 说明

// 通过因数分解，找不到任何素数，使得他们的乘积为27，输出-1 -1。

/**************************************/
// 考察递归
/**************************************/

// 为什么使用 sqrt(num)：

// 数学原理：一个合数（非素数）必定有一个不大于它平方根的素数因子。换句话说，如果
// num
// 有一个大于其平方根的因子，那么它必然也有一个小于或等于其平方根的因子。这是因为，如果
// num 可以被某个数 f 整除，且 f 大于 sqrt(num)，那么 num/f 必定小于 sqrt(num)
// 且能整除 num。

// 效率提升：通过只检查到
// sqrt(num)，可以显著减少判断一个数是否为素数所需的迭代次数。对于大数值，这种方法可以节省大量计算时间。

#include <cmath>
#include <iostream>
bool is_pr(int num) {
  for (int i = 2; i <= std::sqrt(num); ++i) {
    if (num % i == 0) return false;
  }
  return true;
}

int main() {
  int num;
  std::cin >> num;
  bool got_it = false;
  for (int i = 2; i <= std::sqrt(num); ++i) {
    if (num % i == 0 && is_pr(i) && is_pr(num / i)) {
      std::cout << i << " " << num / i << std::endl;
      got_it = true;
      break;
    }
  }
  if (!got_it) std::cout << -1 << " " << -1 << std::endl;
  return 0;
}

// #include <iostream>
// using namespace std;
// #include <cmath>
// bool is_pr(int num) {
//   bool res = true;
//   for (int i = 2; i <= sqrt(num); i++) {
//     if (num % i == 0) return false;
//   }
//   return res;
// }

// int main() {
//   int num;
//   cin >> num;
//   bool f = false;
//   for (int i = 2; i <= sqrt(num); i++) {
//     if (num % i == 0 && is_pr(i)) {
//       if (is_pr(num / i)) {
//         cout << i << " " << num / i << endl;
//         f = true;
//         break;
//       }
//     }
//   }
//   if (!f) cout << -1 << " " << -1 << endl;
//   return 0;
// }
