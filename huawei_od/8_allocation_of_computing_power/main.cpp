// 算法 - CPU算力分配 题目描述
// 现有两组服务器A和B，每组有多个算力不同的CPU，其中A
//             [i] 是A组第i个CPU的运算能力，B
//                 [i]
//                 是B组第i个CPU的运算能力。一组服务器的总算力是各CPU的算力之和。为了让两组服务器的算力相等，允许从每组各选出一个CPU进行一次交换，求两组服务器中，用于交换的CPU的算力，并且要求从A组服务器中选出的CPU，算力尽可能小。

//                     输入描述 :

//     第一行输入为L1和L2，以空格分隔，L1表示A组服务器中的CPU数量，L2表示B组服务器中的CPU数量。第二行输入为A组服务器中各个CPU的算力值，以空格分隔。第三行输入为B组服务器中各个CPU的算力值，以空格分隔。1
//     <= L1 <= 100001 <= L2 <= 100001 <= A[i] <= 1000001 <= B[i] <= 100000

//     输出描述 :

//     对于每组测试数据，输出两个整数，以空格分隔，依次表示A组选出的CPU算力、B组选出的CPU算力。要求从A组选出的CPU的算力尽可能小。

//         备注 : 保证两组服务器的初始总算力不同。答案肯定存在。
/**********思路*********/
// 通过计算两组服务器的总算力，并将A组服务器中每个CPU的算力值及其位置存储在一个HashMap中。然后遍历B组服务器的CPU算力值，计算出一个目标算力值，使得将该算力值从B组换到A组后，两组服务器的总算力相等。
// 具体步骤如下：

// 读取输入，包括A组和B组服务器中CPU的数量以及它们各自的算力值。
// 计算A组和B组的总算力分别为sumA和sumB。
// 创建一个HashMap(map)，用于记录A组中每个CPU的算力值和对应的索引位置。
// 遍历B组的CPU算力值，对于每个B组的CPU算力值，计算出需要从B组换到A组的目标算力值target，使得两组服务器的总算力相等。
// 检查目标算力值是否在A组中，如果在，则输出该目标算力值和当前B组的CPU算力值，即找到满足条件的一对CPU。
// 如果没有找到满足条件的一对CPU，程序结束。
// 通过这种方式，代码实现了在给定两组服务器的CPU算力值情况下，找到一对CPU使得交换后两组服务器的总算力相等的功能。
#include <climits>
#include <iostream>
#include <vector>
int main() {
  int L1, L2;
  std::cin >> L1 >> L2;
  std::vector<int> cpu1(L1), cpu2(L2);
  int sum_A = 0;
  int sum_B = 0;
  std::cin.ignore();
  for (int i = 0; i < L1; ++i) {
    std::cin >> cpu1[i];
    sum_A += cpu1[i];
  }
  std::cin.ignore();
  for (int i = 0; i < L2; ++i) {
    std::cin >> cpu2[i];
    sum_B += cpu2[i];
  }
  int sum = sum_A + sum_B;
  int a = 0;
  int b = 0;
  bool first = true;
  for (int c1 : cpu1) {
    for (int c2 : cpu2) {
      if (c1 != c2) {
        if ((sum_A - c1 + c2) == (sum_B - c2 + c1)) {
          if (first) {
            a = c1;
            b = c2;
            first = false;
          } else if (a > c1) {
            a = c1;
            b = c2;
          }
        }
      }
    }
  }
  std::cout << a << " " << b << std::endl;
  return 0;
}

// #include <stdio.h>

// int main() {
//   int L1 = 0;
//   int L2 = 0;
//   scanf("%d %d", &L1, &L2);
//   int A[100000] = {0};
//   int B[100000] = {0};
//   int i = 0;
//   int j = 0;
//   int flag = 0;
//   int a = 0;
//   int b = 0;
//   int sum_A = 0;
//   int sum_B = 0;
//   for (i = 0; i < L1; i++) {
//     scanf("%d", &A[i]);
//     sum_A += A[i];
//   }
//   for (i = 0; i < L2; i++) {
//     scanf("%d", &B[i]);
//     sum_B += B[i];
//   }
//   for (i = 0; i < L1; i++) {
//     for (j = 0; j < L2; j++) {
//       if (A[i] != B[j]) {
//         if ((sum_A + B[j] - A[i]) == (sum_B + A[i] - B[j])) {
//           if (flag == 0) {
//             a = A[i];
//             b = B[j];
//             flag = 1;
//           } else if (a > A[i] && flag == 1) {
//             a = A[i];
//             b = B[j];
//           }
//         }
//       }
//     }
//   }
//   printf("%d %d\n", a, b);
//   return 0;
// }
