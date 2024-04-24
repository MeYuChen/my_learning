// 小明今年升学到小学一年级Q，来到新班级后发现其他小朋友们身高参差不齐，然后就想基于各小朋友和自己的身高差对他们进行排序，请帮他实现排序。
// 输入描述:
// 第一行为正整数H和N，0<H<200，为小明的身高，0<N<50，为新班级其他小朋友个数第二行为N个正整数H1-HN，分别是其他小朋友的身高，取值范围0<Hi200(1<==N)，N个正整数各不相同输出描述:
// 输出排席结果，各正整数以空格分割。和小明身高差绝对值最小的小朋友排在前面，和小明身高差绝对值最大的小朋友排在最后如果两个小朋友和小明身高差一样，则个子较小的小朋友排在前面。
// 输入
// 100 10
// 95 96 97 98 99 101 102 103 104 105
// 输出
// 99 101 98 102 97 103 96 104 95 105

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

bool compare(int left, int right, int mid_height) {
  int left_offset = std::abs(left - mid_height);
  int right_offset = std::abs(right - mid_height);
  if (left_offset == right_offset) {
    return left < right;
  }
  return left_offset < right_offset;
}

int main() {
  int height, stnums;
  std::cin >> height >> stnums;
  std::vector<int> heights(stnums);
  for (int i = 0; i < stnums; ++i) {
    std::cin >> heights[i];
  }
  std::sort(heights.begin(), heights.end(),
            [&](int left, int right) { return compare(left, right, height); });
  for (int num : heights) {
    std::cout << num << " ";
  }
  std::cout << std::endl;
}

// #include <iostream>
// #include <vector>
// #include <algorithm>
// #include <cmath>

// using namespace std;

// bool compare(int h1, int h2, int mingHeight) {
//     int different1 = abs(h1 - mingHeight);
//     int differert2 = abs(h2 - mingHeight);

//     if (different1 == differert2) {
//         return h1 < h2;
//     }
//     return different1 < differert2;
// }

// int main() {
//     int mingHeight, numFriends;
//     cin >> mingHeight >> numFriends;

//     vector<int> frinedHeights(numFriends);

//     for (int i = 0; i < numFriends; i++) {
//         cin >> frinedHeights[i];
//     }

//     sort(frinedHeights.begin(), frinedHeights.end(), [&](int h1, int h2) {
//         return compare(h1, h2, mingHeight);
//     });

//     for (int height : frinedHeights) {
//         cout << height << " ";
//     }

//     return 0;
// }
