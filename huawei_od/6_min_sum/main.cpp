
// 描述
// 给定两个整数数组a r r a y 1 、 a r r a y 2
// array1、array2array1、array2，数组元素按升序排列，假设从a r r a y 1 、 a r r
// a y 2
// array1、array2array1、array2中分别取出一个元素可构成一对元素，现在需要取出k对元素并对取出的所有元素求和，计算和的最小值。

// 注意:

// 两对元素如果对应于a r r a y 1 、 a r r a y 2
// array1、array2array1、array2中的两个下标均相同，则视为同一对元素

// 输入描述：
// 输入两行数组a r r a y 1 、 a r r a y 2
// array1、array2array1、array2，每行首个数字为数组大小s i z e ( 0 < s i z e < =
// 100 ) size(0<size<=100)size(0<size<=100) 0 < a r r a y 1 [ i ] < = 1000
// 0<array1[i]<=10000<array1[i]<=1000 0 < a r r a y 2 [ i ] < = 1000
// 0<array2[i]<=10000<array2[i]<=1000 接下来一行为正整数k 0 < k < = a r r a y 1.
// s i z e ( ) ∗ a r r a y 2. s i z e ( )
// 0<k<=array1.size()*array2.size()0<k<=array1.size()∗array2.size()

// 输出描述：
// 满足要求的最小和

// 示例1

// 输入：
// 3 1 1 2
// 3 1 2 3
// 2
// 输出：
// 4
// 说明：
// 用例中，需要取2对元素
// 取第一个数组第1个元素与第二个数组第1个元素组成1对元素[1,1];
// 取第一个数组第2个元素与第二个数组第1个元素组成1对元素[1,1];
// 求和为1+1+1+1 = 4，为满足要求的最小和。
#include <algorithm>
#include <iostream>
#include <vector>

int main() {
  int n1, n2;
  std::cin >> n1;
  std::vector<int> arr1(n1);
  int tmp_n;
  for (int i = 0; i < n1; ++i) std::cin >> arr1[i];
  std::cin >> n2;
  std::vector<int> arr2(n2);
  for (int i = 0; i < n2; ++i) std::cin >> arr2[i];

  std::vector<int> sum;
  for (int val_1 : arr1) {
    for (int val_2 : arr2) {
      sum.emplace_back(val_1 + val_2);
    }
  }
  std::sort(sum.begin(), sum.end());
  int k;
  std::cin >> k;
  int res = 0;
  for (int i = 0; i < k; ++i) {
    res += sum[i];
  }
  std::cout << res << std::endl;
  return 0;
}

// #include <algorithm>
// #include <iostream>
// #include <vector>

// using namespace std;

// int main() {
//   int n1, n2, k;
//   cin >> n1;
//   vector<int> arr1(n1);
//   for (int i = 0; i < n1; i++) cin >> arr1[i];

//   cin >> n2;
//   vector<int> arr2(n2);
//   for (int i = 0; i < n2; i++) cin >> arr2[i];

//   cin >> k;

//   vector<int> pairs;
//   for (int v1 : arr1) {
//     for (int v2 : arr2) {
//       pairs.push_back(v1 + v2);
//     }
//   }

//   sort(pairs.begin(), pairs.end());

//   int sumV = 0;
//   for (int i = 0; i < k; i++) {
//     sumV += pairs[i];
//   }

//   cout << sumV << endl;

//   return 0;
// }
