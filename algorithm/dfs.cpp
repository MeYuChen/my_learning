#include <queue>
#include <vector>

class Node {
  int curr;
  Node* parent;
}

void bfsTraversal(const std::vector<std::vector<int>>& map) {
  std::vector<bool> visited(map.size(), false);
  std::queue<int> q;
  std::vector<int> res;
  for (int i = 0; i < map.size(); ++i) {
    if (!visited[i]) {
      visited[i] = true;
      q.push(i);
      while (!q.empty()) {
        int curr = q.front();
        q.pop();
        for (int i = 0; i < map[curr].size(); ++i) {
          int next = map[curr][i];
          if (!visited[next]) q.push(next);
          visited[next] = true;
        }
      }
    }
  }
}

int main() {
  std::vector<std::vector<int>> maze{{0, 1, 0, 0, 0},
                                     {0, 1, 0, 1, 0},
                                     {0, 0, 0, 0, 0},
                                     {0, 1, 1, 1, 0},
                                     {0, 0, 0, 1, 0}};
}