/// @brief 将astar路径分段。拉直
/// @param mapPlan 二值化的占据地图，5cm分辨率
/// @param path astar稀疏的路径点
/// @return 返回拉直的路径点，也是比较稀疏的/或者原始的astar路径
std::vector<LDCV::Point> CTargetNavigatorEx::PathShorten(TMapData &mapPlan, std::vector<LDCV::Point> &path)
{
    // 传入的是像素路径点
    auto TaceSegSparse = [&](LDCV::Point point_one, LDCV::Point point_two) -> std::vector<LDCV::Point> {
        std::vector<LDCV::Point> path_sparse;
        path_sparse.push_back(point_one);
        // const float k_seg_len = 0.5 / 0.05;  // [pixel,]每一个小段的长度0.5m，对应10个像素距离
        float dis             = sqrt((point_one.x - point_two.x) * (point_one.x - point_two.x) + (point_one.y - point_two.y) * (point_one.y - point_two.y));
        int seg_num           = dis / k_seg_len + 0.5;
        if (seg_num < 2) {
            return path_sparse;
        }
        float delta_x = (point_two.x - point_one.x) / (1.f * seg_num);
        float delta_y = (point_two.y - point_one.y) / (1.f * seg_num);
        for (int i = 1; i < seg_num; i++) {
            LDCV::Point seg_point(point_one.x + delta_x * i + 0.5f, point_one.y + delta_y * i + 0.5f);
            path_sparse.push_back(seg_point);
        }
        return path_sparse;
    };

    auto TacePathSparse = [=](std::vector<LDCV::Point> basePoints) -> std::vector<LDCV::Point> {
        if (basePoints.size() < 3) {
            return basePoints;
        }
        std::vector<LDCV::Point> ret;
        for (unsigned int i = 0; i < basePoints.size() - 1; i++) {
            std::vector<LDCV::Point> temp = TaceSegSparse(basePoints[i], basePoints[i + 1]);
            // ret.emplace_back(basePoints[i]);
            ret.insert(ret.end(), temp.begin(), temp.end());
        }
        ret.emplace_back(basePoints.back());
        return ret;
    };

    // 优化拉直拉短
    auto path_points_sparse = TacePathSparse(path);
    std::vector<LDCV::Point> path_adjusted;
    bool is_path_adjust_work = false;
    do {
        path_adjust::PathShorten pp(path_points_sparse.size());
        if (path_points_sparse.size() < 3) {
            break;
        }
        for (size_t i = 0; i < path_points_sparse.size() - 1; i++) {
            auto point_one = path_points_sparse[i];
            auto point_two = path_points_sparse[i + 1];
            float dis      = sqrt((point_one.x - point_two.x) * (point_one.x - point_two.x) + (point_one.y - point_two.y) * (point_one.y - point_two.y));
            pp.AddEdge(i, i + 1, dis);
        }
        for (size_t i = 0; i < path_points_sparse.size() - 2; i++) {
            // linehit
            int meetHit = 0;
            for (size_t j = i + 2; j < path_points_sparse.size(); j++) {
                int x1 = path_points_sparse[i].x;
                int y1 = path_points_sparse[i].y;
                int x2 = path_points_sparse[j].x;
                int y2 = path_points_sparse[j].y;
                int hitX;
                int hitY;
                if (LDCV::CCommonAlg::LineHit(hitX, hitY, 128, x1, y1, x2, y2,
                                            &mapPlan.map[0], mapPlan.mapParam.width, mapPlan.mapParam.height)) {
                    //                     LOGD("hitPoint = (%.2f, %.2f)", slamMap.idx2x(hitX), slamMap.idx2y(hitY));
                    meetHit++;  // 连线发生hit最大允许向前搜索的次数
                    if (meetHit > 5) {
                        break;
                    } else {
                        continue;
                    }
                } else {
                    //
                    auto point_one = path_points_sparse[i];
                    auto point_two = path_points_sparse[j];
                    float dis      = sqrt((point_one.x - point_two.x) * (point_one.x - point_two.x) + (point_one.y - point_two.y) * (point_one.y - point_two.y));
                    pp.AddEdge(i, j, dis);
                }
            }
        }
        std::vector<int> ret = pp.GetShortestPath();
        for (auto item : ret) {
            path_adjusted.push_back(path_points_sparse[item]);
        }
        if (path_adjusted.size() < path_points_sparse.size()) {
            FLOGD("--PATH ADJUST WORKS.");
            is_path_adjust_work = true;
        }
    } while (0);
    // end 优化拉直
    if (is_path_adjust_work == true) {
        return path_adjusted;
    } else {
        return path;
    }
}