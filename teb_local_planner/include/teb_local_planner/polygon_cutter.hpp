#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp>

#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/difference.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>
#include <boost/geometry/algorithms/correct.hpp>

#include <vector>

namespace teb_local_planner
{

namespace bg = boost::geometry;

class PolygonCutter
{
public:

    using Point = bg::model::d2::point_xy<double>;
    using Polygon = bg::model::polygon<Point>;
    using LineString = bg::model::linestring<Point>;
    using MultiLineString = bg::model::multi_linestring<LineString>;
    using MultiPolygon = bg::model::multi_polygon<Polygon>;

public:

    explicit PolygonCutter(double buffer_distance = 0.01)
        : distance_strategy_(buffer_distance)
    {
    }

    // -----------------------------
    // 添加切割线（可多次调用复用）
    // -----------------------------
    void addLine(const std::vector<Point>& pts)
    {
        if (pts.size() < 2) return;

        LineString line;
        line.reserve(pts.size());

        for (const auto& p : pts)
            line.push_back(p);

        cut_lines_.push_back(line);

        buffer_dirty_ = true;
    }

    // 清空所有切割线
    void clear()
    {
        cut_lines_.clear();
        buffer_cache_.clear();
        buffer_dirty_ = true;
    }

    // -----------------------------
    // 核心：polygon - buffer(lines)
    // -----------------------------
    MultiPolygon difference(Polygon polygon)
    {
        normalize(polygon);

        const MultiPolygon& cutter = getBuffer();

        MultiPolygon result;

        bg::difference(polygon, cutter, result);

        return result;
    }

private:

    // -----------------------------
    // buffer缓存（核心优化点）
    // -----------------------------
    const MultiPolygon& getBuffer()
    {
        if (!buffer_dirty_)
            return buffer_cache_;

        buffer_cache_.clear();

        bg::buffer(
            cut_lines_,
            buffer_cache_,
            distance_strategy_,
            side_strategy_,
            join_strategy_,
            end_strategy_,
            point_strategy_);

        buffer_dirty_ = false;

        return buffer_cache_;
    }

    // -----------------------------
    // polygon修正
    // -----------------------------
    void normalize(Polygon& poly)
    {
        if (!bg::is_valid(poly))
        {
            bg::correct(poly);
        }
    }


private:

    // -----------------------------
    // 输入切割线
    // -----------------------------
    MultiLineString cut_lines_;

    // -----------------------------
    // buffer缓存
    // -----------------------------
    MultiPolygon buffer_cache_;
    bool buffer_dirty_ = true;

    // -----------------------------
    // strategies（避免重复构造）
    // -----------------------------
    bg::strategy::buffer::distance_symmetric<double> distance_strategy_;
    bg::strategy::buffer::side_straight side_strategy_;
    bg::strategy::buffer::join_round join_strategy_{4};
    bg::strategy::buffer::end_round end_strategy_{4};
    bg::strategy::buffer::point_circle point_strategy_{4};
};

} // namespace teb_local_planner