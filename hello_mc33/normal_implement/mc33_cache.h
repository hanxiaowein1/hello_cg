#ifndef __CHARLES_MC33_CACHE_H__
#define __CHARLES_MC33_CACHE_H__

#include <unordered_map>
#include <boost/functional/hash.hpp>
#include "charles_mc33_type.h"

class MC33CacheMissing : public std::out_of_range {
public:
    explicit MC33CacheMissing(const std::string& what_arg)
        : std::out_of_range(what_arg) {}
    
    explicit MC33CacheMissing(const char* what_arg)
        : std::out_of_range(what_arg) {}
};

// key: z, y, x, edge(from 0 ~ 7)
template <typename T>
class MC33Cache
{
private:
    std::unordered_map<std::tuple<int, int, int, Edge>, T, boost::hash<std::tuple<int, int, int, Edge>>> cache;
    std::unordered_map<Edge, std::vector<std::tuple<int, int, int, Edge>>> previous_keys = {
        {
            Edge::e0,
            {
                // down back
                {-1, 0, -1, Edge::e6},
                // down
                {-1, 0, 0, Edge::e2},
                // back
                {0, 0, -1, Edge::e4},
            }
        },
        {
            Edge::e1,
            {
                // back
                {0, 0, -1, Edge::e5},
            }
        },
        {
            Edge::e2,
            {
                // back
                {0, 0, -1, Edge::e6},
            }
        },
        {
            Edge::e3,
            {
                // left back
                {0, -1, -1, Edge::e5},
                // left
                {0, -1, 0, Edge::e1},
                // back
                {0, 0, -1, Edge::e7},
            }
        },
        {
            Edge::e4,
            {
                // down
                {-1, 0, 0, Edge::e6},
                // down front
                {-1, 0, 1, Edge::e2},
            }
        },
        {
            Edge::e5,
            {
                // impossible
            }
        },
        {
            Edge::e6,
            {
                // impossible
            }
        },
        {
            Edge::e7,
            {
                // left
                {0, -1, 0, Edge::e5},
                // left front
                {0, -1, 1, Edge::e1},
            }
        },
        {
            Edge::e8,
            {
                // down left
                {-1, -1, 0, Edge::e10},
                // down
                {-1, 0, 0, Edge::e11},
                // left
                {0, -1, 0, Edge::e9},
            }
        },
        {
            Edge::e9,
            {
                // down
                {-1, 0, 0, Edge::e10},
                // down right
                {-1, 1, 0, Edge::e11},
            }
        },
        {
            Edge::e10,
            {
                // impossible
            }
        },
        {
            Edge::e11,
            {
                // left
                {0, -1, 0, Edge::e10},
            }
        },
        {
            Edge::ec,
            {
                // interpolation, no previous search condition.
            }
        }
    };
public:
    T get(const std::tuple<int, int, int, Edge>& key);
    void set(const std::tuple<int, int, int, Edge>& key, const T& value);
    int size()
    {
        return this->cache.size();
    }
};

template <typename T>
T MC33Cache<T>::get(const std::tuple<int, int, int, Edge>& key)
{
    // TODO: can be speed up. for example, I can search directly by key, then by many cases, it will get the value, if not, then go to the catch logic, then handle each edge separately
    // interesting, edge need be recalculated, because (0, 0, 0, Edge::e4) equals (0, 0, 1, Edge::e0)
    const auto& [z, y, x, edge] = key;
    const auto& previous_boxes = this->previous_keys.at(edge);
    for(const auto& previous_box: previous_boxes)
    {
        std::tuple<int, int, int, Edge> search_box = std::make_tuple(
            z + std::get<0>(previous_box),
            y + std::get<1>(previous_box),
            x + std::get<2>(previous_box),
            std::get<3>(previous_box)
        );
        if(this->cache.contains(search_box))
        {
            return this->cache.at(search_box);
        }
    }
    // seems visual studio cannot skip this line's exception, and cannot skip user handled exception, so, it's better to use self defined exception
    // return this->cache.at(key);
    if(this->cache.contains(key))
    {
        return this->cache.at(key);
    }
    else
    {
        throw MC33CacheMissing("missing key in mc33 cache!");
    }
}

template <typename T>
void MC33Cache<T>::set(const std::tuple<int, int, int, Edge>& key, const T& value)
{
    this->cache.emplace(key, value);
}

#endif