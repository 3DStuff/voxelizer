#pragma once

#include <vector>

namespace hidden {
    template <typename base_t>
    using buf_1d = std::vector<base_t>;
    template <typename base_t>
    using buf_2d = std::vector<buf_1d<base_t>>;
    template <typename base_t>
    using buf_3d = std::vector<buf_2d<base_t>>;
};

//! simple 2d array
template <typename base_t>
class buffer2d {
private:
    hidden::buf_2d<base_t> _buf;

public:
    buffer2d() = default;
    buffer2d(size_t x, size_t y) 
        : _buf(x, hidden::buf_1d<base_t>(y))
    {}
    buffer2d(size_t x, size_t y, base_t val) 
        : _buf(x, hidden::buf_1d<base_t>(y, val))
    {}

    void clear() {
        _buf.clear();
    }
    
    size_t size() const {
        return _buf.size();
    }
    
    hidden::buf_1d<base_t> &operator[](size_t i) {
        return _buf[i];
    }
    const hidden::buf_1d<base_t> &operator[](size_t i) const {
        return _buf[i];
    }
};

//! simple 3d array
template <typename base_t>
class buffer3d {
private:
    hidden::buf_3d<base_t> _buf;

public:
    buffer3d() = default;
    buffer3d(size_t x, size_t y, size_t z) 
        : _buf(x, 
               hidden::buf_2d<base_t>(y, 
               hidden::buf_1d<base_t>(z))
              )
    {}
    buffer3d(size_t x, size_t y, size_t z, base_t val) 
        : _buf(x, 
               hidden::buf_2d<base_t>(y, 
               hidden::buf_1d<base_t>(z, val))
              )
    {}

    void clear() {
        _buf.clear();
    }
    
    size_t size() const {
        return _buf.size();
    }
    
    hidden::buf_2d<base_t> &operator[](size_t i) {
        return _buf[i];
    }
    const hidden::buf_2d<base_t> &operator[](size_t i) const {
        return _buf[i];
    }
};
