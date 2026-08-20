#ifndef SD_RING_BUFFER_H
#define SD_RING_BUFFER_H
#include <Arduino.h>
#include <string.h>

// Byte ring buffer over caller-provided storage (place storage in DMAMEM).
// Single-producer (control loop) / single-consumer (drain) in the cooperative superloop; NOT ISR-safe.
// On overflow, drops oldest bytes to make room and counts them so the logger can emit a gap marker.
class SdRingBuffer
{
public:
    void init(uint8_t* storage, size_t capacity)
    { _buf = storage; _cap = capacity; _head = 0; _tail = 0; _count = 0; _dropped = 0; }

    size_t   capacity() const { return _cap; }
    size_t   size() const     { return _count; }
    size_t   space() const    { return _cap - _count; }
    uint32_t dropped() const  { return _dropped; }
    void     clear_dropped()  { _dropped = 0; }

    // Append n bytes; if short on room, drop oldest bytes first (counted).
    void push(const uint8_t* data, size_t n)
    {
        if (_cap == 0 || n == 0) return;
        if (n >= _cap) {                       // keep only the last _cap bytes
            _dropped += (uint32_t)(_count + (n - _cap));
            _head = _tail = _count = 0;
            data += (n - _cap); n = _cap;
        } else if (n > space()) {
            _drop(n - space());
        }
        size_t first = _cap - _head; if (first > n) first = n;
        memcpy(_buf + _head, data, first);
        if (n > first) memcpy(_buf, data + first, n - first);
        _head = (_head + n) % _cap;
        _count += n;
    }

    // Point to up to the largest contiguous readable run at the tail (no wrap). Returns its length.
    size_t peek(const uint8_t** out) const
    {
        if (_count == 0) { *out = _buf; return 0; }
        *out = _buf + _tail;
        size_t contig = _cap - _tail;
        return (contig < _count) ? contig : _count;
    }

    void consume(size_t n)
    {
        if (n > _count) n = _count;
        _tail = (_tail + n) % _cap;
        _count -= n;
    }

private:
    void _drop(size_t n)
    {
        if (n > _count) n = _count;
        _tail = (_tail + n) % _cap;
        _count -= n;
        _dropped += (uint32_t)n;
    }
    uint8_t* _buf = nullptr;
    size_t   _cap = 0, _head = 0, _tail = 0, _count = 0;
    uint32_t _dropped = 0;
};
#endif
