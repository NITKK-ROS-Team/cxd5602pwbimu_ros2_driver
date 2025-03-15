/*
* Copyright (c) 2025 NITK.K ROS-Team
*
* SPDX-License-Identifier: Apache-2.0
*/

// ref: https://github.com/RobTillaart/CRC
// Copyright (c) 2021-2024 Rob Tillaart

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef ____CXD5602PWBIMU_CRC8_HPP__
#define ____CXD5602PWBIMU_CRC8_HPP__

#include <cstdint>

typedef uint32_t crc_size_t;
#define CRC8_POLYNOME               0x07
#define CRC8_INITIAL                0x00
#define CRC8_XOR_OUT                0x00
#define CRC8_REV_IN                 false
#define CRC8_REV_OUT                false


namespace cxd5602pwbimu_node
{

class CRC8
{
public:
  CRC8(
    uint8_t polynome = CRC8_POLYNOME,
    uint8_t initial = CRC8_INITIAL,
    uint8_t xorOut = CRC8_XOR_OUT,
    bool reverseIn = CRC8_REV_IN,
    bool reverseOut = CRC8_REV_OUT);

  void reset(
    uint8_t polynome = CRC8_POLYNOME,
    uint8_t initial = CRC8_INITIAL,
    uint8_t xorOut = CRC8_XOR_OUT,
    bool reverseIn = CRC8_REV_IN,
    bool reverseOut = CRC8_REV_OUT);

  void restart();
  uint8_t calc() const;
  crc_size_t count() const;

  void add(uint8_t);

  void add(const uint8_t *, crc_size_t);

  void add(const uint8_t *, crc_size_t, crc_size_t);

  void setPolynome(uint8_t polynome) {_polynome = polynome;}

  void setInitial(uint8_t initial) {_initial = initial;}

  void setXorOut(uint8_t xorOut) {_xorOut = xorOut;}

  void setReverseIn(bool reverseIn) {_reverseIn = reverseIn;}

  void setReverseOut(bool reverseOut) {_reverseOut = reverseOut;}

  uint8_t getPolynome() const {return _polynome;}

  uint8_t getInitial() const {return _initial;}

  uint8_t getXorOut() const {return _xorOut;}

  bool getReverseIn() const {return _reverseIn;}

  bool getReverseOut() const {return _reverseOut;}

  [[deprecated("Use calc() instead")]]
  uint8_t getCRC() const;

  [[deprecated("Use setInitial() instead")]]
  void setStartXOR(uint8_t initial) {_initial = initial;}

  [[deprecated("Use setXorOut() instead")]]
  void setEndXOR(uint8_t xorOut) {_xorOut = xorOut;}

  [[deprecated("Use getInitial() instead")]]
  uint8_t getStartXOR() const {return _initial;}

  [[deprecated("Use getXorOut() instead")]]
  uint8_t getEndXOR() const {return _xorOut;}

  [[deprecated("Use add() with yieldPeriod instead")]]
  void enableYield() const {}

  [[deprecated("Use add() without yieldPeriod instead")]]
  void disableYield() const {}
  static uint8_t reverse8bits(uint8_t value)
  {
    value = ((value & 0xF0) >> 4) | ((value & 0x0F) << 4);
    value = ((value & 0xCC) >> 2) | ((value & 0x33) << 2);
    value = ((value & 0xAA) >> 1) | ((value & 0x55) << 1);
    return value;
  }

private:
  void _add(uint8_t);

  uint8_t _polynome;
  uint8_t _initial;
  uint8_t _xorOut;
  bool _reverseIn;
  bool _reverseOut;
  uint8_t _crc;
  crc_size_t _count;
};


}  // namespace cxd5602pwbimu_node

#endif  // ____CXD5602PWBIMU_CRC8_HPP__
