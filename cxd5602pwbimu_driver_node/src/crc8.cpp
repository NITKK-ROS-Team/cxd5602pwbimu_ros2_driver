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

#include "cxd5602pwbimu_driver_node/imu_class.hpp"

namespace cxd5602pwbimu_driver_node
{

CRC8::CRC8(
  uint8_t polynome,
  uint8_t initial,
  uint8_t xorOut,
  bool reverseIn,
  bool reverseOut)
: _polynome(polynome),
  _initial(initial),
  _xorOut(xorOut),
  _reverseIn(reverseIn),
  _reverseOut(reverseOut),
  _crc(initial),
  _count(0u)
{}

void CRC8::reset(
  uint8_t polynome,
  uint8_t initial,
  uint8_t xorOut,
  bool reverseIn,
  bool reverseOut)
{
  _polynome = polynome;
  _initial = initial;
  _xorOut = xorOut;
  _reverseIn = reverseIn;
  _reverseOut = reverseOut;
  restart();
}

void CRC8::restart()
{
  _crc = _initial;
  _count = 0u;
}

uint8_t CRC8::calc() const
{
  uint8_t rv = _crc;
  if (_reverseOut) {rv = reverse8bits(rv);}
  rv ^= _xorOut;
  return rv;
}

crc_size_t CRC8::count() const
{
  return _count;
}

void CRC8::add(uint8_t value)
{
  _count++;
  _add(value);
}

void CRC8::add(const uint8_t * array, crc_size_t length)
{
  _count += length;
  while (length--) {
    _add(*array++);
  }
}

void CRC8::add(const uint8_t * array, crc_size_t length, crc_size_t yieldPeriod)
{
  _count += length;
  crc_size_t period = yieldPeriod;
  while (length--) {
    _add(*array++);
    if (--period == 0) {
      period = yieldPeriod;
    }
  }
}

void CRC8::_add(uint8_t value)
{
  if (_reverseIn) {value = reverse8bits(value);}
  _crc ^= value;
  for (uint8_t i = 8; i; i--) {
    if (_crc & (1 << 7)) {
      _crc <<= 1;
      _crc ^= _polynome;
    } else {
      _crc <<= 1;
    }
  }
}

uint8_t CRC8::getCRC() const
{
  return calc();
}

}
