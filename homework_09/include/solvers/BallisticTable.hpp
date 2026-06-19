#pragma once

#include <stdexcept>
#include <vector>
#include <array>
#include <cstddef>  // для std::size_t
#include <fstream>
#include <stdexcept>
#include <algorithm>  //for lower_bound

// #define TABLE_USE_FLOAT

#ifdef TABLE_USE_FLOAT
using TableValue = float;
#else
using TableValue = double;
#endif

/* Required:
 * завантажувати таблицю з файлу;
 * зберігати 5 осей: Z0, V0, m, d, l;
 * мати плоский std::vector<Result> data;
 */

// Результат в кожному вузлі сітки
struct Result {
  TableValue ffTime;  // час польоту
  TableValue hDist;   // горизонтальна дистанція
};

namespace {

// Індекс і коефіцієнт для одного виміру
struct Interp {
  int lo;           // нижній індекс в осі
  TableValue frac;  // коефіцієнт [0..1]
};

// find closest lower node and ration for an axis
Interp findInterp(TableValue val, const std::vector<TableValue>& axis)
{
  if (val <= axis.front()) {  // clamp on the lowest point
    return {0, 0.0f};
  }

  if (val >= axis.back()) {  // clamp on the highest point
    return {(int)axis.size() - 2, 1.0f};
  }

  auto it = std::lower_bound(axis.begin(), axis.end(), val);

  int upper = std::distance(axis.begin(), it);  //(int)(it - axis.begin()) - 1;
  int lower = upper - 1;

  const TableValue x0 = axis[lower];
  const TableValue x1 = axis[upper];
  if (x1 == x0) {
    throw std::runtime_error("Axis contains duplicate points");
  }

  float frac = (val - x0) / (x1 - x0);
  return {lower, frac};
}

};  // namespace

struct BallisticTable {
  // 5 осей — кожна зі своїм набором вузлів (нерівномірний крок)
  /*
      1. Кожна вісь (axisZ0, axisV0, ...) — відсортований вектор значень з довільним кроком
      2. Розмір масиву data = добуток розмірів усіх осей
      3. Доступ до вузла — через at(iz, iv, im, id, il), де індекси відповідають позиціям на осях
      4. Для запиту між вузлами — багатовимірна лінійна інтерполяція по 5 осях
  */
  /*
    Мінімальні тести, які варто зробити:
          lookup точно у вузлі таблиці повертає значення вузла;
          lookup посередині між двома значеннями робить правильний lerp;
          значення нижче/вище осі clamp-иться;
          TableSolver підключається через фабрику.
  */
  // Плоский масив розміром |Z0| * |V0| * |M| * |D| * |L| => 5 dimensions
  static constexpr size_t kDims = 5;
  std::vector<Result> data;
  std::array<size_t, kDims> shape{};                // first row in the file
  std::array<std::vector<TableValue>, kDims> axes;  // next 5 rows in the file
  std::array<size_t, kDims> strides{};              // calculated

  void makeStrides()
  {
    size_t stride = 1;

    for (size_t dim = kDims; dim-- > 0;) {
      strides[dim] = stride;
      stride *= shape[dim];
    }
  }

  size_t flatInd(const std::array<size_t, kDims>& indices) const
  {
    size_t index = 0;

    for (size_t dim = 0; dim < kDims; ++dim) {
      if (indices[dim] >= shape[dim]) {
        throw std::runtime_error("Index is out of range");
      }

      index += indices[dim] * strides[dim];
    }

    return index;
  }

  // Завантаження з текстового файлу
  bool load(const char* path)
  {
    std::ifstream f(path);
    if (!f.is_open()) {
      throw std::runtime_error("Cannot open ballistic table file");  // return false;
    }

    // read sizes of axises
    for (size_t dim = 0; dim < kDims; ++dim) {
      size_t val;
      f >> val;

      if (val < 2) {
        throw std::runtime_error("Axis must contain minimum 2 points");
      }

      shape[dim] = val;
      axes[dim].resize(val);
    }

    // read axis points by axis
    for (auto& axis : axes) {
      for (auto& val : axis) {
        f >> val;
      }
    }

    makeStrides();
    size_t total = strides[0] * shape[0];  //(size_t)nZ * nV * nM * nD * nL;
    data.resize(total);

    // Порядок: Z0 → V0 → m → d → l (зовнішній → внутрішній)
    for (size_t i = 0; i < total; i++)
      f >> data[i].ffTime >> data[i].hDist;

    return f.good();  // all necessary data read without errors(eof, fail, bad bits not set)
  }

  auto lookup(const std::array<TableValue, kDims>& query) const -> Result
  {
    // find closest nodes
    std::array<Interp, kDims> intervals;
    for (size_t dim = 0; dim < kDims; ++dim) {
      intervals[dim] = findInterp(query[dim], axes[dim]);
    }

    constexpr std::size_t kCornerCount = std::size_t{1} << kDims;  // 1 shr 5 - 32 вершини гіперкуба
    Result result{0.0, 0.0};

    for (std::size_t num = 0; num < kCornerCount; ++num) {  // by every corner of 5D cell
      std::array<std::size_t, kDims> cornerIndices{};       // 5 indices of the corner num
      TableValue weight = TableValue{1};

      for (std::size_t dim = 0; dim < kDims; ++dim) {
        const bool useUpper = ((num >> dim) & std::size_t{1}) != 0;

        if (useUpper) {
          cornerIndices[dim] = intervals[dim].lo + 1;
          weight *= intervals[dim].frac;
        }
        else {
          cornerIndices[dim] = intervals[dim].lo;
          weight *= TableValue{1} - intervals[dim].frac;
        }
      }

      result.ffTime += weight * data[flatInd(cornerIndices)].ffTime;
      result.hDist += weight * data[flatInd(cornerIndices)].hDist;
    }

    return Result{result};
  }

  /*
      // 2^5 = 32 вершини гіперкуба
      // Згортаємо: 32 → 16 → 8 → 4 → 2 → 1
      // l: 32 → 16
  // Лінійна інтерполяція для Result (обидва поля паралельно)
  Result lerp(const Result& a, const Result& b, TableValue frac)
  {
    return {a.ffTime + (b.ffTime - a.ffTime) * frac, a.hDist + (b.hDist - a.hDist) * frac};
  }
  auto lookup(float z0, float v0, float m, float d, float l) const -> Result
    {
      Interp iz = findInterp(z0, axisZ0);
      Interp iv = findInterp(v0, axisV0);
      Interp im = findInterp(m, axisM);
      Interp id = findInterp(d, axisD);
      Interp il = findInterp(l, axisL);

      // 2^5 = 32 вершини гіперкуба
      // Згортаємо: 32 → 16 → 8 → 4 → 2 → 1
      // l: 32 → 16
      Result v[16];
      for (int a = 0; a < 2; a++)
        for (int b = 0; b < 2; b++)
          for (int c = 0; c < 2; c++)
            for (int e = 0; e < 2; e++) {
              auto& lo = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo);
              auto& hi = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo + 1);
              v[a * 8 + b * 4 + c * 2 + e] = lerp(lo, hi, il.frac);
            }

      // d: 16 → 8
      Result w[8];
      for (int a = 0; a < 2; a++)
        for (int b = 0; b < 2; b++)
          for (int c = 0; c < 2; c++) {
            w[a * 4 + b * 2 + c] = lerp(v[a * 8 + b * 4 + c * 2], v[a * 8 + b * 4 + c * 2 + 1], id.frac);
          }

      // m: 8 → 4
      Result u[4];
      for (int a = 0; a < 2; a++)
        for (int b = 0; b < 2; b++) {
          u[a * 2 + b] = lerp(w[a * 4 + b * 2], w[a * 4 + b * 2 + 1], im.frac);
        }

      // V0: 4 → 2
      Result s[2];
      for (int a = 0; a < 2; a++) {
        s[a] = lerp(u[a * 2], u[a * 2 + 1], iv.frac);
      }

      // Z0: 2 → 1
      return lerp(s[0], s[1], iz.frac);
    } */
};
