// Checked ASCII OBJ decoding. No memoized validation and no geometry reduction.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <charconv>
#include <cmath>
#include <cstring>
#include <fstream>
#include <string_view>
#include <vector>

namespace py = pybind11;
namespace {
std::string_view Token(std::string_view &line) {
  auto first = line.find_first_not_of(" \t\r");
  if (first == line.npos) { line = {}; return {}; }
  line.remove_prefix(first);
  auto end = line.find_first_of(" \t\r");
  auto value = line.substr(0, end);
  line = end == line.npos ? std::string_view{} : line.substr(end);
  return value;
}

template<class T> T Number(std::string_view token) {
  if (!token.empty() && token.front() == '+') token.remove_prefix(1);
  if (token.empty()) throw std::invalid_argument("empty OBJ token");
  T value{};
  auto result = std::from_chars(token.data(), token.data() + token.size(), value);
  if (result.ec != std::errc{} || result.ptr != token.data() + token.size())
    throw std::invalid_argument("invalid OBJ numeric token");
  return value;
}

struct Obj {
  std::vector<double> vertices, uv;
  std::vector<int64_t> faces;
};

Obj Read(const std::string &path, bool with_uv) {
  std::ifstream file(path);
  if (!file) throw std::invalid_argument("cannot open OBJ");
  Obj out;
  std::string storage;
  while (std::getline(file, storage)) {
    std::string_view line(storage);
    auto tag = Token(line);
    if (tag == "v" || (with_uv && tag == "vt")) {
      auto &values = tag == "v" ? out.vertices : out.uv;
      const int count = tag == "v" ? 3 : 2;
      for (int i = 0; i < count; ++i) {
        double value = Number<double>(Token(line));
        if (!std::isfinite(value)) throw std::invalid_argument("nonfinite OBJ coordinates");
        values.push_back(value);
      }
    } else if (tag == "f") {
      for (int i = 0; i < 3; ++i) {
        auto corner = Token(line);
        auto slash = corner.find('/');
        auto vertex = corner.substr(0, slash);
        auto index = Number<int64_t>(vertex);
        if (index < 1) throw std::invalid_argument("invalid collision proxy mesh");
        out.faces.push_back(index - 1);
        if (with_uv) {
          if (slash == corner.npos) throw std::invalid_argument("display mesh face UV indices mismatch");
          auto rest = corner.substr(slash + 1);
          auto second = rest.find('/');
          auto texture = rest.substr(0, second);
          if (texture != vertex) throw std::invalid_argument("display mesh face UV indices mismatch");
          if (second != rest.npos) Number<int64_t>(rest.substr(second + 1));
        }
      }
      if (!Token(line).empty()) throw std::invalid_argument("collision proxy requires triangular surface");
    }
  }
  if (file.bad()) throw std::invalid_argument("cannot read OBJ");
  if (out.vertices.empty() || out.faces.empty()) throw std::invalid_argument("invalid collision proxy mesh");
  for (auto i : out.faces)
    if (size_t(i) >= out.vertices.size() / 3) throw std::invalid_argument("invalid collision proxy mesh");
  return out;
}

template<class T> py::array_t<T> Array(const std::vector<T> &data, int columns) {
  py::array_t<T> out({py::ssize_t(data.size() / columns), py::ssize_t(columns)});
  if (!data.empty()) std::memcpy(out.mutable_data(), data.data(), data.size() * sizeof(T));
  return out;
}
}  // namespace

PYBIND11_MODULE(_obj_arrays, m) {
  m.def("read_obj", [](const std::string &path, bool with_uv) {
    Obj obj;
    { py::gil_scoped_release unlock; obj = Read(path, with_uv); }
    py::object uv = with_uv ? py::object(Array(obj.uv, 2)) : py::none();
    return py::make_tuple(Array(obj.vertices, 3), Array(obj.faces, 3), uv);
  }, py::arg("path"), py::arg("with_uv") = false);
}
