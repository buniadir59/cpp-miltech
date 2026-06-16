

struct BallisticTable {
  bool load(const std::string& path);
  Result lookup(float z0, float v0, float m, float d, float l) const;
};