#pragma once
namespace SemanticSLAM {
struct SemanticPoint {
  int u;    // question: 4个参数含义
  int v;
  float x;
  float y;
  float prob;
  SemanticPoint(int _u, int _v, float _x, float _y, float _prob)
      : u(_u), v(_v), x(_x), y(_y), prob(_prob) {}
  SemanticPoint() : u(0), v(0), x(0), y(0), prob(0) {}
};
}
