#include "RRTRoot.h"

RRTNode::RRTNode(const int& x, const int& y, RRTNode* parent) {
    this->pos = Vec2i::Create(x,y);
    this->parent = parent;
}

RRTNode::RRTNode(const Vec2i& pos, RRTNode* parent) {
    this->pos = pos;
    this->parent = parent;
}

RRTNode::~RRTNode() {
    this->parent = nullptr;
}