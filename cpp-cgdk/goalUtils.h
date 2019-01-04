#pragma once

namespace goals
{
    struct Never  { bool operator()() const { return false; } };
    struct Always { bool operator()() const { return true; } };
}


