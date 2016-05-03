#include "branch.h"


Branch::Branch()
{
    Vec4 V40(0.0, 0.0, 0.0, 1.0);
    Vec4 V41(0.0, 0.0, 2.0, 2.0);
    Vec4 V42(0.0, 0.0, 3.0, 1.0);
    Vec4 V43(0.0, 0.0, 4.0, 1.0);
    Vec4 V44(0.5, 0.5, 5.0, 1.0);
    Vec4 V45(0.0, 0.5, 7.0, 1.0);

    this->articulations_.push_back(V40);
    this->articulations_.push_back(V41);
    this->articulations_.push_back(V42);
    this->articulations_.push_back(V43);
    this->articulations_.push_back(V44);
    this->articulations_.push_back(V45);

}
