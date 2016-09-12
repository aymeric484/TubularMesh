#ifndef BRANCHTOPO_H
#define BRANCHTOPO_H

#include <cgogn/core/basic/dart.h>
#include "coucheconcentrique.h"
using Dart = cgogn::Dart;

class BranchTopo
{
public:
    BranchTopo();
    std::vector<Dart> volume_control_;
    std::vector<CoucheConcentrique> subdivised_volume_control_;
};

#endif // BRANCHTOPO_H
