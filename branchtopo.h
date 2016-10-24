#ifndef BRANCHTOPO_H
#define BRANCHTOPO_H

#include <cgogn/core/basic/dart.h>
#include "coucheconcentrique.h"
using Dart = cgogn::Dart;

class BranchTopo
{
public:
    BranchTopo();

    //
    // Ces darts représente nos prismes à base triangulaire après l extrusion
    std::vector<Dart> volume_control_;

    //
    // On utilise ces couches concentrique ppour avoir accès aux darts obtenu par la subdivision des couches concentrique
    std::vector<CoucheConcentrique> subdivised_volume_control_;
};

#endif // BRANCHTOPO_H
