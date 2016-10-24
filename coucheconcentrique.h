#ifndef COUCHECONCENTRIQUE_H
#define COUCHECONCENTRIQUE_H


#include <math.h>

#include <cgogn/core/cmap/cmap3.h>
#include <cgogn/core/basic/dart.h>

#include <cgogn/io/map_import.h>
#include <time.h>
#include "config.h"

using Dart = cgogn::Dart;

class CoucheConcentrique
{
public:


    CoucheConcentrique(unsigned int, std::vector<Dart>);

    //
    //
    std::vector<Dart> indic_volumes_;
    unsigned int etage_;



private:




};

#endif // COUCHECONCENTRIQUE_H
