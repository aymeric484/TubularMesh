#include "coucheconcentrique.h"

CoucheConcentrique::CoucheConcentrique(unsigned int num_couche, std::vector<Dart> volumes)
{
    etage_ = num_couche;
    indic_volumes_ = volumes;
}
