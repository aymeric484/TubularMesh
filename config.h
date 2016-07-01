#ifndef CONFIG_H
#define CONFIG_H


#define Pi  3.14159265359
#define DISTANCE_MIN 7.0
#define COURBURE_MAX 0.5 // diverge en dessous de 0.0004 => probablement des NaN dans les calculs de courbures => ne viendrait pas de l'algo

#define TYPE_PRIMITIVE 8
#define MASK_SUBDIV_RAY 0.5 // doit être compris entre 0 et 1 exclus ; Plus on est proche de 1, plus la subdivision sera brutale
// avec a = MASK_SUBDIV_RAY, et "n" le nombre d'appuis sur "C". On a : épaisseur du bord = ((1-a) ^ n) * Rayon
// On a d'étranges résultats si cette valeurs est < 0.5 => pas idéale du tout pour l'étude du bord car pas l'épaisseur n'est pas décroissante d'une tranche à l'autre






#endif // CONFIG_H
