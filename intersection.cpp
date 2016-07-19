#include "intersection.h"

Intersection::Intersection(int& ind)
{
    indicateur_ = ind;
}

void Intersection::ComputeConnectivity()
{

    std::vector<Vec3> points_proj;

    //
    // Trouver le plus grand rayon entre le centre et tout les points

    // Pour pas s'embêter, on pourrait avoir d_max trés trés grand, comme la bb_ par exemple, ou 10^3 fois le rayon de centre_
    double d_max;
    Vec3 Icentre = centre_.head<3>();

    for(Vec3 point_courant : contours_)
    {
        Vec3 res = Icentre - point_courant;
        double d = res.norm();
        if(d > d_max)
            d_max = d;
    }

    //
    // Projeter tout ces points sur une sphère
    //

    for(Vec3 point_courant : contours_)
    {
        Vec3 res = point_courant - Icentre;
        double distance = res.norm();
        double ratio = d_max/distance;
        points_proj.push_back(Icentre + res*ratio);
    }

    //
    // Test de calcul pour voir si l'on est bien sur la sphère
    /*
    Vec3 sum = {0.0, 0.0, 0.0};
    for(Vec3 pts : points_proj)
    {
        Vec3 res = Icentre - pts;
        std::cout << "la distance est : " << res.norm() << std::endl;
    }*/


    //
    //
    // Algorithme de creation du volume convexe en utilisant des faces triangulaires
    //
    //

    std::vector<TriangleGeo> volume_courant; // renvoi des indices correspondant à des points de contours_

    int nb_pts = contours_.size();

    //
    // Commencer par un tétraèdre

    Vec3 p1 = contours_[0];
    Vec3 p2 = contours_[1];
    Vec3 p3 = contours_[nb_pts - 2];
    Vec3 p4 = contours_[nb_pts - 1];
    TriangleGeo T1(p1, p2, p3, 0, 1, nb_pts - 2);
    TriangleGeo T2(p1, p2, p4, 0, 1, nb_pts - 1);
    TriangleGeo T3(p1, p3, p4, 0, nb_pts - 2, nb_pts - 1);
    TriangleGeo T4(p2, p3, p4, 1, nb_pts - 2, nb_pts - 1);
    T1.OrientedNormal(p4);
    T2.OrientedNormal(p3);
    T3.OrientedNormal(p2);
    T4.OrientedNormal(p1);
    volume_courant.push_back(T1);
    volume_courant.push_back(T2);
    volume_courant.push_back(T3);
    volume_courant.push_back(T4);

    //
    // Boucle while pour modifier le volume

    bool check = false;
    bool face_change = false;
    int incre = 0;
    std::vector<TriangleGeo>::iterator it = volume_courant.begin();
    while(!check)
    {
        TriangleGeo T_courant = *it;
        //std::cout << "  indice 1 :" << T_courant.connectivity_[0] << "  indice 2 :" << T_courant.connectivity_[1] << "  indice 3 :" << T_courant.connectivity_[2] << std::endl;
        std::cout << "sommets :  x = " << T_courant.connectivity_[0] <<  "  y = " << T_courant.connectivity_[1] <<  "  z = " << T_courant.connectivity_[2] <<  std::endl;
        //sleep(1.9);

        for(int i = 0; i < contours_.size(); i++)
        {
            bool point_volume = false;

            // trois options :
            // 1--écarter les points qui correspondent à la condition "si le point appartient déjà au volume" => relativement lourd à tester
            // 2--ou bien "si il a appartenu au volume" => pas forcement correcte
            // 3--ou bien "si il appartient à la face triangulaire" => plus facile car comparer i avec les 3 indices du triangle => reste pas optimisé

            // Attention : problèmes avec les points coplanaires ! => à tester avant avec des triangles dans un premier tps
            // Dans un deuxième temp, imposer un seuil à dépasser pour le produit scalaire pour ne pas complexifier le volume pour rien

            // Option 1
            for(TriangleGeo T_volume : volume_courant)
            {
                for(int k = 0; k < 3; k++)
                {
                    if(T_volume.connectivity_[k] == i)
                    {
                        point_volume = true;
                        break;
                    }
                }
            }



            // On ne veut pas que le point à tester appartienne au triangle courant => option 3
            //if(i != T_courant.connectivity_[0] && i != T_courant.connectivity_[1] && i != T_courant.connectivity_[2])
            // Option 1 (suite)
            if(!point_volume)
            {
                // reli un point du triangle au point courant
                Vec3 vec_courant = contours_[i] - T_courant.sommets_[0];//

                vec_courant.normalize();// pour pas qu'il soit trop grand

                //std::cout << "vec_courant :  x = " << vec_courant[0] << "  y = " << vec_courant[1] << "  y = " << vec_courant[2] <<  std::endl;
                //std::cout << "le produit scalaire vaut : " << T_courant.normal_.dot(vec_courant) << std::endl;



                // Si le produit scalaire est négatif, alors c'est que le point est en dehors du polyèdre => modifier la liste avec des triangle contenant le nouveau point
                if(T_courant.normal_.dot(vec_courant) < -0.001)
                {


                    //
                    // Il faut créer un nouveau tétraèdre sans la face courante et réinitialiser l'itérateur

                    // Création des triangles en incluant le nouveau point
                    TriangleGeo T5(T_courant.sommets_[0], T_courant.sommets_[1], contours_[i], T_courant.connectivity_[0], T_courant.connectivity_[1], i);
                    TriangleGeo T6(T_courant.sommets_[0], T_courant.sommets_[2], contours_[i], T_courant.connectivity_[0], T_courant.connectivity_[2], i);
                    TriangleGeo T7(T_courant.sommets_[2], T_courant.sommets_[1], contours_[i], T_courant.connectivity_[2], T_courant.connectivity_[1], i);
                    // Réparation des normales à l'aide d'un point intérieur au tétraèdre
                    T5.OrientedNormal(T_courant.sommets_[2]);
                    T6.OrientedNormal(T_courant.sommets_[1]);
                    T7.OrientedNormal(T_courant.sommets_[0]);

                    *it = T5;
                    // std::cout << "  indice 1 :" << T5.connectivity_[0] << "  indice 2 :" << T5.connectivity_[1] << "  indice 3 :" << T5.connectivity_[2] << std::endl;
                    // pushback pour simplifier la tache mais utiliser insertion si pb pluss tard pour reconstituer le polyèdre
                    volume_courant.push_back(T6);
                    volume_courant.push_back(T7);

                    //std::cout << "taille de la liste : " << volume_courant.size() << std::endl;

                    // Nous permettra de ne pas incrementer l'itérateur à la prochaine itération
                    face_change = true;

                    // Sortir ensuite de la boucle for car on ne doit plus effectuer le test
                    break;
                }
            }
        }

        if(face_change)
        {
            // Dans tt les cas, réinitialiser l'itérateur ou au moins reculer de 1 pour optimiser ou pas en fait... à voir
            // pb possible généré par les pushback avec l'itérator qui n'irait pas jusqu'au bout
            face_change = false;
            it = volume_courant.begin() + incre;

        }
        else
        {
            std::cout << "Itération sur les faces du polyèdre : " << incre << std::endl;

            incre++;

            if(incre > volume_courant.size() - 2)
                check = true;
            else
                it++;
        }
    }

    /*
    // Stocker la connectivité dans un tableau stockant les indices des sommets 3 par 3 car notre volume ne comporte que des triangles

    // Le volume courant se stocke dans ce tableau et se fait modifier ainsi

    // Les faces doivent être orientées => cross product de 2 vecteurs coplanaires, puis pour le sens de la normale, faire le test point du volume-normale
    // si < 0, alors inverser le vecteur
    // sinon, on a la bonne normale orienté vers l'intérieure
    // Créer une classe triangle pour le calcul de la normale, la structure {3*Vec3}, et la structure {1,2,8} => des indices qui codent les points de notre nuage
    // Modifier l'ordre des Vec3 dans la structure selon le sens de la normale
    // la connectivité stocke un tableau de triangles
    // une fois qu'on a le sens et la direction de la normale, effectuer le test dot(normale,AM) avec M un point quelconque de notre nuage de point
    // Si dot > 0, alors on ignore le point et on passe au suivant
    // Sinon, on supprime notre triangle te on en créer 3 nouveaux reliant à chaque fois 2 points à
    */

    // Stockage de la connectivité finale en attribut

    faces_ = volume_courant;

    // Vérification du contenu de faces_ => les triangles formant le volume : verification de la connectivité ... toute les arêtes apparaisent 2 fois
    /*
    for(TriangleGeo T : faces_)
    {
        std::cout << "  T0: "  << T.connectivity_[0] << "  T1: " << T.connectivity_[1] << "  T2: " << T.connectivity_[2] << std::endl;

    }*/

}
