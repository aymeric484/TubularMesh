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
    double d_max = 0;
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
    // Algorithme de création du volume convexe en utilisant des faces triangulaires
    //
    //

    std::vector<TriangleGeo> volume_courant; // renvoi des indices correspondant à des points de contours_

    int nb_pts = contours_.size();
    int nb_courant = 4;

    //
    // Commencer par un tétraèdre

    Vec3 p1 = points_proj[0];
    Vec3 p2 = points_proj[1];
    Vec3 p3 = points_proj[2];
    Vec3 p4 = points_proj[7];
    Vec3 Bcentre = (p1 + p2 + p3 + p4)/nb_courant;
    TriangleGeo T1(p1, p2, p3, 0, 1, 2, -1);
    TriangleGeo T2(p1, p2, p4, 0, 1, 3, -1);
    TriangleGeo T3(p1, p3, p4, 0, 2, 3, -1);
    TriangleGeo T4(p2, p3, p4, 1, 2, 3, -1);
    T1.OrientedNormal(Bcentre);
    T2.OrientedNormal(Bcentre);
    T3.OrientedNormal(Bcentre);
    T4.OrientedNormal(Bcentre);
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

            // Faire test plus long distance avec T_courant && contours


//
            // On ne veut pas que le point à tester appartienne au triangle courant => option 3
            //if(i != T_courant.connectivity_[0] && i != T_courant.connectivity_[1] && i != T_courant.connectivity_[2])

            // Option 1 (suite)
            if(!point_volume)
            {
                // reli un point du triangle au point courant
                Vec3 vec_courant = points_proj[i] - T_courant.sommets_[0];

                //vec_courant.normalize();// pour pas qu'il soit trop grand

                //std::cout << "vec_courant :  x = " << vec_courant[0] << "  y = " << vec_courant[1] << "  y = " << vec_courant[2] <<  std::endl;
                //std::cout << "le produit scalaire vaut : " << T_courant.normal_.dot(vec_courant) << std::endl;



                // Si le produit scalaire est négatif, alors c'est que le point est en dehors du polyèdre => modifier la liste avec des triangle contenant le nouveau point
                if(T_courant.normal_.dot(vec_courant) < -0.001)
                {

                    //
                    // Il faut créer un nouveau tétraèdre sans la face courante et réinitialiser l'itérateur

                    // Création des triangles en incluant le nouveau point
                    TriangleGeo T5(T_courant.sommets_[0], T_courant.sommets_[1], points_proj[i], T_courant.connectivity_[0], T_courant.connectivity_[1], i, -1);
                    TriangleGeo T6(T_courant.sommets_[0], T_courant.sommets_[2], points_proj[i], T_courant.connectivity_[0], T_courant.connectivity_[2], i, -1);
                    TriangleGeo T7(T_courant.sommets_[2], T_courant.sommets_[1], points_proj[i], T_courant.connectivity_[2], T_courant.connectivity_[1], i, -1);

                    // Réparation des normales à l'aide d'un point intérieur au polyèdre courant (tétraèdre à la première itération) => on choisit le barycentre
                    Bcentre = (Bcentre*nb_courant + points_proj[i])/(nb_courant + 1);
                    nb_courant++;
                    T5.OrientedNormal(Bcentre);
                    T6.OrientedNormal(Bcentre);
                    T7.OrientedNormal(Bcentre);

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
    // Sinon, on supprime notre triangle et on en créer 3 nouveaux reliant à chaque fois 2 points à
    */

    // Stockage de la connectivité finale en attribut

    int count = 0;

    for(TriangleGeo T_actu : volume_courant)
    {

        int a = T_actu.connectivity_[0];
        int b = T_actu.connectivity_[1];
        int c = T_actu.connectivity_[2];

        //std::cout << "cond1 " << a/TYPE_PRIMITIVE << "   cond2 " << b/TYPE_PRIMITIVE << "   cond3 " << c/TYPE_PRIMITIVE << std::endl;
        std::cout << "cond1 " << a << "   cond2 " << b << "   cond3 " << c << std::endl;

        //
        // Ici, trouver de quelle branche provient la face à l'aide de a,b,c qui nous donnent les indices des sommets de contours_
        for(int j : branches_incidentes_)
        {
            //std::cout << "branche j "<< j << std::endl;

            bool cond1 = a/TYPE_PRIMITIVE == j;
            bool cond2 = b/TYPE_PRIMITIVE == j;
            bool cond3 = c/TYPE_PRIMITIVE == j;

            if( cond1 && cond2 && cond3 )
                T_actu.num_branch_ = j;
        }


        //
        // Boucle nous permettant de vérifier qui sont les voisins => remplissage des attribut ind1,2,3
        for(int i = 0 ; i < volume_courant.size(); i++)
        {

            // Ecarter la condition où l'on est sur le triangle courant
            if(count != i)
            {
                TriangleGeo T_comp = volume_courant[i];

                if((a == T_comp.connectivity_[0] && b == T_comp.connectivity_[2]) || (a == T_comp.connectivity_[1] && b == T_comp.connectivity_[0]) || (a == T_comp.connectivity_[2] && b == T_comp.connectivity_[1]))
                    T_actu.ind1_ = i;
                if((b == T_comp.connectivity_[0] && c == T_comp.connectivity_[2]) || (b == T_comp.connectivity_[1] && c == T_comp.connectivity_[0]) || (b == T_comp.connectivity_[2] && c == T_comp.connectivity_[1]))
                    T_actu.ind2_ = i;
                if((c == T_comp.connectivity_[0] && a == T_comp.connectivity_[2]) || (c == T_comp.connectivity_[1] && a == T_comp.connectivity_[0]) || (c == T_comp.connectivity_[2] && a == T_comp.connectivity_[1]))
                    T_actu.ind3_ = i;
            }
        }

        //
        // On remplace le triangle courant et on passe au suivant
        volume_courant[count] = T_actu;
        count++;
    }
    faces_ = volume_courant;



    // Vérification du contenu de faces_ => les triangles formant le volume : verification de la connectivité ... toute les arêtes apparaisent 2 fois

    for(TriangleGeo T : faces_)
    {
        //std::cout << "  T0: "  << T.connectivity_[0] << "  T1: " << T.connectivity_[1] << "  T2: " << T.connectivity_[2] << std::endl;
        std::cout << "  T0: "  << T.num_branch_ << std::endl;

    }

}

void Intersection::ComputeConnectivity2()
{

    //
    //
    // déplacer les points de l'intersection sur une sphère de référence
    //
    //

    std::vector<Vec3> points_proj;

    //
    // Trouver le plus grand rayon entre le centre et tout les points
    //

    // Pour pas s'embêter, on pourrait avoir d_max trés trés grand, comme la bb_ par exemple, ou 10^3 fois le rayon de centre_
    double d_max = 0;
    Vec3 Icentre = centre_.head<3>();

    for(Vec3 point_courant : contours_)
    {
        Vec3 res = Icentre - point_courant;
        double d = res.norm();
        if(d > d_max)
            d_max = d;
    }


    //
    // Projeter tout ces points sur une sphère de rayon dmax
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
    //

    /*
    Vec3 sum = {0.0, 0.0, 0.0};
    for(Vec3 pts : points_proj)
    {
        Vec3 res = Icentre - pts;
        std::cout << "la distance est : " << res.norm() << std::endl;
    }*/



    //
    //
    // Algorithme de création du volume convexe en utilisant des faces triangulaires
    //
    //


    std::vector<TriangleGeo> volume_courant; // renvoi des indices correspondant à des points de contours_
    int nb_courant = 4;

    //
    // Commencer par un tétraèdre
    //

    //
    // Première face

    Vec3 p1 = points_proj[0];
    Vec3 p2 = points_proj[1];
    Vec3 p3 = points_proj[2];
    TriangleGeo T1(p1, p2, p3, 0, 1, 2, -1);

    //
    // trouver le point le plus éloigné

    double dist_ref = 0;
    int ind_ref;
    for(int a = 3; a < points_proj.size(); a++)
    {
        Vec3 resultante = points_proj[a] - p1;
        double dist = resultante.dot(T1.normal_)*resultante.dot(T1.normal_);
        if( dist > dist_ref)
        {
            dist_ref = dist;
            ind_ref = a;
        }
    }

    std::cout << "premier sommet : " << ind_ref << std::endl;
    Vec3 p4 = points_proj[ind_ref];
    T1.OrientedNormal(p4);
    volume_courant.push_back(T1);


    //
    // Fermer le tétraèdre avec 3 autre faces

    // Calcul d'un point interieur (pas forcement nécéssaire)
    //Vec3 Bcentre = (p1 + p2 + p3 + p4)/nb_courant;
    // Construction des trois autre triangles du tétraèdre
    TriangleGeo T2(p1, p2, p4, 0, 1, ind_ref, -1);
    TriangleGeo T3(p1, p3, p4, 0, 2, ind_ref, -1);
    TriangleGeo T4(p2, p3, p4, 1, 2, ind_ref, -1);
    // Orientation des faces vers le centre du tétraèdre (vers l'intérieur)
    T2.OrientedNormal(p3);
    T3.OrientedNormal(p2);
    T4.OrientedNormal(p1);
    // Stockage des faces dans un buffer volume courant
    volume_courant.push_back(T2);
    volume_courant.push_back(T3);
    volume_courant.push_back(T4);

    //
    // Boucle while pour modifier le volume en lui ajoutant des faces
    //

    bool check = false;
    bool face_change = false;
    int incre = 0;
    std::vector<TriangleGeo>::iterator it = volume_courant.begin();

    while(!check)
    {

        //
        // On s'intéresse à une des faces du volume

        TriangleGeo T_courant = *it;
        int appartenance = -1;

        if((int(T_courant.connectivity_[0]/TYPE_PRIMITIVE) == int(T_courant.connectivity_[1]/TYPE_PRIMITIVE)) || (int(T_courant.connectivity_[0]/TYPE_PRIMITIVE) == int(T_courant.connectivity_[2]/TYPE_PRIMITIVE)))
            appartenance = int(T_courant.connectivity_[0]/TYPE_PRIMITIVE);

        if(int(T_courant.connectivity_[1]/TYPE_PRIMITIVE) == int(T_courant.connectivity_[2]/TYPE_PRIMITIVE))
            appartenance = int(T_courant.connectivity_[1]/TYPE_PRIMITIVE);

        double ref_dist = 0;
        int ref_ind = -1;
        bool indic_modif_ref = false;
        int ref_prioritaire = -1; // Si l'on trouve un point compatible sur la même branche, alors il est obligatoirement prioritaire
        double dist_prioritaire = 0;
        bool lock = false; // une fois mis a true, ne se remet plus a false pour le restant de la boucle


        //
        // Pour tout les point, verifier qu'ils ne sont pas des points du volume et prendre le plus loin de la face parmis ces points ajouter ici condition  appartenance sommets-branche

        for(int i = 0; i < contours_.size(); i++)
        {
            // Nous donne l'appartenance du point au volume
            bool point_volume = false;
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

            // Conserve la plus petite valeur et stocke l'indice (plus petite => plus grande dans les scalaires négatifs)
            if(!point_volume)
            {
                double dist = T_courant.normal_.dot(points_proj[i]-T_courant.sommets_[0]);

                if(dist < 0)
                {
                    if(dist < ref_dist && lock == false)
                    {
                        ref_dist = dist;
                        ref_ind = i;
                        indic_modif_ref = true;
                    }
                    bool temp = appartenance == int(i/TYPE_PRIMITIVE);
                    std::cout << "sur la face : " <<  temp << std::endl;

                    if( dist < dist_prioritaire && appartenance == int(i/TYPE_PRIMITIVE))
                    {
                        std::cout << "condition même branche vérifiée" << std::endl;
                        dist_prioritaire = dist;
                        ref_dist = dist_prioritaire;
                        ref_ind = i;
                        lock = true;
                        indic_modif_ref = true;
                    }
                }
            }
        }


        //
        // Ajouter ce point au volume en supprimant la face courante si ref_dist à été modifié

        sleep(1);

        if(indic_modif_ref == true)
        {
            std::cout << "  ind0: "<< T_courant.connectivity_[0] << "  ind1: "<< T_courant.connectivity_[1] << "  ind2: "<< T_courant.connectivity_[2] << std::endl;
            std::cout << "nouveau point : " << ref_ind << std::endl;

            // Création des triangles en incluant le nouveau point
            TriangleGeo T5(T_courant.sommets_[0], T_courant.sommets_[1], points_proj[ref_ind], T_courant.connectivity_[0], T_courant.connectivity_[1], ref_ind, -1);
            TriangleGeo T6(T_courant.sommets_[0], points_proj[ref_ind], T_courant.sommets_[2], T_courant.connectivity_[0],  ref_ind, T_courant.connectivity_[2], -1);
            TriangleGeo T7(points_proj[ref_ind], T_courant.sommets_[1],T_courant.sommets_[2] ,  ref_ind, T_courant.connectivity_[1], T_courant.connectivity_[2], -1);

            // Réparation des normales à l'aide d'un point intérieur au polyèdre courant (tétraèdre à la première itération) => on choisit le barycentre
            //Bcentre = (Bcentre*nb_courant + points_proj[ref_ind])/(nb_courant + 1);
            nb_courant++;
            T5.OrientedNormal(T_courant.sommets_[2]);
            T6.OrientedNormal(T_courant.sommets_[1]);
            T7.OrientedNormal(T_courant.sommets_[0]);

            // On stocke les face dans le volume courant
            *it = T5;
            //volume_courant.push_back(T6);
            //volume_courant.push_back(T7);
            volume_courant.insert(volume_courant.begin() + incre + 1, T6);
            volume_courant.insert(volume_courant.begin() + incre + 2, T7);

            // Nous permettra de ne pas incrementer l'itérateur à la prochaine itération, comme ça on refait le test avec T5
            face_change = true;
        }



        //
        // Si il y a eu changement, on reste sur l'itérateur, sinon on incremente l'itérateur => on passe à la face suivante

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

            // Condition de sortie à tester, qd on a parcouru toute les faces du volume et qu'on ne trouve plus de quoi modifier
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
    // Sinon, on supprime notre triangle et on en créer 3 nouveaux reliant à chaque fois 2 points à
    */



    //
    //
    // Stockage de la connectivité finale en attribut
    //
    //


    int count = 0;

    for(TriangleGeo T_actu : volume_courant)
    {

        int a = T_actu.connectivity_[0];
        int b = T_actu.connectivity_[1];
        int c = T_actu.connectivity_[2];

        //std::cout << "cond1 " << a/TYPE_PRIMITIVE << "   cond2 " << b/TYPE_PRIMITIVE << "   cond3 " << c/TYPE_PRIMITIVE << std::endl;
        std::cout << "cond1 " << a << "   cond2 " << b << "   cond3 " << c << std::endl;

        //
        // Ici, trouver de quelle branche provient la face à l'aide de a,b,c qui nous donnent les indices des sommets de contours_
        for(int j : branches_incidentes_)
        {
            //std::cout << "branche j "<< j << std::endl;

            bool cond1 = a/TYPE_PRIMITIVE == j;
            bool cond2 = b/TYPE_PRIMITIVE == j;
            bool cond3 = c/TYPE_PRIMITIVE == j;

            if( cond1 && cond2 && cond3 )
                T_actu.num_branch_ = j;
        }


        //
        // Boucle nous permettant de vérifier qui sont les voisins => remplissage des attribut ind1,2,3
        for(int i = 0 ; i < volume_courant.size(); i++)
        {

            // Ecarter la condition où l'on est sur le triangle courant
            if(count != i)
            {
                TriangleGeo T_comp = volume_courant[i];

                if((a == T_comp.connectivity_[0] && b == T_comp.connectivity_[2]) || (a == T_comp.connectivity_[1] && b == T_comp.connectivity_[0]) || (a == T_comp.connectivity_[2] && b == T_comp.connectivity_[1]))
                    T_actu.ind1_ = i;
                if((b == T_comp.connectivity_[0] && c == T_comp.connectivity_[2]) || (b == T_comp.connectivity_[1] && c == T_comp.connectivity_[0]) || (b == T_comp.connectivity_[2] && c == T_comp.connectivity_[1]))
                    T_actu.ind2_ = i;
                if((c == T_comp.connectivity_[0] && a == T_comp.connectivity_[2]) || (c == T_comp.connectivity_[1] && a == T_comp.connectivity_[0]) || (c == T_comp.connectivity_[2] && a == T_comp.connectivity_[1]))
                    T_actu.ind3_ = i;
            }
        }

        //
        // On remplace le triangle courant et on passe au suivant
        volume_courant[count] = T_actu;
        count++;
    }
    faces_ = volume_courant;



    // Vérification du contenu de faces_ => les triangles formant le volume : verification de la connectivité ... toute les arêtes apparaisent 2 fois
    for(TriangleGeo T : faces_)
        std::cout << "  T0: "  << T.num_branch_ << std::endl;

}

void Intersection::ComputeConnectivity3()
{

    //
    //
    // Déplacer les points de l'intersection sur une sphère de référence
    //
    //

    std::vector<Vec3> points_proj;


    //
    // Trouver le plus grand rayon entre le centre et tout les points
    //

    // Pour pas s'embêter, on pourrait avoir d_max trés trés grand, comme la bb_ par exemple, ou 10^3 fois le rayon de centre_
    double d_max = 0;
    Vec3 Icentre = centre_.head<3>();

    for(Vec3 point_courant : contours_)
    {
        Vec3 res = Icentre - point_courant;
        double d = res.norm();
        if(d > d_max)
            d_max = d;
    }


    //
    // Projeter tout ces points sur une sphère de rayon dmax
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
    //

    /*
    Vec3 sum = {0.0, 0.0, 0.0};
    for(Vec3 pts : points_proj)
    {
        Vec3 res = Icentre - pts;
        std::cout << "la distance est : " << res.norm() << std::endl;
    }*/



    //
    //
    // Algorithme de création du volume convexe en utilisant des faces triangulaires
    //
    //


    std::vector<TriangleGeo> volume_courant; // renvoi des indices correspondant à des points de contours_
    int nb_courant = 4;

    //
    // Commencer par un tétraèdre
    //

    //
    // Première face

    Vec3 p1 = points_proj[0];
    Vec3 p2 = points_proj[1];
    Vec3 p3 = points_proj[2];
    TriangleGeo T1(p1, p2, p3, 0, 1, 2, -1);

    //
    // Trouver le point le plus éloigné

    double dist_ref = 0;
    int ind_ref;
    for(int a = 3; a < points_proj.size(); a++)
    {
        Vec3 resultante = points_proj[a] - p1;
        double dist = resultante.dot(T1.normal_)*resultante.dot(T1.normal_);
        if( dist > dist_ref)
        {
            dist_ref = dist;
            ind_ref = a;
        }
    }

    std::cout << "premier sommet : " << ind_ref << std::endl;
    Vec3 p4 = points_proj[ind_ref];
    T1.OrientedNormal(p4);
    volume_courant.push_back(T1);


    //
    // Fermer le tétraèdre avec 3 autre faces

    // Calcul d'un point interieur (pas forcement nécéssaire)
    //Vec3 Bcentre = (p1 + p2 + p3 + p4)/nb_courant;
    // Construction des trois autre triangles du tétraèdre
    TriangleGeo T2(p1, p2, p4, 0, 1, ind_ref, -1);
    TriangleGeo T3(p1, p3, p4, 0, 2, ind_ref, -1);
    TriangleGeo T4(p2, p3, p4, 1, 2, ind_ref, -1);
    // Orientation des faces vers le centre du tétraèdre (vers l'intérieur)
    T2.OrientedNormal(p3);
    T3.OrientedNormal(p2);
    T4.OrientedNormal(p1);
    // Stockage des faces dans un buffer volume courant
    volume_courant.push_back(T2);
    volume_courant.push_back(T3);
    volume_courant.push_back(T4);


    //
    // Boucle pour modifier le volume en lui ajoutant des faces => on parcourt tout les points n'appartenant pas encore au volume
    //

    /*
    bool check = false;
    bool face_change = false;
    int incre = 0;
    std::vector<TriangleGeo>::iterator it = volume_courant.begin();*/

    int id_point = 0;

    for(Vec3 s : points_proj)
    {
        bool point_volume = false ;
        std::cout << "id point :  " << id_point << std::endl;

        //
        // On vérifie que l'on est pas sur le volume
        for(TriangleGeo T_volume : volume_courant)
        {

            for(int k = 0; k < 3; k++)
            {
                if(T_volume.connectivity_[k] == id_point)
                {
                    point_volume = true;
                    break;
                }
            }
            if(point_volume)
                break;
        }
//

        int id_face = 0;
        std::vector<int> ind_face_interne;

        if(!point_volume)
        {
            std::cout << "souvent ou pas " << std::endl;

            //
            // On stocke les faces voyant le point à l'extérieur, s'il n'appartient pas au volume il y en a forcement une pour chaque point puisqu'ils sont tous sur l'enveloppe convexe
            for(TriangleGeo T_courant : volume_courant)
            {

                double dist = T_courant.normal_.dot(points_proj[id_point] - T_courant.sommets_[0]);
                std::cout << "dist :  " << dist<< std::endl;
                if(dist < -0.001)
                    ind_face_interne.push_back(id_face);

                id_face++;
            }


            //
            // On stocke toute les arêtes des faces visibles par le point (n'appartenant pas au volume)
            std::vector<int> ind_sommets;
            std::vector<int> ind_restant;

            for(int l = 0; l < ind_face_interne.size(); l++)
            {
                TriangleGeo F = volume_courant[ind_face_interne[l]];

                int a = F.connectivity_[0];
                int b = F.connectivity_[1];
                int c = F.connectivity_[2];

                ind_sommets.push_back(a);
                ind_sommets.push_back(b);
                ind_sommets.push_back(b);
                ind_sommets.push_back(c);
                ind_sommets.push_back(c);
                ind_sommets.push_back(a);
            }


            //
            // On compte les arêtes identiques : si elles sont présentes 2 fois, alors elles sont internes. Si elles ne le sont 1 fois, alors on en aura besoin pour les nouvelles faces à créer
            for(int n = 0; n < ind_sommets.size(); n++)
            {

                // dpair et dimpair forment une arête
                int dpair = ind_sommets[n];
                n++;
                int dimpair = ind_sommets[n];

                // On cherche les arêtes identiques et on incrémente count arête
                int count_arete = 1;
                for(int m = 0; m < ind_sommets.size(); m++)
                {
                    if((n != m) && (dimpair == ind_sommets[m]) && (dpair == ind_sommets[m+1]))
                        count_arete++;
                }

                // Si l'arête n'a pas de doublon dans la liste d'arête, alors elle est nécessaire à la création des nouvelles faces car c'est une arête "de bord"
                if(count_arete < 2)
                {
                    ind_restant.push_back(dpair);
                    ind_restant.push_back(dimpair);
                }
            }

            //
            // Creation de nouvelles faces (une pour chaque arête)
            std::vector<TriangleGeo> Nouv_faces;
            for(int w = 1; w < ind_restant.size(); w++)
            {
                TriangleGeo T(points_proj[ind_restant[w-1]], points_proj[ind_restant[w]], s, ind_restant[w-1], ind_restant[w], id_point, -1);
                w++;
                // N'importe quel point du volume, tant qu'il n'est pas un point de l'arête courante => ind_sommets est une super variable pour ça?
                int increment = 0;

                while((ind_sommets[increment] == ind_restant[w-1]) || (ind_sommets[increment] == ind_restant[w]))
                    increment++;

                Vec3 point_interieur = points_proj[ind_sommets[increment]];

                T.OrientedNormal(point_interieur);

                Nouv_faces.push_back(T);
            }


            //
            // On rempli un vector intermediaire de faces qui ne contient pas les faces à supprimer
            std::vector<TriangleGeo> volume_temp_buffer;
            int var = 0;

            for(TriangleGeo T_actu : volume_courant)
            {
                // Vérification : si il faut supprimer la face, alors mauvaise_face = true
                bool mauvaise_face = false;
                for(int h : ind_face_interne)
                {
                    if(var == h)
                        mauvaise_face = true;
                }

                // Si la face n'est pas mauvaise, alors on a le droit de l'ajouter dans le buffer de face temporaire
                if(mauvaise_face == false)
                    volume_temp_buffer.push_back(T_actu);

                var++;
            }


            //
            // On ajoute les nouvelles faces
            for(TriangleGeo T_nouv : Nouv_faces)
                volume_temp_buffer.push_back(T_nouv);


            //
            // On écrase le volume courant
            volume_courant = volume_temp_buffer;
        }

        id_point++;
    }



    //
    //
    // Stockage de la connectivité finale en attribut
    //
    //


    int count = 0;

    for(TriangleGeo T_actu : volume_courant)
    {

        int a = T_actu.connectivity_[0];
        int b = T_actu.connectivity_[1];
        int c = T_actu.connectivity_[2];

        //std::cout << "cond1 " << a/TYPE_PRIMITIVE << "   cond2 " << b/TYPE_PRIMITIVE << "   cond3 " << c/TYPE_PRIMITIVE << std::endl;
        std::cout << "cond1 " << a << "   cond2 " << b << "   cond3 " << c << std::endl;

        //
        // Ici, trouver de quelle branche provient la face à l'aide de a,b,c qui nous donnent les indices des sommets de contours_
        for(int j : branches_incidentes_)
        {
            //std::cout << "branche j "<< j << std::endl;

            bool cond1 = a/TYPE_PRIMITIVE == j;
            bool cond2 = b/TYPE_PRIMITIVE == j;
            bool cond3 = c/TYPE_PRIMITIVE == j;

            if( cond1 && cond2 && cond3 )
                T_actu.num_branch_ = j;
        }


        //
        // Boucle nous permettant de vérifier qui sont les voisins => remplissage des attribut ind1,2,3
        for(int i = 0 ; i < volume_courant.size(); i++)
        {

            // Ecarter la condition où l'on est sur le triangle courant
            if(count != i)
            {
                TriangleGeo T_comp = volume_courant[i];

                if((a == T_comp.connectivity_[0] && b == T_comp.connectivity_[2]) || (a == T_comp.connectivity_[1] && b == T_comp.connectivity_[0]) || (a == T_comp.connectivity_[2] && b == T_comp.connectivity_[1]))
                    T_actu.ind1_ = i;
                if((b == T_comp.connectivity_[0] && c == T_comp.connectivity_[2]) || (b == T_comp.connectivity_[1] && c == T_comp.connectivity_[0]) || (b == T_comp.connectivity_[2] && c == T_comp.connectivity_[1]))
                    T_actu.ind2_ = i;
                if((c == T_comp.connectivity_[0] && a == T_comp.connectivity_[2]) || (c == T_comp.connectivity_[1] && a == T_comp.connectivity_[0]) || (c == T_comp.connectivity_[2] && a == T_comp.connectivity_[1]))
                    T_actu.ind3_ = i;
            }
        }

        //
        // On remplace le triangle courant et on passe au suivant
        volume_courant[count] = T_actu;
        count++;
    }
    faces_ = volume_courant;



    // Vérification du contenu de faces_ => les triangles formant le volume : verification de la connectivité ... toute les arêtes apparaisent 2 fois
    for(TriangleGeo T : faces_)
        std::cout << "  T0: "  << T.num_branch_ << std::endl;

}



void Intersection::ComputeConnectivity4()
{

    //
    //
    // Déplacer les points de l'intersection sur une sphère de référence
    //
    //

    std::vector<Vec3> points_proj;


    //
    // Trouver le plus grand rayon entre le centre et tout les points
    //

    // Pour pas s'embêter, on pourrait avoir d_max trés trés grand, comme la bb_ par exemple, ou 10^3 fois le rayon de centre_
    double d_max = 0;
    Vec3 Icentre = centre_.head<3>();

    for(Vec3 point_courant : contours_)
    {
        Vec3 res = Icentre - point_courant;
        double d = res.norm();
        if(d > d_max)
            d_max = d;
    }


    //
    // Projeter tout ces points sur une sphère de rayon dmax
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
    //

    /*
    Vec3 sum = {0.0, 0.0, 0.0};
    for(Vec3 pts : points_proj)
    {
        Vec3 res = Icentre - pts;
        std::cout << "la distance est : " << res.norm() << std::endl;
    }*/



    //
    //
    // Algorithme de création du volume convexe en utilisant des faces triangulaires
    //
    //


    std::vector<TriangleGeo> volume_courant; // renvoi des indices correspondant à des points de contours_
    int nb_courant = 4;

    //
    // Commencer par les faces initiales (issues des branches) => tout les points de l'enveloppe seront déjà présents
    //

    int entier = 0;
    while(entier < contours_.size())
    {
        //
        // Faire un ou plusieurs triangle, les stocker comme lot de 3 Vec3  et 3 indices
        std::vector<Vec3> triangles_of_branch;
        std::vector<int> indices;
        for(int i = 0; i < TYPE_PRIMITIVE; i++)
        {
            triangles_of_branch.push_back(points_proj[entier + i]);
            indices.push_back(entier + i);
        }

        //
        // Création d'un point de référence pour l'orientation des faces
        Vec3 p_extern;
        if(entier + TYPE_PRIMITIVE > contours_.size())
            p_extern = points_proj[entier - TYPE_PRIMITIVE];
        else
            p_extern = points_proj[entier - TYPE_PRIMITIVE];

        //
        // Stocker ces triangles dans le volume courant
        for(int k = 0; k < triangles_of_branch.size(); k = k + 3)
        {
            // Créer le triangle
            TriangleGeo T(triangles_of_branch[k], triangles_of_branch[k + 1] , triangles_of_branch[k + 2], indices[k], indices[k + 1], indices[k + 2], int(indices[k]/TYPE_PRIMITIVE));
            // Orienter le triangle
            T.OrientedNormal(p_extern);
            // Le stocker
            volume_courant.push_back(T);
        }

        entier = entier + TYPE_PRIMITIVE;
    }


    //
    // Initialiser le tableau d'arête libre (vu qu'on travail avec des triangles pour l'instant, toute les arêtes sont libres => mais à généraliser plus tard)
    //

    std::vector<int> arete_libre;
    for(TriangleGeo T : volume_courant)
    {
        int a = T.connectivity_[0];
        int b = T.connectivity_[1];
        int c = T.connectivity_[2];

        arete_libre.push_back(a);
        arete_libre.push_back(b);
        arete_libre.push_back(b);
        arete_libre.push_back(c);
        arete_libre.push_back(c);
        arete_libre.push_back(a);
    }


    //
    // Algo de vérif de convexité des faces candidates => tester tte les faces possibles, pour chaque arête avec chaque points de point_proj d'une autre branche
    //

    for(int s = 0; s < arete_libre.size(); s++)
    {
        //
        // On récupère les deux sommets de l'arête
        int s1 = arete_libre[s];
        s++;
        int s2 = arete_libre[s];
        int appartenance1 = int(s1/TYPE_PRIMITIVE);
        int appartenance2 = int(s2/TYPE_PRIMITIVE);

        //
        // On parcourt tout les points de notre ensemble
        int counter = 0;
        for(Vec3 v : points_proj)
        {
            // On filtre les point de l'arête actuelle et plus généralement ceux ayant une branche en commun avec l'arête
            int appartenance_courante = int(counter/TYPE_PRIMITIVE);
            if(counter != s1 && counter != s2 && (appartenance_courante != appartenance1 || appartenance_courante != appartenance2))
            {
                // Alors on peut construire une face
                TriangleGeo T_test(points_proj[s1], points_proj[s2], v, s1, s2, counter ,-1);
                // Puis on trouve un point externe à la face appartenant déjà au volume par exemple
                Vec3 point_ref;
                for(int p = 0; p < points_proj.size(); p++)
                {
                    if(p != s1 && p != s2 && p != counter)
                    {
                        point_ref = points_proj[p];
                        break;
                    }
                }
                // Puis on oriente correctement notre face de test
                T_test.OrientedNormal(point_ref);

                // Ensuite, on effectue le test de convexité avec tout les autre points de notre ensemble
                bool check_error = false;
                for(Vec3 point_test : points_proj)
                {
                    double dist = T_test.normal_.dot(point_test - T_test.sommets_[0]); // petit doute sur les cas particuliers
                    //std::cout << "dist :  " << dist<< std::endl;
                    if(dist > -0.001)
                        check_error = true;

                }
                if(check_error = false)
                {
                    volume_courant.push_back(T_test);

                    // Et mettre à jour les arêtes libres
                }

            }
            counter++;
        }
    }




    // mise à jour de arete libre => à faire à chaque iteration => critère de l'arête libre
    /*
    for(TriangleGeo T : volume_courant)
    {
        int a = T.connectivity_[0];
        int b = T.connectivity_[1];
        int c = T.connectivity_[2];

        if(!arete_libre.empty())
        {
            for(int i = 0; i < arete_libre.size(); i++)
            {
                int s1 = arete_libre[i];
                i++;
                int s2 = arete_libre[i];

                if(a == s2)
                {}


            }






        }
        else
        {
            arete_libre.push_back(a);
            arete_libre.push_back(b);
            arete_libre.push_back(b);
            arete_libre.push_back(c);
            arete_libre.push_back(c);
            arete_libre.push_back(a);
        }



    }*/


    //
    //
    // Stockage de la connectivité finale en attribut
    //
    //


    int count = 0;

    for(TriangleGeo T_actu : volume_courant)
    {

        int a = T_actu.connectivity_[0];
        int b = T_actu.connectivity_[1];
        int c = T_actu.connectivity_[2];

        //std::cout << "cond1 " << a/TYPE_PRIMITIVE << "   cond2 " << b/TYPE_PRIMITIVE << "   cond3 " << c/TYPE_PRIMITIVE << std::endl;
        std::cout << "cond1 " << a << "   cond2 " << b << "   cond3 " << c << std::endl;

        //
        // Ici, trouver de quelle branche provient la face à l'aide de a,b,c qui nous donnent les indices des sommets de contours_
        for(int j : branches_incidentes_)
        {
            //std::cout << "branche j "<< j << std::endl;

            bool cond1 = a/TYPE_PRIMITIVE == j;
            bool cond2 = b/TYPE_PRIMITIVE == j;
            bool cond3 = c/TYPE_PRIMITIVE == j;

            if( cond1 && cond2 && cond3 )
                T_actu.num_branch_ = j;
        }


        //
        // Boucle nous permettant de vérifier qui sont les voisins => remplissage des attribut ind1,2,3
        for(int i = 0 ; i < volume_courant.size(); i++)
        {

            // Ecarter la condition où l'on est sur le triangle courant
            if(count != i)
            {
                TriangleGeo T_comp = volume_courant[i];

                if((a == T_comp.connectivity_[0] && b == T_comp.connectivity_[2]) || (a == T_comp.connectivity_[1] && b == T_comp.connectivity_[0]) || (a == T_comp.connectivity_[2] && b == T_comp.connectivity_[1]))
                    T_actu.ind1_ = i;
                if((b == T_comp.connectivity_[0] && c == T_comp.connectivity_[2]) || (b == T_comp.connectivity_[1] && c == T_comp.connectivity_[0]) || (b == T_comp.connectivity_[2] && c == T_comp.connectivity_[1]))
                    T_actu.ind2_ = i;
                if((c == T_comp.connectivity_[0] && a == T_comp.connectivity_[2]) || (c == T_comp.connectivity_[1] && a == T_comp.connectivity_[0]) || (c == T_comp.connectivity_[2] && a == T_comp.connectivity_[1]))
                    T_actu.ind3_ = i;
            }
        }

        //
        // On remplace le triangle courant et on passe au suivant
        volume_courant[count] = T_actu;
        count++;
    }
    faces_ = volume_courant;



    // Vérification du contenu de faces_ => les triangles formant le volume : verification de la connectivité ... toute les arêtes apparaisent 2 fois
    for(TriangleGeo T : faces_)
        std::cout << "  T0: "  << T.num_branch_ << std::endl;

}
