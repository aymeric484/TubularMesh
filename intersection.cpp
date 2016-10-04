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
                if(dist < 0)
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
            p_extern = points_proj[entier + TYPE_PRIMITIVE];

        //
        // Stocker ces triangles dans le volume courant
        for(int k = 0; k < triangles_of_branch.size(); k = k + 3)
        {
            // Créer le triangle à l'aide de notre class "TriangleGeo"
            TriangleGeo T(triangles_of_branch[k], triangles_of_branch[k + 1] , triangles_of_branch[k + 2], indices[k], indices[k + 1], indices[k + 2], int(indices[k]/TYPE_PRIMITIVE));
            // Orienter le triangle
            T.OrientedNormal(Icentre);
            // Le stocker
            volume_courant.push_back(T);
        }

        entier = entier + TYPE_PRIMITIVE;
    }


    //
    // Initialiser le tableau d'arête libre (vu qu'on travail avec des triangles pour l'instant, toute les arêtes sont libres => mais à généraliser plus tard)
    //

    std::vector<int> ind_aretes;
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

        ind_aretes.push_back(a);
        ind_aretes.push_back(b);
        ind_aretes.push_back(b);
        ind_aretes.push_back(c);
        ind_aretes.push_back(c);
        ind_aretes.push_back(a);
    }


    //
    // Algo de vérif de convexité des faces candidates => tester tte les faces possibles, pour chaque arête avec chaque points de point_proj d'une autre branche
    //

    // Tout ça doit ce passer dans une boucle while => algo de convergence

    bool closed = false;
    int priorite = 2;
    while(!closed)
    {
        int count_test = 0;
        bool modif = false;
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

                // On filtre les point de l'arête actuelle et ceux ayant une branche en commun avec les deux sommets de l'arête
                int appartenance_courante = int(counter/TYPE_PRIMITIVE);
                if(counter != s1 && counter != s2 && (appartenance_courante != appartenance1 || appartenance_courante != appartenance2))
                {
                    count_test++;
                    std::cout << "nouv face" << std::endl;

                    // Alors on peut construire une face
                    TriangleGeo T_test(points_proj[s2], points_proj[s1], v, s2, s1, counter ,-1);
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
                    int d = T_test.connectivity_[0];
                    int e = T_test.connectivity_[1];
                    int f = T_test.connectivity_[2];
                    // Premier test : test de convexité avec tt les autres points de l'ensemble (non coplanaire)
                    bool check_error_conv = false;
                    int count2 = 0; // mettre une condition sur le point_test que l'on instancie après
                    for(Vec3 point_test : points_proj)
                    {
                        // On ne veut pas tester avec un point de la même face
                        if(count2 != d && count2 != e && count2 != f)
                        {
                            double dist = T_test.normal_.dot(point_test - T_test.sommets_[0]); // Petit doute sur les cas particuliers
                            std::cout << "dist :  " << dist<< std::endl;
                            if(dist < 0)
                                check_error_conv = true;
                        }
                        count2++;
                    }

                    // Deuxième test : On regarde si notre face se complémente bien avec le sens des arêtes des faces imposées
                    bool check_error_cond = false;
                    for(int q = 0; q < 3; q++)
                    {
                        int v1 = T_test.connectivity_[q];
                        int v2;
                        if(q + 1 < 3)
                            v2 = T_test.connectivity_[q + 1];
                        else
                            v2 = T_test.connectivity_[0];
                        for(int u = 0; u < ind_aretes.size(); u++)
                        {
                            int a1 = ind_aretes[u];
                            u++;
                            int a2 = ind_aretes[u];
                            if(a1 == v1)
                            {
                                if(a2 == v2)
                                    check_error_cond = true;
                            }
                        }
                    }

                    // Troisième test : Test de priorité => on préfère les faces qui auront le plus de doublon (le max étant 3) => il faudrait au moins en avoir 2 supp avec chaque face
                    // Tester arête courante, avec aretes_libre, récupérer une valeur de priorité pour chaque correspondance
                    // Idéalement, on devrait avoir 1 puis 2 ect.... puis 3.
                    int nb_complete = 0;
                    for(int r = 0; r < arete_libre.size(); r++)
                    {
                        int b1 = arete_libre[r];
                        r++;
                        int b2 = arete_libre[r];

                        //Tester cb d'aretes le triangle supprimera on cherche 1 puis 2 puis 3
                        if(d == b2 && e == b1)
                            nb_complete++;
                        if(e == b2 && f == b1)
                            nb_complete++;
                        if(f == b2 && d == b1)
                            nb_complete++;
                    }

                    //std::cout << "avec " << nb_complete << std::endl;




                    // Si un seul des points n'est pas à l'intérieur de la face, alors error = true. Sinon, le triangle est valide => On peut le stocker
                    if(check_error_conv == false && check_error_cond == false && (volume_courant.size() == 3 || nb_complete > priorite))
                    {
                        std::cout << "cond vérifié" << std::endl;
                        volume_courant.push_back(T_test);
                        modif = true; // Signifie que l'on a modifier le volume courant => sortie des deux boucles for()
                        break;
                    }

                }

                counter++;
                if(modif)
                    break;
            }
            if(modif)
                break;

        }
        if(!modif)
            priorite--;
        else
            priorite = 2;

        // Cas où il n'y a pas de solution du tout => on s'est trompé à un moment
        if(priorite == 0)
        {}

        std::cout << "count_test" << count_test << std::endl;
        sleep(1);
        for(int i = 0; i < arete_libre.size(); i++)
            std::cout << "sommet" << arete_libre[i] << std::endl;
        sleep(5);
        // Et mettre à jour les arêtes libres
        // Parcourir une liste d'arêtes du volume pour mettre à jour arête libre
        // Puis sortir de la boucle si l'on fait ça

        // regroupe toute les arêtes du volume courant


        //
        // Construction du tableau d'arête

        ind_aretes.clear();
        for(TriangleGeo T : volume_courant)
        {
            int a = T.connectivity_[0];
            int b = T.connectivity_[1];
            int c = T.connectivity_[2];
            ind_aretes.push_back(a);
            ind_aretes.push_back(b);
            ind_aretes.push_back(b);
            ind_aretes.push_back(c);
            ind_aretes.push_back(c);
            ind_aretes.push_back(a);
            std::cout << a << "  " << b << "  " << c << std::endl;
        }

        //
        // Parcourt de ce tableau d'arête pour trouver les doublons
        arete_libre.clear();
        for(int w = 0; w < ind_aretes.size(); w++)
        {
            int s1 = ind_aretes[w];
            w++;
            int s2 = ind_aretes[w];

            // Pour tout couple s1 et s2, checher dans la liste un équivalent s2 & s1
            bool exist = false;
            for(int j = 0; j < ind_aretes.size(); j++)
            {
                if(s2 == ind_aretes[j])
                {
                    //std::cout << " found a match" << std::endl;
                    j++;
                    if(s1 == ind_aretes[j])
                    {
                        std::cout << " found a second match" << std::endl;
                        exist = true;
                    }
                }
                else
                    j++;
            }

            // Dans le cas où l'on a pas trouvé de doublon, c'est que l'arête est libre
            if(exist == false)
            {
                arete_libre.push_back(s1);
                arete_libre.push_back(s2);
            }
        }
        if(arete_libre.empty())
        {
            std::cout << "sortie boucle" << std::endl;
            closed = true;
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


void Intersection::ComputeConnectivity5()
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
            p_extern = points_proj[entier + TYPE_PRIMITIVE];

        //
        // Stocker ces triangles dans le volume courant
        for(int k = 0; k < triangles_of_branch.size(); k = k + 3)
        {
            // Créer le triangle à l'aide de notre class "TriangleGeo"
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

    std::vector<int> ind_aretes;
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

        ind_aretes.push_back(a);
        ind_aretes.push_back(b);
        ind_aretes.push_back(b);
        ind_aretes.push_back(c);
        ind_aretes.push_back(c);
        ind_aretes.push_back(a);
    }


    //
    // Algo de vérif de convexité des faces candidates => tester tte les faces possibles, pour chaque arête avec chaque points de point_proj d'une autre branche
    //

    // Tout ça doit ce passer dans une boucle while => algo de convergence

    bool closed = false;
    int priorite = 2;
    // Variables intermédiaires de sauvegarde des embranchements => elles nous permettrons de retourner en arrière
    std::vector<TriangleGeo> triangles_possibles; // on y stocke les triangles ayant été compatible
    std::vector<int> code_iteration; // On y stocke l'itération à laquelle chaque triangle aurrait été compatible
    int var = 0;
    while(!closed)
    {
        int count_test = 0;
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

                // On filtre les point de l'arête actuelle et ceux ayant une branche en commun avec les deux sommets de l'arête
                int appartenance_courante = int(counter/TYPE_PRIMITIVE);
                if(counter != s1 && counter != s2 && (appartenance_courante != appartenance1 || appartenance_courante != appartenance2))
                {
                    count_test++;
                    std::cout << "nouv face" << std::endl;

                    // Alors on peut construire une face
                    TriangleGeo T_test(points_proj[s2], points_proj[s1], v, s2, s1, counter ,-1);
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
                    int d = T_test.connectivity_[0];
                    int e = T_test.connectivity_[1];
                    int f = T_test.connectivity_[2];
                    // Premier test : test de convexité avec tt les autres points de l'ensemble (non coplanaire)
                    bool check_error_conv = false;
                    int count2 = 0; // mettre une condition sur le point_test que l'on instancie après
                    for(Vec3 point_test : points_proj)
                    {
                        // On ne veut pas tester avec un point de la même face
                        if(count2 != d && count2 != e && count2 != f)
                        {
                            double dist = T_test.normal_.dot(point_test - T_test.sommets_[0]); // Petit doute sur les cas particuliers
                            std::cout << "dist :  " << dist<< std::endl;
                            //sleep(1);
                            if(dist < -0.001)
                                check_error_conv = true;
                        }
                        count2++;
                    }

                    // Deuxième test : On regarde si notre face se complémente bien avec le sens des arêtes des faces imposées
                    bool check_error_cond = false;
                    for(int q = 0; q < 3; q++)
                    {
                        int v1 = T_test.connectivity_[q];
                        int v2;
                        if(q + 1 < 3)
                            v2 = T_test.connectivity_[q + 1];
                        else
                            v2 = T_test.connectivity_[0];
                        for(int u = 0; u < ind_aretes.size(); u++)
                        {
                            int a1 = ind_aretes[u];
                            u++;
                            int a2 = ind_aretes[u];
                            if(a1 == v1)
                            {
                                if(a2 == v2)
                                    check_error_cond = true;
                            }
                        }
                    }

                    // Troisième test : Test de priorité => on préfère les faces qui auront le plus de doublon (le max étant 3) => il faudrait au moins en avoir 2 supp avec chaque face
                    // Tester arête courante, avec aretes_libre, récupérer une valeur de priorité pour chaque correspondance
                    // Idéalement, on devrait avoir 1 puis 2 ect.... puis 3.
                    int nb_complete = 0;
                    for(int r = 0; r < arete_libre.size(); r++)
                    {
                        int b1 = arete_libre[r];
                        r++;
                        int b2 = arete_libre[r];

                        //Tester cb d'aretes le triangle supprimera on cherche 1 puis 2 puis 3
                        if(d == b2 && e == b1)
                            nb_complete++;
                        if(e == b2 && f == b1)
                            nb_complete++;
                        if(f == b2 && d == b1)
                            nb_complete++;
                    }

                    //std::cout << "avec " << nb_complete << std::endl;

                    // Si un seul des points n'est pas à l'intérieur de la face, alors error = true. Sinon, le triangle est valide => On peut le stocker
                    if(check_error_conv == false && check_error_cond == false && (volume_courant.size() == 3 || nb_complete > priorite))
                    {
                        std::cout << "cond vérifié" << std::endl;
                        triangles_possibles.push_back(T_test);
                        code_iteration.push_back(volume_courant.size() - 3); // Actually, 3 might not always be the number of faces we'll start with
                        //volume_courant.push_back(T_test);
                    }
                }
                counter++;
            }
        }



        //
        // L'objectif ici est de déterminer toutes les possibilités que l'on a
        if(triangles_possibles.empty())
            priorite--;
        else
        {
            volume_courant.push_back(triangles_possibles[var]);
            priorite = 2;
        }

        // Cas où il n'y a pas de solution du tout => on s'est trompé à un moment, donc on revient en arrière et on essai avec un autre triangle
        if(priorite == 0)
        {
            // On se replace à l'itération précédente
            volume_courant.pop_back(); // Signifie que le dernier triangle ajouté était faux et qu'il faut en prendre un autre dans les choix que l'on avait
            var++; // Cette variable incrémente la liste des choix de triangles que l'on avait => le prochain sera selectionné
        }

        std::cout << "count_test" << count_test << std::endl;
        sleep(1);
        for(int i = 0; i < arete_libre.size(); i++)
            std::cout << "sommet" << arete_libre[i] << std::endl;
        sleep(5);
        // Et mettre à jour les arêtes libres
        // Parcourir une liste d'arêtes du volume pour mettre à jour arête libre
        // Puis sortir de la boucle si l'on fait ça

        // regroupe toute les arêtes du volume courant


        //
        // Construction du tableau d'arête

        ind_aretes.clear();
        for(TriangleGeo T : volume_courant)
        {
            int a = T.connectivity_[0];
            int b = T.connectivity_[1];
            int c = T.connectivity_[2];
            ind_aretes.push_back(a);
            ind_aretes.push_back(b);
            ind_aretes.push_back(b);
            ind_aretes.push_back(c);
            ind_aretes.push_back(c);
            ind_aretes.push_back(a);
            std::cout << a << "  " << b << "  " << c << std::endl;
        }

        //
        // Parcourt de ce tableau d'arête pour trouver les doublons
        arete_libre.clear();
        for(int w = 0; w < ind_aretes.size(); w++)
        {
            int s1 = ind_aretes[w];
            w++;
            int s2 = ind_aretes[w];

            // Pour tout couple s1 et s2, checher dans la liste un équivalent s2 & s1
            bool exist = false;
            for(int j = 0; j < ind_aretes.size(); j++)
            {
                if(s2 == ind_aretes[j])
                {
                    //std::cout << " found a match" << std::endl;
                    j++;
                    if(s1 == ind_aretes[j])
                    {
                        std::cout << " found a second match" << std::endl;
                        exist = true;
                    }
                }
                else
                    j++;
            }

            // Dans le cas où l'on a pas trouvé de doublon, c'est que l'arête est libre
            if(exist == false)
            {
                arete_libre.push_back(s1);
                arete_libre.push_back(s2);
            }
        }
        if(arete_libre.empty())
        {
            std::cout << "sortie boucle" << std::endl;
            closed = true;
        }
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


void Intersection::ComputeConnectivity6()
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
    // Test de convexité des 3 faces de départ
    //
    /*
    TriangleGeo TX1(points_proj[0], points_proj[1], points_proj[2], 0, 1, 2, -1);
    TriangleGeo TX2(points_proj[3], points_proj[4], points_proj[5], 3, 4, 5, -1);
    TriangleGeo TX3(points_proj[6], points_proj[7], points_proj[8], 6, 7, 8, -1);

    TX1.OrientedNormal(Icentre);
    TX2.OrientedNormal(Icentre);
    TX3.OrientedNormal(Icentre);

    int counter = 0;
    bool ce = false;
    for(Vec3 proj : points_proj)
    {
        if(counter != 0 && counter != 1 && counter != 2)
        {
            double dist1 = TX1.normal_.dot(proj - TX1.sommets_[0]);
            if(dist1 < 0)
                ce = true;
        }
        if(counter != 3 && counter != 4 && counter != 5)
        {
            double dist2 = TX2.normal_.dot(proj - TX2.sommets_[0]);
            if(dist2 < 0)
                ce = true;

        }
        if(counter != 6 && counter != 7 && counter != 8)
        {
            double dist3 = TX3.normal_.dot(proj - TX3.sommets_[0]);
            if(dist3 < 0)
                ce = true;
        }
        counter++;
    }

    if(ce)
        std::cout << " Bad input data" << std::endl;
    */





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
    // Algo de réparation des faces du polyèdres obtenu
    //
    //

    std::cout << "  hello!  " << std::endl;

    bool modif = true;
    while(modif)
    {
        int ind_T1 = 0;
        int ind_T2 = 0;
        modif = false;
        for(TriangleGeo T_courant : volume_courant)
        {
            int v1 = T_courant.connectivity_[0];
            int v2 = T_courant.connectivity_[1];
            int v3 = T_courant.connectivity_[2];

            int appartenance1 = int(v1/TYPE_PRIMITIVE);
            int appartenance2 = int(v2/TYPE_PRIMITIVE);
            int appartenance3 = int(v3/TYPE_PRIMITIVE);
            int appartenance_utile;

            bool sim1 = (appartenance1 == appartenance2);
            bool sim2 = (appartenance2 == appartenance3);
            bool sim3 = (appartenance3 == appartenance1);
            bool T_interessant = false;

            // On y stocke une des arêtes à potentielement détruire
            int arete11;
            int arete12;
            // Idem ici
            int arete21;
            int arete22;


            //
            // Il faut exactement 1 arete de la branche dans le triangle. Alors, on stocke les deux autres
            //

            //
            // Il s'agit ici de l'arête v1v2
            if(sim1 && (!sim2))
            {
                // arête v2v3
                arete11 = v2;
                arete12 = v3;
                // arête v3v1
                arete21 = v3;
                arete22 = v1;
                // Nous indique qu'il s'agit d'un triangle lié à la branche par une arête
                T_interessant = true;
                // Nous indique de quelle branche il s'agit
                appartenance_utile = appartenance1;
            }
            //
            // Il s'agit ici de l'arête v2v3
            if(sim2 && (!sim3))
            {
                // arête v3v1
                arete11 = v3;
                arete12 = v1;
                // arête v1v2
                arete21 = v1;
                arete22 = v2;
                // Nous indique qu'il s'agit d'un triangle lié à la branche par une arête
                T_interessant = true;
                // Nous indique de quelle branche il s'agit
                appartenance_utile = appartenance2;
            }
            //
            // Il s'agit ici de l'arête v3v1
            if(sim3 && (!sim2))
            {
                // arête v1v2
                arete11 = v1;
                arete12 = v2;
                // arête v2v3
                arete21 = v2;
                arete22 = v3;
                // Nous indique qu'il s'agit d'un triangle lié à la branche par une arête
                T_interessant = true;
                // Nous indique de quelle branche il s'agit
                appartenance_utile = appartenance3;
            }


            //
            // Si T répond à une des conditions, trouver laquelle des 2 arêtes doit être détruite en cherchant le triangle voisin et en vérifiant que ce dernier ne soit pas parfait et contienne un point de même appartenance
            //

            if(T_interessant)
            {
                ind_T2 = 0;
                for(TriangleGeo T_test : volume_courant)
                {
                    int v4 = T_test.connectivity_[0];
                    int v5 = T_test.connectivity_[1];
                    int v6 = T_test.connectivity_[2];

                    int appartenance4 = int(v4/TYPE_PRIMITIVE);
                    int appartenance5 = int(v5/TYPE_PRIMITIVE);
                    int appartenance6 = int(v6/TYPE_PRIMITIVE);

                    //
                    // On écarte ensuite les triangles parfaits (appartenant completement à une branche)
                    if((appartenance4 != appartenance5) || (appartenance5 != appartenance6) || (appartenance4 != appartenance6))
                    {
                        //
                        // Créer le tableau d'arête du triangle courant
                        std::vector<int> aretes_triangle;
                        aretes_triangle.push_back(v4);
                        aretes_triangle.push_back(v5);
                        aretes_triangle.push_back(v5);
                        aretes_triangle.push_back(v6);
                        aretes_triangle.push_back(v6);
                        aretes_triangle.push_back(v4);

                        //
                        // test : Condition, le triangle est voisin de l'arête1 ou de l'arête2
                        for(int i = 0; i < aretes_triangle.size(); i++)
                        {
                            // Tester une arête courante
                            int s1 = aretes_triangle[i];
                            i++;
                            int s2 = aretes_triangle[i];


                            // Extraire le 4ème point
                            int s4;
                            if(i + 2 < aretes_triangle.size())
                                s4 = aretes_triangle[i + 2];
                            else
                                s4 = aretes_triangle[i - 4];
                            int appartenance7 = int(s4/TYPE_PRIMITIVE);

                            // Voir si une arête est voisine et si le 3ème point appartient à la branche dont il est question (codée par appartenance_utile)
                            if((s1 == arete12 && s2 == arete11 && appartenance7 == appartenance_utile) || (s1 == arete22 && s2 == arete21 && appartenance7 == appartenance_utile))
                            {
                                //
                                // ici, créer les nouveaux triangles
                                int arete_new1;
                                int arete_new2;
                                for(int l1 : T_test.connectivity_)
                                {
                                    if(l1 != s1 && l1 != s2)
                                        arete_new1 = l1;

                                }
                                for(int l2 : T_courant.connectivity_)
                                {
                                    if(l2 != s1 && l2 != s2)
                                        arete_new2 = l2;

                                }

                                TriangleGeo T1_new(points_proj[s1], points_proj[arete_new1], points_proj[arete_new2], s1, arete_new1, arete_new2, -1);
                                TriangleGeo T2_new(points_proj[s2], points_proj[arete_new1], points_proj[arete_new2], s2, arete_new1, arete_new2, -1);
                                T1_new.OrientedNormal(Icentre);
                                T2_new.OrientedNormal(Icentre);

                                // effectuer un test ici sur nos deux triangles et leurs convexité
                                int count = 0;
                                bool check_error_conv = false;
                                for(Vec3 pt : points_proj)
                                {
                                    std::cout << "PT" << std::endl;

                                    // On ne veut pas tester avec un point de la même face
                                    if(count != s1 && count != arete_new1 && count != arete_new2)
                                    {
                                        double dist1 = T1_new.normal_.dot(pt - T1_new.sommets_[0]);
                                        if(dist1 < 0)
                                        {
                                            check_error_conv = true;
                                            std::cout << "true1" << std::endl;
                                        }
                                    }
                                    if(count != s2 && count != arete_new1 && count != arete_new2)
                                    {
                                        double dist2 = T2_new.normal_.dot(pt - T2_new.sommets_[0]);
                                        if(dist2 < 0)
                                        {
                                            check_error_conv = true;
                                            std::cout << "true2" << std::endl;
                                        }
                                    }
                                    count++;
                                }

                                // Si les deux triangles ont réussis le test de convergence, alors c'est OK => on peut remplacer les triangles et quitter les boucles
                                if(!check_error_conv)
                                {
                                    volume_courant[ind_T1] = T1_new;
                                    volume_courant[ind_T2] = T2_new;

                                    modif = true;
                                    break;
                                }
                            }
                        }
                    }
                    if(modif)
                        break;
                    else
                        ind_T2++;
                }
            }
            if(modif)
                break;
            else
                ind_T1++;
        }

        //
        // Construire les deux nouveaux triangles et remplacer les anciens
        //

        std::cout << "hello 2 " << std::endl;


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


void Intersection::ComputeConnectivity7()
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
        double ratio = 20*d_max/distance;
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
    // Test de convexité des 3 faces de départ
    //

    /*
    TriangleGeo TX1(points_proj[0], points_proj[1], points_proj[2], 0, 1, 2, -1);
    TriangleGeo TX2(points_proj[6], points_proj[4], points_proj[5], 6, 4, 5, -1);
    TriangleGeo TX3(points_proj[9], points_proj[10], points_proj[8], 9, 10, 8, -1);

    TX1.OrientedNormal(Icentre);
    TX2.OrientedNormal(Icentre);
    TX3.OrientedNormal(Icentre);

    int counter = 0;
    bool ce = false;
    for(Vec3 proj : points_proj)
    {
        if(counter != 0 && counter != 1 && counter != 2)
        {
            double dist1 = TX1.normal_.dot(proj - TX1.sommets_[0]);
            if(dist1 < 0)
                ce = true;
        }
        if(counter != 6 && counter != 4 && counter != 5)
        {
            double dist2 = TX2.normal_.dot(proj - TX2.sommets_[0]);
            if(dist2 < 0)
                ce = true;

        }
        if(counter != 9 && counter != 10 && counter != 8)
        {
            double dist3 = TX3.normal_.dot(proj - TX3.sommets_[0]);
            if(dist3 < 0)
                ce = true;
        }
        counter++;
    }

    if(ce)
        std::cout << " Bad input data" << std::endl;*/




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

    Vec3 p1 = points_proj[1];
    Vec3 p2 = points_proj[2];
    Vec3 p3 = points_proj[3];
    TriangleGeo T1(p1, p2, p3, 1, 2, 3, -1);

    //
    // Trouver le point le plus éloigné

    double dist_ref = 0;
    int ind_ref = 0;
    for(int a = 3; a < points_proj.size(); a++)
    {
        Vec3 resultante = points_proj[a] - p1;
        resultante.normalize();
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
    TriangleGeo T2(p1, p2, p4, 1, 2, ind_ref, -1);
    TriangleGeo T3(p1, p3, p4, 1, 3, ind_ref, -1);
    TriangleGeo T4(p2, p3, p4, 2, 3, ind_ref, -1);
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
            //
            // On stocke les faces voyant le point à l'extérieur, s'il n'appartient pas au volume il y en a forcement une pour chaque point puisqu'ils sont tous sur l'enveloppe convexe
            for(TriangleGeo T_courant : volume_courant)
            {

                Vec3 res = s - T_courant.sommets_[0];
                res.normalize();
                double dist = T_courant.normal_.dot(res);
                std::cout << "dist :  " << dist<< std::endl;

                if(dist < 0.05 && dist > -0.05)
                {
                    std::cout << " point quasi-coplanaire  " << std::endl;
                    std::cout << " id_face du volume courant  " << id_face  << std::endl;
                    std::cout << " id_point n'appartenant pas au volume  " << id_point  << std::endl;
                    std::cout << " indice 0 : " << T_courant.connectivity_[0] << " indice 1 : " << T_courant.connectivity_[1] << " indice 2 : " << T_courant.connectivity_[2] << std::endl;
                }

                if(dist < 0)
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

            bool cond1 = a/(TYPE_PRIMITIVE + 1) == j;
            bool cond2 = b/(TYPE_PRIMITIVE + 1) == j;
            bool cond3 = c/(TYPE_PRIMITIVE + 1) == j;

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


void Intersection::ComputeConnectivity8()
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


    //
    // Construire la première face
    //

    //
    // Première face

    Vec3 p1 = points_proj[0];
    Vec3 p2 = points_proj[1];
    Vec3 p3 = points_proj[2];
    TriangleGeo T1(p1, p2, p3, 1, 2, 3, -1);

    //
    // Prendre un point d'une autre branche pour ensuite orienter la face puis la stocker

    Vec3 p4 = points_proj[points_proj.size() - 1];
    T1.OrientedNormal(p4);
    volume_courant.push_back(T1);


    //
    // Initialiser le tableau d'arête libre (vu qu'on travail avec un triangle pour l'instant, toute les arêtes sont libres => mais à généraliser plus tard, OU PAS)
    //

    std::vector<int> ind_aretes;
    std::vector<int> arete_libre;
    int a = T1.connectivity_[0];
    int b = T1.connectivity_[1];
    int c = T1.connectivity_[2];

    arete_libre.push_back(a);
    arete_libre.push_back(b);
    arete_libre.push_back(b);
    arete_libre.push_back(c);
    arete_libre.push_back(c);
    arete_libre.push_back(a);

    ind_aretes.push_back(a);
    ind_aretes.push_back(b);
    ind_aretes.push_back(b);
    ind_aretes.push_back(c);
    ind_aretes.push_back(c);
    ind_aretes.push_back(a);


    //
    // Algo de vérif de convexité des faces candidates => tester tte les faces possibles, pour chaque arête avec chaque points de point_proj d'une autre branche
    //

    // Tout ça doit ce passer dans une boucle while => algo de convergence

    bool closed = false;
    int priorite = 2; // En avons nous besoin? Il ne devrais pas y avoir de face convexe.
    while(!closed)
    {
        int count_test = 0;
        bool modif = false;
        for(int s = 0; s < arete_libre.size(); s++)
        {
            //
            // On récupère les deux sommets de l'arête
            int s1 = arete_libre[s];
            s++;
            int s2 = arete_libre[s];

            //
            // On parcourt tout les points de notre ensemble
            int counter = 0;
            for(Vec3 v : points_proj)
            {
                if(counter != s1 && counter != s2)
                {
                    count_test++;

                    // Alors on peut construire une face
                    TriangleGeo T_test(points_proj[s2], points_proj[s1], v, s2, s1, counter ,-1);
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
                    int d = T_test.connectivity_[0];
                    int e = T_test.connectivity_[1];
                    int f = T_test.connectivity_[2];
                    // Premier test : test de convexité avec tt les autres points de l'ensemble (non coplanaire)
                    bool check_error_conv = false;
                    int count2 = 0; // mettre une condition sur le point_test que l'on instancie après
                    for(Vec3 point_test : points_proj)
                    {
                        // On ne veut pas tester avec un point de la même face
                        if(count2 != d && count2 != e && count2 != f)
                        {
                            double dist = T_test.normal_.dot(point_test - T_test.sommets_[0]); // Petit doute sur les cas particuliers
                            std::cout << "dist :  " << dist<< std::endl;
                            if(dist < 0.015)
                                check_error_conv = true;
                        }
                        count2++;
                    }

                    // Deuxième test : On regarde si notre face se complémente bien avec le sens des arêtes des faces imposées => verifie quelles sont bien orientées => un double check
                    bool check_error_cond = false;
                    for(int q = 0; q < 3; q++)
                    {
                        int v1 = T_test.connectivity_[q];
                        int v2;
                        if(q + 1 < 3)
                            v2 = T_test.connectivity_[q + 1];
                        else
                            v2 = T_test.connectivity_[0];
                        for(int u = 0; u < ind_aretes.size(); u++)
                        {
                            int a1 = ind_aretes[u];
                            u++;
                            int a2 = ind_aretes[u];
                            if(a1 == v1)
                            {
                                if(a2 == v2)
                                    check_error_cond = true;
                            }
                        }
                    }

                    // Troisième test : Test de priorité => on préfère les faces qui auront le plus de doublon (le max étant 3) => il faudrait au moins en avoir 2 supp avec chaque face
                    // Tester arête courante, avec aretes_libre, récupérer une valeur de priorité pour chaque correspondance
                    // Idéalement, on devrait avoir 1 puis 2 ect.... puis 3.
                    int nb_complete = 0;
                    for(int r = 0; r < arete_libre.size(); r++)
                    {
                        int b1 = arete_libre[r];
                        r++;
                        int b2 = arete_libre[r];

                        //Tester cb d'aretes le triangle supprimera on cherche 1 puis 2 puis 3
                        if(d == b2 && e == b1)
                            nb_complete++;
                        if(e == b2 && f == b1)
                            nb_complete++;
                        if(f == b2 && d == b1)
                            nb_complete++;
                    }


                    // Si un seul des points n'est pas à l'intérieur de la face, alors error = true. Sinon, le triangle est valide => On peut le stocker
                    if(check_error_conv == false && check_error_cond == false && (volume_courant.size() == 1 || nb_complete > priorite))
                    {
                        volume_courant.push_back(T_test);
                        modif = true; // Signifie que l'on a modifier le volume courant => sortie des deux boucles for()
                        break;
                    }

                }

                counter++;
                if(modif)
                    break;
            }
            if(modif)
                break;

        }
        // priorité vaudra 0 si pas de solution du tout
        if(!modif)
            priorite--;
        else
            priorite = 2;
        for(int i = 0; i < arete_libre.size(); i++)
            std::cout << "sommet" << arete_libre[i] << std::endl;
        //sleep(1);
        // Et mettre à jour les arêtes libres
        // Parcourir une liste d'arêtes du volume pour mettre à jour arête libre
        // Puis sortir de la boucle si l'on fait ça
        // regroupe toute les arêtes du volume courant


        //
        // Construction du tableau d'arête

        ind_aretes.clear();
        for(TriangleGeo T : volume_courant)
        {
            int a = T.connectivity_[0];
            int b = T.connectivity_[1];
            int c = T.connectivity_[2];
            ind_aretes.push_back(a);
            ind_aretes.push_back(b);
            ind_aretes.push_back(b);
            ind_aretes.push_back(c);
            ind_aretes.push_back(c);
            ind_aretes.push_back(a);
            std::cout << a << "  " << b << "  " << c << std::endl;
        }

        //
        // Parcourt de ce tableau d'arête pour trouver les doublons
        arete_libre.clear();
        for(int w = 0; w < ind_aretes.size(); w++)
        {
            int s1 = ind_aretes[w];
            w++;
            int s2 = ind_aretes[w];

            // Pour tout couple s1 et s2, checher dans la liste un équivalent s2 & s1
            bool exist = false;
            for(int j = 0; j < ind_aretes.size(); j++)
            {
                if(s2 == ind_aretes[j])
                {
                    //std::cout << " found a match" << std::endl;
                    j++;
                    if(s1 == ind_aretes[j])
                    {
                        std::cout << " found a second match" << std::endl;
                        exist = true;
                    }
                }
                else
                    j++;
            }

            // Dans le cas où l'on a pas trouvé de doublon, c'est que l'arête est libre
            if(exist == false)
            {
                arete_libre.push_back(s1);
                arete_libre.push_back(s2);
            }
        }
        if(arete_libre.empty())
        {
            std::cout << "sortie boucle" << std::endl;
            closed = true;
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

            bool cond1 = a/(TYPE_PRIMITIVE + 1) == j;
            bool cond2 = b/(TYPE_PRIMITIVE + 1) == j;
            bool cond3 = c/(TYPE_PRIMITIVE + 1) == j;

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


std::vector<int> Intersection::ComputeConnectivity9()
{

    std::vector<int> output;

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
    // Test pour voir si l'on est bien sur la sphère
    //

    /*
    Vec3 sum = {0.0, 0.0, 0.0};
    for(Vec3 pts : points_proj)
    {
        Vec3 res = Icentre - pts;
        std::cout << "la distance est : " << res.norm() << std::endl;
    }*/


    //
    // Test de convexité des Nb_branches*TYPE_PRIMITIVE faces de départ
    //

    std::vector<TriangleGeo> Triangles_connus;

    for(int m = 0; m < points_proj.size(); m++)
    {
        int ref_centre = int(m/(TYPE_PRIMITIVE + 1))*(TYPE_PRIMITIVE + 1);
        if(ref_centre != m)
        {
            int ind_next;
            if((m + 1)%(TYPE_PRIMITIVE + 1) != 0)
                ind_next = m + 1;
            else
                ind_next = ref_centre + 1;

            TriangleGeo TX(points_proj[ref_centre], points_proj[m], points_proj[ind_next], ref_centre, m, ind_next, -1);
            TX.OrientedNormal(Icentre); // Icentre n'est peut être pas la bonne option => cas particulier ou le cenrte de l'intersection sort du polyèdre même après une projection
            Triangles_connus.push_back(TX);
        }
    }



    int counter = 0;
    bool ce = false;
    int indice_branche_fausse_1 = -1;
    int indice_branche_fausse_2 = -1;

    for(Vec3 proj : points_proj)
    {
        for(TriangleGeo TX_courant : Triangles_connus)
        {
            int a = TX_courant.connectivity_[0];
            int b = TX_courant.connectivity_[1];
            int c = TX_courant.connectivity_[2];

            if(counter != a && counter != b && counter != c)
            {
                double dist = TX_courant.normal_.dot(proj - TX_courant.sommets_[0]);
                //std::cout << "dist  " << dist <<std::endl;
                if(dist < -0.002)
                {
                    // Objectif : Trouver 2 bouts de branche fautifs.
                    if(indice_branche_fausse_1 == -1)
                        indice_branche_fausse_1 = int(TX_courant.connectivity_[0]/(TYPE_PRIMITIVE+1));
                    if(indice_branche_fausse_2 == -1 && indice_branche_fausse_1 != int(TX_courant.connectivity_[0]/(TYPE_PRIMITIVE+1)))
                        indice_branche_fausse_2 = int(TX_courant.connectivity_[0]/(TYPE_PRIMITIVE+1));

                    ce = true;
                }
            }
        }
        counter++;
    }


    if(ce)
    {
        std::cout << " Bad input data" << std::endl;



        // return l'indice de la branche à modifier et l'index de l'articulation à supprimer
        // On utilisera ensuite une méthode ModifyBranch pour supprimer les articulations en surplus.
        // (Défaut si l'on simplifie trop le squelette)
        // Côté squelette, on modifiera les contours de l'intersection après avoir modifié les branche
    }
    else
    {
        // signifiera que tout est valide
        output.push_back(-1);
        output.push_back(-1);
        std::cout << "good input data" << std::endl;
    }


    //
    //
    // Algorithme de création du volume convexe en utilisant des faces triangulaires
    //
    //


    std::vector<TriangleGeo> volume_courant; // renvoi des indices correspondant à des points de contours_


    //
    // Construire la première face
    //

    //
    // Première face

    Vec3 p1 = points_proj[0];
    Vec3 p2 = points_proj[1];
    Vec3 p3 = points_proj[2];
    TriangleGeo T1(p1, p2, p3, 0, 1, 2, -1);

    //
    // Prendre un point d'une autre branche pour ensuite orienter la face puis la stocker

    Vec3 p4 = points_proj[points_proj.size() - 1];
    T1.OrientedNormal(p4); // Oriente la normale vers le point (produit scalaire positif)
    volume_courant.push_back(T1);


    //
    // Initialiser le tableau d'arête libre (vu qu'on travail avec un triangle pour l'instant, toute les arêtes sont libres => mais à généraliser plus tard, OU PAS)
    //

    std::vector<int> ind_aretes;
    std::vector<int> arete_libre;

    int a = T1.connectivity_[0];
    int b = T1.connectivity_[1];
    int c = T1.connectivity_[2];

    arete_libre.push_back(a);
    arete_libre.push_back(b);
    arete_libre.push_back(b);
    arete_libre.push_back(c);
    arete_libre.push_back(c);
    arete_libre.push_back(a);

    ind_aretes.push_back(a);
    ind_aretes.push_back(b);
    ind_aretes.push_back(b);
    ind_aretes.push_back(c);
    ind_aretes.push_back(c);
    ind_aretes.push_back(a);


    //
    // Algo de vérif de convexité des faces candidates => tester tte les faces possibles, pour chaque arête avec chaque points de point_proj d'une autre branche
    //

    // Tout ça doit ce passer dans une boucle while => algo de convergence

    bool closed = false;
    int indic_incre = 0;
    while(!closed)
    {
        int count_test = 0;
        bool modif = false;
        for(int s = 0; s < arete_libre.size(); s++)
        {
            //
            // On récupère les deux sommets de l'arête
            int s1 = arete_libre[s];
            s++;
            int s2 = arete_libre[s];

            //
            // On parcourt tout les points de notre ensemble
            int counter = 0;
            for(Vec3 v : points_proj)
            {
                if(counter != s1 && counter != s2)
                {
                    count_test++;

                    // Alors on peut construire une face
                    TriangleGeo T_test(points_proj[s2], points_proj[s1], v, s2, s1, counter ,-1);
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
                    int d = T_test.connectivity_[0];
                    int e = T_test.connectivity_[1];
                    int f = T_test.connectivity_[2];
                    // Premier test : test de convexité avec tt les autres points de l'ensemble (non coplanaire)
                    bool check_error_conv = false;
                    int count2 = 0; // mettre une condition sur le point_test que l'on instancie après
                    for(Vec3 point_test : points_proj)
                    {
                        // On ne veut pas tester avec un point de la même face
                        if(count2 != d && count2 != e && count2 != f)
                        {
                            double dist = T_test.normal_.dot(point_test - T_test.sommets_[0]); // Petit doute sur les cas particuliers
                            if(dist < 0)
                                check_error_conv = true;
                        }
                        count2++;
                    }

                    // Deuxième test : On regarde si notre face se complémente bien avec le sens des arêtes des faces imposées => verifie quelles sont bien orientées => un double check
                    bool check_error_cond = false;
                    for(int q = 0; q < 3; q++)
                    {
                        int v1 = T_test.connectivity_[q];
                        int v2;
                        if(q + 1 < 3)
                            v2 = T_test.connectivity_[q + 1];
                        else
                            v2 = T_test.connectivity_[0];
                        for(int u = 0; u < ind_aretes.size(); u++)
                        {
                            int a1 = ind_aretes[u];
                            u++;
                            int a2 = ind_aretes[u];
                            if(a1 == v1)
                            {
                                if(a2 == v2)
                                    check_error_cond = true;
                            }
                        }
                    }


                    // Si un seul des points n'est pas à l'intérieur de la face, alors error = true. Sinon, le triangle est valide => On peut le stocker
                    if(check_error_conv == false && check_error_cond == false)
                    {
                        volume_courant.push_back(T_test);
                        modif = true; // Signifie que l'on a modifier le volume courant => sortie des deux boucles for()
                        indic_incre++;
                        break;
                    }

                }

                counter++;
                if(modif)
                    break;
            }
            if(modif)
                break;

        }

        // Affichage arêtes restante
        /*
        for(int i = 0; i < arete_libre.size(); i++)
            std::cout << "sommet" << arete_libre[i] << std::endl;*/


        //
        // Construction du tableau d'arête

        ind_aretes.clear();
        for(TriangleGeo T : volume_courant)
        {
            int a = T.connectivity_[0];
            int b = T.connectivity_[1];
            int c = T.connectivity_[2];
            ind_aretes.push_back(a);
            ind_aretes.push_back(b);
            ind_aretes.push_back(b);
            ind_aretes.push_back(c);
            ind_aretes.push_back(c);
            ind_aretes.push_back(a);
        }

        //
        // Parcourt de ce tableau d'arête pour trouver les doublons
        arete_libre.clear();
        for(int w = 0; w < ind_aretes.size(); w++)
        {
            int s1 = ind_aretes[w];
            w++;
            int s2 = ind_aretes[w];

            // Pour tout couple s1 et s2, checher dans la liste un équivalent s2 & s1
            bool exist = false;
            for(int j = 0; j < ind_aretes.size(); j++)
            {
                if(s2 == ind_aretes[j])
                {
                    j++;
                    if(s1 == ind_aretes[j])
                    {
                        // Vérifie si l'on trouve un doublon
                        /*std::cout << " found a second match" << std::endl;*/
                        exist = true;
                    }
                }
                else
                    j++;
            }

            // Dans le cas où l'on a pas trouvé de doublon, c'est que l'arête est libre
            if(exist == false)
            {
                arete_libre.push_back(s1);
                arete_libre.push_back(s2);
            }
        }
        if(arete_libre.empty())
        {
            closed = true;
        }
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

        // Affichage des sommets de chaque triangle
        /*std::cout << "V0 " << a << "   V1 " << b << "   V2 " << c << std::endl;*/

        //
        // Ici, trouver de quelle branche provient la face à l'aide de a,b,c qui nous donnent les indices des sommets de contours_
        for(int j : branches_incidentes_)
        {
            bool cond1 = a/(TYPE_PRIMITIVE + 1) == j;
            bool cond2 = b/(TYPE_PRIMITIVE + 1) == j;
            bool cond3 = c/(TYPE_PRIMITIVE + 1) == j;

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
    /*
    for(TriangleGeo T : faces_)
        std::cout << "  T0: "  << T.num_branch_ << std::endl;*/

    return output;

}
