#include "intersection.h"

Intersection::Intersection(int& ind)
{
    indicateur_ = ind;
}

void Intersection::ComputeConnectivity9()
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
    /*
    for(Vec3 point_courant : contours_)
    {
        Vec3 res = point_courant - Icentre;
        double distance = res.norm();
        double ratio = d_max/distance;
        points_proj.push_back(Icentre + res*ratio);
    }*/

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
    // Projection sur sphère avec extra transformation
    //

    for(int h = 0; h < contours_.size(); h++)
    {
        Vec3 point_courant = contours_[h];
        int ref_centre = int(h/(TYPE_PRIMITIVE + 1))*(TYPE_PRIMITIVE + 1);
        if(ref_centre != h)
            point_courant = point_courant + 0.99*(contours_[ref_centre] - point_courant);

        Vec3 res = point_courant - Icentre;
        double distance = res.norm();
        double ratio = d_max/distance;
        points_proj.push_back(Icentre + res*ratio);
    }

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
                if(dist < -0.02)
                    ce = true;
            }
        }
        counter++;
    }
    if(ce)
        std::cout << " Bad input data" << std::endl;

    //
    //
    // Détection et réparation de l'érreur de croisement des faces lors de la projection
    //
    //
    /*
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


    int ind_bad_point;
    int counter = 0;
    bool ce = false;

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
                if(dist < -0.02)
                {
                    ce = true;
                    ind_bad_point = counter;
                }
            }
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


}
