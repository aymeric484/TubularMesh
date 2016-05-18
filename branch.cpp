#include "branch.h"



Branch::Branch()
{

    //fixed
    /*
    Vec4 V40(0.0, 1.0, 1.1, 2.0);
    Vec4 V41(0.5, 1.1, 2.0, 2.0);
    Vec4 V42(0.6, 1.5, 3.2, 2.0);
    Vec4 V43(0.9, 1.7, 5.0, 2.0);
    Vec4 V44(2.0, 2.1, 6.3, 2.0);
    Vec4 V45(2.0, 3.0, 8.0, 2.0);
    Vec4 V46(2.0, 4.0, 8.9, 2.0);
    //Vec4 V47(4.0, 4.0, 9.0, 1.0);
    //Vec4 V48(2.0, 4.0, 10.0, 1.0);
    //Vec4 V49(2.0, 4.0, 12.0, 1.0);

    Vec4 V_externe_begin(0.0, 0.0, 0.0, 6.0);
    Vec4 V_externe_end(2.0, 4.4, 10.0, 2.0);

    articulation_externe_end_ = V_externe_end;
    articulation_externe_begin_ = V_externe_begin;


    articulations_.push_back(V40);
    articulations_.push_back(V41);
    articulations_.push_back(V42);
    articulations_.push_back(V43);
    articulations_.push_back(V44);
    articulations_.push_back(V45);
    articulations_.push_back(V46);
    //articulations_.push_back(V47);
    //articulations_.push_back(V48);
    //articulations_.push_back(V49);
    */

    //random
    /*
    Vec4 V_externe_begin(0.0, 0.0, 0.0, 6.0);
    Vec4 Vprec = V_externe_begin;
    Vec4 Vcourant;
    Vec4 Vrand;

    int rand1, rand2, rand3;
    double frac1, frac2, frac3;


    srand(time(NULL));
    int i_end = rand() % 10 + 3;
    for(int i = 0; i < 80; i++ )
    {

        rand1=(rand() % 20000 + 200);
        frac1 = 1/(double)rand1*2000;
        rand2=(rand() % 20000 + 200);
        frac2 = 1/(double)rand2*2000;
        rand3=(rand() % 20000 + 200);
        frac3 = 1/(double)rand3*2000;
        Vrand[0] = frac1;
        Vrand[1] = frac2;
        Vrand[2] = frac3 + 5;
        Vrand[3] = 2;
        Vcourant = Vprec + Vrand;
        Vcourant[3] = 6;
        std::cout << " Point numero " << i << "  X =" << Vcourant[0] << "  Y =" << Vcourant[1] << "  Z =" << Vcourant[2] << std::endl;
        articulations_.push_back(Vcourant);
        Vprec = Vcourant;

    }

    rand1=(rand() % 20000 + 200);
    frac1 = 1/(double)rand1*2000;
    rand2=(rand() % 20000 + 200);
    frac2 = 1/(double)rand2*2000;
    rand3=(rand() % 20000 + 200);
    frac3 = 1/(double)rand3*2000;
    Vrand[0] = frac1;
    Vrand[1] = frac2;
    Vrand[2] = frac3 + 5;
    Vrand[3] = 2;
    articulation_externe_end_ = Vcourant + Vrand;
    articulation_externe_end_[3]= 6;
    articulation_externe_begin_ = V_externe_begin;
    //articulation_externe_end_ = V_externe_end;
    */


    //true
    /*

    Vec4 V43(157.529, 165.104, 143.265, 10.6542);//
    Vec4 V44(156.741, 165.766, 143.338, 10.6164);//
    Vec4 V45(156.164, 166.293, 143.566, 10.5596);//
    Vec4 V46(155.589, 166.821, 143.805, 10.5028);//
    Vec4 V47(154.999, 167.335, 144.084, 10.4461);//
    Vec4 V48(154.394, 167.836, 144.413, 10.3893);//
    Vec4 V49(153.791, 168.344, 144.778, 10.3325);//
    Vec4 V4a(153.204, 168.877, 145.158, 10.2947);//
    Vec4 V4b(152.632, 169.431, 145.541, 10.219);//

    Vec4 V4c(151.759, 169.569, 145.936, 10.5637);//
    Vec4 V4d(150.929, 169.761, 146.285, 10.5154);//
    Vec4 V4e(150.175, 170.046, 146.552, 10.4429);//
    Vec4 V4f(149.506, 170.437, 146.726, 10.3703);//
    Vec4 V4g(148.916, 170.927, 146.819, 10.2978);//
    Vec4 V4h(148.341, 171.436, 146.907, 10.287);//
    // 14 volumes reliant ces 15 faces

    Vec4 V_externe_begin(158.567, 164.244, 143.404, 11.5886);
    Vec4 V_externe_end(147.713, 171.881, 147.066, 10.343);

    articulation_externe_end_ = V_externe_end;
    articulation_externe_begin_ = V_externe_begin;

    articulations_.push_back(V43);
    articulations_.push_back(V44);
    articulations_.push_back(V45);
    articulations_.push_back(V46);
    articulations_.push_back(V47);
    articulations_.push_back(V48);
    articulations_.push_back(V49);
    articulations_.push_back(V4a);
    articulations_.push_back(V4b);
    articulations_.push_back(V4c);
    articulations_.push_back(V4d);
    articulations_.push_back(V4e);
    articulations_.push_back(V4f);
    articulations_.push_back(V4g);
    articulations_.push_back(V4h);
    */


    Vec4 V40(158.567, 164.244, 143.404, 11.5886);
    Vec4 V41(152.632, 169.431, 145.541, 10.219);
    Vec4 V42(132.136, 178.481, 150.965, 10.1528);
    Vec4 V43(128.815, 180.92, 149.244, 10.1195);
    Vec4 V44(106.709, 182.813, 165.666, 6.40015);
    Vec4 V45(98.8012, 184.416, 172.686, 6.34969);
    Vec4 V46(96.7544, 192.683, 181.163, 4.54742);
    Vec4 V47(95.2864, 200.203, 186.224, 4.33793);
    Vec4 V48(96.3256, 209.397, 195.3, 3.96686);
    Vec4 V49(101.788, 216.014, 201.224, 3.20456);
    Vec4 V4a(99.8711, 239.244, 202.948, 2.58874);
    Vec4 V4b(102.484, 244.174, 205.795, 2.47286);
    Vec4 V4c(109.035, 245.666, 209.102, 1.63297);
    Vec4 V4d(121.965, 254.711, 212.793, 1.45202);
    Vec4 V4e(120.023, 259.572, 216.045, 1.00817);
    // 14 volumes reliant ces 15 faces

    Vec4 V_externe_begin(175.259, 157.97, 138.061, 11.5886);
    Vec4 V_externe_end(120.415, 258.765, 223.422, 0.7);

    articulation_externe_end_ = V_externe_end;
    articulation_externe_begin_ = V_externe_begin;

    articulations_.push_back(V40);
    articulations_.push_back(V41);
    articulations_.push_back(V42);
    articulations_.push_back(V43);
    articulations_.push_back(V44);
    articulations_.push_back(V45);
    articulations_.push_back(V46);
    articulations_.push_back(V47);
    articulations_.push_back(V48);
    articulations_.push_back(V49);
    articulations_.push_back(V4a);
    articulations_.push_back(V4b);
    articulations_.push_back(V4c);
    articulations_.push_back(V4d);
    articulations_.push_back(V4e);

    branch_size_= articulations_.size();

}

// surveiller l'appel à ComputeMatrixFromBranch
void Branch::SubdiBranch(const double& seuil)
{

    std::vector<Vec4> points_inter; // vecteur intermediaire de stockage de points à ajouter à la branche
    std::vector<unsigned int> pos;  // indice où chacun de ces points doit être ajouté

    unsigned int decalage;          // indice ou inserer
    unsigned int size_courbure;
    unsigned int size_indices;
    unsigned int offset;

    Vec4 joint;     // variable intermediaire pour le remplissage de points_inter
    Vec4 nouv_arti; // variable intermediaire pour le remplissage de articulations_

    Vec3 AB; // segment central
    Vec3 AA; // tangente precedant A
    Vec3 BB; // opposé de tangente suivant B

    bool modif = true; // pour rentrer dans la boucle
    ComputeMatrixFromBranch(); // on utilisera notamment la courbure, calculée par cette méthode


    // Tant qu'il y a des modifications dans la boucle, on reparcourt les courbures
    while(modif == true)
    {
        modif = false; // si pas de modification, on quittera la boucle
        offset = 0; // il semble judicieux de ne vérifier qu'une courbure sur 2, d'où la variable offset

        // On parcourt toute les courbures d'indice paire (offset = 0) ou impaire (offset = 1)
        while(offset < 2)
        {

            // réinitialisation des vectors intermédiaires et calcul de la nouvelle taille de courbure_[]
            pos.clear();
            points_inter.clear();
            size_courbure = courbure_.size();


            // test de courbure en i=0, en utilisant un point externe à la branche (le point d'indice -1) pour le calcul du point interpolé
            if(courbure_[0] > seuil && offset == 0)
            {
                // Segment [AB] que l'on veut détruire par subdivision
                AB = articulations_[1].head<3>() -articulations_[0].head<3>();

                // Calcul de la tangente arrivant sur A, que l'on normalise car rapport à la norme de [AB] pour quelle soit du meme ordre de grandeur
                AA = articulations_[0].head<3>() -articulation_externe_begin_.head<3>();
                AA.normalize();
                AA = AA*AB.norm();

                // Calcul de la tangente arrivant sur B, que l'on normalise car rapport à la norme de [AB] pour quelle soit du meme ordre de grandeur
                BB = articulations_[1].head<3>() -articulations_[2].head<3>();
                BB.normalize();
                BB = BB*AB.norm();

                // interpolation spline cubique pour les coordonées XYZ
                joint[0] = articulations_[0][0] + AB[0]/2 + AA[0]/16 - BB[0]/16;
                joint[1] = articulations_[0][1] + AB[1]/2 + AA[1]/16 - BB[1]/16;
                joint[2] = articulations_[0][2] + AB[2]/2 + AA[2]/16 - BB[2]/16;

                // interpolation linéaire pour le rayon
                joint[3] = (articulations_[0][3] + articulations_[1][3])/2;

                std::cout<< " c =" << courbure_[0]<< " indice 0 " << std::endl;
                points_inter.push_back(joint);
                pos.push_back(0);
            }


            // test de courbure en i, de 2 en 2, en calculant ensuite un point interpolant si besoin
            for(unsigned int i= 2 - offset; i < size_courbure - 2; i=i+2)
            {
                if(courbure_[i] > seuil)
                {
                    // Segment [AB] que l'on veut détruire par subdivision
                    AB = articulations_[i+1].head<3>() -articulations_[i].head<3>();

                    // Calcul de la tangente arrivant sur A, que l'on normalise car rapport à la norme de [AB] pour quelle soit du meme ordre de grandeur
                    AA = articulations_[i].head<3>() -articulations_[i-1].head<3>();
                    AA.normalize();
                    AA = AA*AB.norm();

                    // Calcul de la tangente arrivant sur B, que l'on normalise car rapport à la norme de [AB] pour quelle soit du meme ordre de grandeur
                    BB = articulations_[i+1].head<3>() -articulations_[i+2].head<3>();
                    BB.normalize();
                    BB = BB*AB.norm();

                    // interpolation spline cubique pour les coordonées XYZ
                    joint[0] = articulations_[i][0] + AB[0]/2 + AA[0]/16 - BB[0]/16;
                    joint[1] = articulations_[i][1] + AB[1]/2 + AA[1]/16 - BB[1]/16;
                    joint[2] = articulations_[i][2] + AB[2]/2 + AA[2]/16 - BB[2]/16;

                    // interpolation linéaire pour le rayon
                    joint[3] = (articulations_[i][3] + articulations_[i+1][3])/2;

                    std::cout<< " c =" << courbure_[i] << "  indice "<< i << std::endl;
                    points_inter.push_back(joint);
                    pos.push_back(i);
                }
            }

            // test de courbure en i = imax, en utilisant un point externe à la branche (le point d'indice imax + 1) pour le calcul du point interpolé
            if(((size_courbure) %2 && offset == 0) || ((size_courbure + 1) %2 && offset == 1) )
            {
                if(courbure_[size_courbure - 2] > seuil)
                {
                    // Segment [AB] que l'on veut détruire par subdivision
                    AB = articulations_[size_courbure - 1].head<3>() -articulations_[size_courbure - 2].head<3>();

                    // Calcul de la tangente arrivant sur A, que l'on normalise car rapport à la norme de [AB] pour quelle soit du meme ordre de grandeur
                    AA = articulations_[size_courbure - 2].head<3>() -articulations_[size_courbure - 3].head<3>();
                    AA.normalize();
                    AA = AA*AB.norm();

                    // Calcul de la tangente arrivant sur B, que l'on normalise car rapport à la norme de [AB] pour quelle soit du meme ordre de grandeur
                    BB = articulations_[size_courbure - 1].head<3>() -articulation_externe_end_.head<3>();
                    BB.normalize();
                    BB = BB*AB.norm();

                    // interpolation spline cubique pour les coordonées XYZ
                    joint[0] = articulations_[size_courbure - 2][0] + AB[0]/2 + AA[0]/16 - BB[0]/16;
                    joint[1] = articulations_[size_courbure - 2][1] + AB[1]/2 + AA[1]/16 - BB[1]/16;
                    joint[2] = articulations_[size_courbure - 2][2] + AB[2]/2 + AA[2]/16 - BB[2]/16;

                    // interpolation linéaire pour le rayon
                    joint[3] = (articulations_[size_courbure - 2][3] + articulations_[size_courbure - 1][3])/2;

                    std::cout<< " c =" << courbure_[size_courbure -2] << " dernier indice "<< std::endl;
                    points_inter.push_back(joint);
                    pos.push_back(size_courbure - 2);
                }
            }

            size_indices = pos.size(); // la position a changé et doit être recalculée pour la suite

            // insertion dans ma branche, au bon indice, de tout les points interpolé
            for(unsigned int j = 0; j < size_indices ; j++ )
            {
                modif = true; // true car on modifie la branche
                decalage = pos[size_indices - 1 - j] + 1; // indice où placer le nouveau point
                nouv_arti = points_inter[size_indices - 1 - j]; // nouveau point à inserer
                articulations_.insert(articulations_.begin() + decalage, nouv_arti);
                std::cout<< " itération "<< j << "  X  "<< nouv_arti[0] << "  Y  "<< nouv_arti[1] << "  Z  "<< nouv_arti[2] << "  R  "<< nouv_arti[3] << std::endl;

            }

            offset++; // passage à 1 => indice impaire vont être testés ou à 2 => sortie de boucle car tout a été testé

            // mise à jour des axis + courbure si on a ajouté un point
            if(modif)
                ComputeMatrixFromBranch();

        }
    }

    // Version avec les itérator (incomplète)
    /*
    while(change)
    {
        std::cout << "b " << loop_count << std::endl;

        ComputeMatrixFromBranch();
        //std::vector<Vec4>::iterator itb = articulations_.begin();
        //std::vector<Vec4>::iterator ite = articulations_.end();
        change=false;
        count= 0;
        offset = 1;

        std::cout << "b "  << loop_count << std::endl;


        for(std::vector<Vec4>::iterator it = itb; it < ite - 1; ++it)
        {

            //std::cout<< " x " << (*(it))[0] << " y " << (*(it))[1] << " z " << (*(it))[2] << " r " << (*(it))[3]<<std::endl;
            if(offset==2)
                std::cout << *(it+1) << std::endl;

            if(courbure_[count] > seuil)
            {

                //recuperation des coordonées des 4 points nécessaire à l'interpolation
                if(it == itb)
                    coord_xyz_prec = { articulation_externe_begin_[0], articulation_externe_begin_[1], articulation_externe_begin_[2]};
                else
                    coord_xyz_prec = { (*(it-offset))[0], (*(it-offset))[1], (*(it-offset))[2] };

                coord_xyz_cour = { (*(it))[0], (*(it))[1], (*(it))[2] };
                coord_xyz_suiv = { (*(it+1))[0], (*(it+1))[1], (*(it+1))[2] };

                if(it == ite - 2)
                    coord_xyz_suiv2 = { articulation_externe_end_[0], articulation_externe_end_[1], articulation_externe_end_[2]};
                else
                    coord_xyz_suiv2 = { (*(it+2))[0], (*(it+2))[1], (*(it+2))[2] };

                //création du point sur la courbe hermitienne (interpolation spline cubique)
                nouveau_point[0] = (9*coord_xyz_cour[0] - coord_xyz_prec[0] + 9*coord_xyz_suiv[0] - coord_xyz_suiv2[0])/16;
                nouveau_point[1] = (9*coord_xyz_cour[1] - coord_xyz_prec[1] + 9*coord_xyz_suiv[1] - coord_xyz_suiv2[1])/16;
                nouveau_point[2] = (9*coord_xyz_cour[2] - coord_xyz_prec[2] + 9*coord_xyz_suiv[2] - coord_xyz_suiv2[2])/16;


                // calcul rayon moyen puis affectation à la nouvelle articulation (interpolation linéaire)
                nouveau_point[3] = ((*it)[3] +(*(it+1))[3])/2;

                //insertion du point après l'itérateur
                articulations_.insert(it+1, nouveau_point);

                //on se place sur le nouveau point et on prend en compte le décalage subit par it-1 à cause de l'insertion
                it++;
                ite++;


                offset=2;
                change=true;

            }
            else { offset = 1;}

            count++;

        }*/

}

void Branch::CreateCircleCoordinates(const unsigned int& primitive_size)
{

    //
    //
    // Initialisation
    //
    //


    Eigen::Matrix3d RI; // rotation inverse de XYZ à NBT

    Vec4 coord4_int;
    Vec3 B_projection_prec;     // pour le calcul de la projection de la bitangente precedente
    Vec3 dB_prec;               // variation de B entre i-1 et i

    double TermeN, TermeB, TermeT;
    double sum = 0;
    double torsion_prec;
    double Theta_prec;

    ComputeMatrixFromBranch();

    for(int i = 0; i < branch_size_; i++)
    {

        //
        //
        // Calcul de l'angle de torsion en i
        //
        //

        // Calcul de la matrice de rotation de xyz à NBT en chaque point (pour le calcul d'angle de torsion principalement)
        RI << N_axis_[i][0], N_axis_[i][1], N_axis_[i][2],
              B_axis_[i][0], B_axis_[i][1], B_axis_[i][2],
              T_axis_[i][0], T_axis_[i][1], T_axis_[i][2] ;


        // Calcul de l'angle de torsion en chaque points
        if(i != 0)
        {
            //calcul de la projection sur ( O, N, B) de la bitangente précédente
            B_projection_prec = RI*B_axis_[i-1];
            B_projection_prec[2] = 0;
            B_projection_prec.normalize();

            // Bitangente courante - Bitangente précédente => {0,1,0} - {Bn,Bb,0} , avec Bn et Bb composantes de la projetion sur (O,N,B) de B[i-1],
            dB_prec = - B_projection_prec;
            dB_prec[1] = 1 + dB_prec[1];
            torsion_prec = dB_prec.norm();

            // Calcul d'angle
            // Le terme de droite provient d'un produit mixte (B(i-1)proj ^ (0,1,0)) . (0,0,1) et nous donne un signe => un sens de rotation
            Theta_prec = 2 * asin( torsion_prec/2 ) * dB_prec[0]/ fabs(dB_prec[0]);

            theta_.push_back(Theta_prec);
        }
        else { theta_.push_back(0.0); } // premier terme de theta est forcement nul, car pas de B[i-1] pour lui



        //
        //
        // Calcul des coordonnées des points de chaque face de notre maillage
        //
        //

        sum = sum + theta_[i];
        for(int j = 0; j < primitive_size; j++)
        {
            // on se place sur un cercle de centre {0,0,0} et de rayon "articulations_[i][3]", on fait une rotation d'angle "sum" pour compenser la torsion
            TermeN = std::cos(2*Pi/primitive_size*j + sum/**/)*articulations_[i][3];
            TermeB = std::sin(2*Pi/primitive_size*j + sum/**/)*articulations_[i][3];
            TermeT = 0;
            coord4_int = {TermeN, TermeB, TermeT, 1}; //coordonées dans le repère local
            coord4_int = NBT_to_xyz_[i]*coord4_int; // coordonées dans le repère d'origine
            // std::cout << "local : N " <<  TermeN  << " B "<< TermeB << " T "<< TermeT << std::endl;
            // std::cout << "global : X " <<  coord4_int[0]  << " Y "<< coord4_int[1] << " Z "<< coord4_int[2] << std::endl;
            pos_vertices_.push_back(coord4_int.head<3>());
        }

    }
}


void Branch::ComputeMatrixFromBranch()
{
    //
    //
    // Initialisation
    //
    //

    branch_size_= articulations_.size();
    NBT_to_xyz_.clear();
    T_axis_.clear();
    N_axis_.clear();
    B_axis_.clear();
    courbure_.clear();
    theta_.clear();

    Vec3 normalized_tan_prec;   // tangente du segment precedent
    Vec3 normalized_tan_suiv;   // tangente du segment suivant
    Vec3 normalized_tan_moye;   // pour la tangente
    Vec3 normalized_nor;        // pour la normale
    Vec3 normalized_vec3;       // pour la bitangente


    // M(indice_ligne, indice_colonne) pour get/set les valeurs
    Eigen::Matrix4d M;  // matrice de changement de repère (rotation + translation) NBT au coordonées XYZ


    //
    //
    // Calcul des axes TNB en chaque point de la branch
    //
    //

    // normalisation de la tangente T(-1)
    normalized_tan_prec = articulations_[0].head<3>() - articulation_externe_begin_.head<3>();
    normalized_tan_prec.normalize();
    // normalisation de la tangente T(0)
    normalized_tan_suiv = articulations_[1].head<3>() - articulations_[0].head<3>();
    normalized_tan_suiv.normalize();
    // Calcul de la tangente moyenne TM(0)
    normalized_tan_moye = (normalized_tan_prec + normalized_tan_suiv )/2;
    normalized_tan_moye.normalize();
    T_axis_.push_back(normalized_tan_moye);
    // Calcul de la normale N(0) + normalisation et calcul de la courbure
    normalized_nor = normalized_tan_suiv - normalized_tan_prec;
    courbure_.push_back(normalized_nor.norm());
    normalized_nor = normalized_nor/courbure_[0];
    N_axis_.push_back(normalized_nor);

    for( int i = 1; i < branch_size_ - 1; i++ )
    {
        // normalisation de la tangente T(i-1)
        normalized_tan_prec = articulations_[i].head<3>() - articulations_[i-1].head<3>();
        normalized_tan_prec.normalize();
        // normalisation de la tangente T(i)
        normalized_tan_suiv = articulations_[i+1].head<3>() - articulations_[i].head<3>();
        normalized_tan_suiv.normalize();
        // Calcul de la tangente moyenne TM(i)
        normalized_tan_moye = (normalized_tan_prec + normalized_tan_suiv )/2;
        normalized_tan_moye.normalize();
        T_axis_.push_back(normalized_tan_moye);
        // Calcul de la normale + normalisation et calcul de la courbure
        normalized_nor = normalized_tan_suiv - normalized_tan_prec;
        courbure_.push_back(normalized_nor.norm());
        normalized_nor = normalized_nor/courbure_[i];
        N_axis_.push_back(normalized_nor);
    }

    // normalisation de la tangente T(size -2)
    normalized_tan_prec = articulations_[branch_size_-1].head<3>() - articulations_[branch_size_-2].head<3>();
    normalized_tan_prec.normalize();
    // normalisation de la tangente T(size -1)
    normalized_tan_suiv = articulation_externe_end_.head<3>() - articulations_[branch_size_-1].head<3>();
    normalized_tan_suiv.normalize();
    // Calcul de la tangente moyenne TM(size -1b)
    normalized_tan_moye = (normalized_tan_prec + normalized_tan_suiv )/2;
    normalized_tan_moye.normalize();
    T_axis_.push_back(normalized_tan_moye);
    // Calcul de la normale N(size-1) + normalisation et calcul de la courbure
    normalized_nor = normalized_tan_suiv - normalized_tan_prec;
    courbure_.push_back(normalized_nor.norm());
    normalized_nor = normalized_nor/courbure_[branch_size_ -1];
    N_axis_.push_back(normalized_nor);

    // Calcul de B(i) comme produit vectoriel de T(i) et N(i)
    for( int i = 0 ; i < T_axis_.size() ; i++ )
    {
        normalized_vec3 = T_axis_[i].cross(N_axis_[i]);
        normalized_vec3.normalize();
        B_axis_.push_back(normalized_vec3);
    }


    //
    //
    // Creation des matrices de changement de repère
    //
    //

    // Le repère est (N,B,T) ; le cercle se trouvera dans le plan (articulation[i], N[i], B(i))
    for(int i = 0; i < branch_size_; i++)
    {
        // empecher l'inversion du repère lorsque la normale s'inverse
        if(i != 0)
        {
            if(N_axis_[i].dot(N_axis_[i-1]) < 0)
            {
                N_axis_[i] = - N_axis_[i];
                B_axis_[i] = - B_axis_[i];
            }
        }

        // Remplissage de NBT_to_xyz (rotation + translation)
        M << N_axis_[i][0], B_axis_[i][0], T_axis_[i][0], articulations_[i][0],
             N_axis_[i][1], B_axis_[i][1], T_axis_[i][1], articulations_[i][1],
             N_axis_[i][2], B_axis_[i][2], T_axis_[i][2], articulations_[i][2],
                  0       ,      0       ,      0       ,      1               ;

        NBT_to_xyz_.push_back(M);

    }

    // Affichage des valeurs
    /*
    for(int i = 0; i < branch_size_; i++)
    {
        std::cout << " iteration " << i << std::endl;
        std::cout << " coord " << articulations_[i][0] << " ; " << articulations_[i][1] << " ; " << articulations_[i][2] << std::endl;
        std::cout << " tangente " << T_axis_[i][0] << " ; " << T_axis_[i][1] << " ; " << T_axis_[i][2] << std::endl;
        std::cout << " normale " << N_axis_[i][0] << " ; " << N_axis_[i][1] << " ; " << N_axis_[i][2] << std::endl;
        std::cout << " Bitangente " << B_axis_[i][0] << " ; " << B_axis_[i][1] << " ; " << B_axis_[i][2] << std::endl;
        std::cout << " courbure " << courbure_[i] << std::endl;
        std::cout << " theta " << theta_[i]*180/Pi << std::endl;
    }*/

}
