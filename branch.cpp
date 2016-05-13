#include "branch.h"


Branch::Branch()
{
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


    Vec4 V_externe_end(2.0, 4.4, 10.0, 2.0);


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


    Vec4 V_externe_begin(0.0, 0.0, 0.0, 6.0);
    Vec4 Vprec = V_externe_begin;
    Vec4 Vcourant;
    Vec4 Vrand;

    int rand1, rand2, rand3;
    double frac1, frac2, frac3;


    srand(time(NULL));
    int i_end = rand() % 10 + 3;
    for(int i = 0; i < 8; i++ )
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

    branch_size_= articulations_.size();

}

// surveiller l'appel à ComputeMatrixFromBranch
void Branch::SubdiBranch(const double& seuil)
{
    std::vector<Vec4> points_inter;
    std::vector<unsigned int> pos;

    unsigned int decalage;
    unsigned int size_courbure;
    unsigned int size_indices;
    unsigned int offset = 0;

    Vec4 joint;
    Vec4 nouv_arti;

    Vec3 AB; // segment central
    Vec3 AA; // tangente precedant A
    Vec3 BB; // opposé de tangente suivant B

    bool modif = true;
    ComputeMatrixFromBranch();
   /*
    Vec3 coord_xyz_prec;
    Vec3 coord_xyz_cour;
    Vec3 coord_xyz_suiv;
    Vec3 coord_xyz_suiv2;

    Vec4 nouveau_point;


    unsigned int count;
    unsigned int offset;
    //unsigned int incre = 0;
    bool change = true;
    unsigned int loop_count=0;*/


    // insert fonctionne avec un iterator; utiliser it.next() if it != myvector.end() si besoin

    // boucle avec moyenne des courbures de 2 points consecutifs comme condition
    /*
    for(std::vector<Vec4>::iterator it = articulations_.begin(); it < articulations_.end() - 1; ++it)
    {
        std::cout<< " iteration " << count << std::endl;
        std::cout<< " x " << (*(it))[0] << " y " << (*(it))[1] << " z " << (*(it))[2] << " r " << (*(it))[3]<<std::endl;

        if((courbure_[count] + courbure_[count + 1])/2 > seuil)
        {

            if(it == articulations_.begin())
                coord_xyz_prec = { articulation_externe_begin_[0], articulation_externe_begin_[1], articulation_externe_begin_[2]};
            else
                coord_xyz_prec = { (*(it-offset))[0], (*(it-offset))[1], (*(it-offset))[2] };

            coord_xyz_cour = { (*(it))[0], (*(it))[1], (*(it))[2] };
            coord_xyz_suiv = { (*(it+1))[0], (*(it+1))[1], (*(it+1))[2] };

            if(it == articulations_.end() - 2)
                coord_xyz_suiv2 = { articulation_externe_end_[0], articulation_externe_end_[1], articulation_externe_end_[2]};
            else
                coord_xyz_suiv2 = { (*(it+2))[0], (*(it+2))[1], (*(it+2))[2] };

            //norm_cour = ((coord_xyz_suiv - coord_xyz_cour).norm() + (coord_xyz_cour - coord_xyz_prec).norm())/2;
            //norm_suiv = ((coord_xyz_suiv2 - coord_xyz_suiv).norm() + (coord_xyz_suiv - coord_xyz_cour).norm())/2;

            // version normalisée qui semble fausse
            //nouveau_point = (coord_xyz_cour + coord_xyz_suiv + T_axis_[count]/4 - T_axis_[count + 1]/4)/2;

            // version dénormalisée par les facteurs norm_cour et norm_suiv
            //nouveau_point = (coord_xyz_cour + coord_xyz_suiv + T_axis_[count]/4*norm_cour - T_axis_[count + 1]/4*norm_suiv)/2;


            //version toute simple avec 4 points
            nouveau_point = 1/2*(coord_xyz_cour/8 - coord_xyz_prec/8 + 9*coord_xyz_suiv/8 - coord_xyz_suiv2/8);

            // calcul rayon moyen puis affectation à articulation
            ray = ((*it)[3] +(*(it+1))[3])/2;
            nouvelle_arti = { nouveau_point[0], nouveau_point[1], nouveau_point[2], ray};
            articulations_.insert(it+1, nouvelle_arti);
            it++;

            // on modifie une variable offset car l'articulation précédente vient d'être décalée de 1
            offset = 2;

        }
        else{ offset = 1; }

        //pour gérer des couples de points
        it++;
        count = count + 2;

    }*/


    while(modif == true)
    {
        modif = false;
        offset = 0;

        while(offset < 2)
        {

            pos.clear();
            points_inter.clear();

            size_courbure = courbure_.size();
            std::cout<< " nouveau squelette "<<std::endl;

            //il semble judicieux de ne vérifier qu'une courbure sur 2 pour l'instant, d'où la variable offset
            if(courbure_[0] > seuil && offset == 0)
            {
                AB = articulations_[1].head<3>() -articulations_[0].head<3>();
                AA = articulations_[0].head<3>() -articulation_externe_begin_.head<3>();
                AA.normalize();
                AA = AA*AB.norm();
                BB = articulations_[1].head<3>() -articulations_[2].head<3>();
                BB.normalize();
                BB = BB*AB.norm();

                joint[0] = articulations_[0][0] + AB[0]/2 + AA[0]/16 - BB[0]/16;
                joint[1] = articulations_[0][1] + AB[1]/2 + AA[1]/16 - BB[1]/16;
                joint[2] = articulations_[0][2] + AB[2]/2 + AA[2]/16 - BB[2]/16;

                /*
                joint[0] = (9*articulations_[0][0] - articulation_externe_begin_[0] + 9*articulations_[1][0] - articulations_[2][0])/16;
                joint[1] = (9*articulations_[0][1] - articulation_externe_begin_[1] + 9*articulations_[1][1] - articulations_[2][1])/16;
                joint[2] = (9*articulations_[0][2] - articulation_externe_begin_[2] + 9*articulations_[1][2] - articulations_[2][2])/16;
                joint[3] = (articulations_[0][3] + articulations_[1][3])/2;*/

                std::cout<< " c =" << courbure_[0]<< " indice 0 " << std::endl;
                points_inter.push_back(joint);
                pos.push_back(0);
            }


            for(unsigned int i= 2 - offset; i < size_courbure - 2; i=i+2)
            {
                if(courbure_[i] > seuil)
                {
                    AB = articulations_[i+1].head<3>() -articulations_[i].head<3>();
                    AA = articulations_[i].head<3>() -articulations_[i-1].head<3>();
                    AA.normalize();
                    AA = AA*AB.norm();
                    BB = articulations_[i+1].head<3>() -articulations_[i+2].head<3>();
                    BB.normalize();
                    BB = BB*AB.norm();

                    joint[0] = articulations_[i][0] + AB[0]/2 + AA[0]/16 - BB[0]/16;
                    joint[1] = articulations_[i][1] + AB[1]/2 + AA[1]/16 - BB[1]/16;
                    joint[2] = articulations_[i][2] + AB[2]/2 + AA[2]/16 - BB[2]/16;

                    /*
                    joint[0] = (9*articulations_[i][0] - articulations_[i-1][0] + 9*articulations_[i+1][0] - articulations_[i+2][0])/16;
                    joint[1] = (9*articulations_[i][1] - articulations_[i-1][1] + 9*articulations_[i+1][1] - articulations_[i+2][1])/16;
                    joint[2] = (9*articulations_[i][2] - articulations_[i-1][2] + 9*articulations_[i+1][2] - articulations_[i+2][2])/16;
                    */
                    joint[3] = (articulations_[i][3] + articulations_[i+1][3])/2;
                    std::cout<< " c =" << courbure_[i] << "  indice "<< i << std::endl;

                    points_inter.push_back(joint);
                    pos.push_back(i);
                }
            }

            if(((size_courbure) %2 && offset == 0) || ((size_courbure + 1) %2 && offset == 1) )
            {
                if(courbure_[size_courbure - 2] > seuil)
                {
                    AB = articulations_[size_courbure - 1].head<3>() -articulations_[size_courbure - 2].head<3>();
                    AA = articulations_[size_courbure - 2].head<3>() -articulations_[size_courbure - 3].head<3>();
                    AA.normalize();
                    AA = AA*AB.norm();
                    BB = articulations_[size_courbure - 1].head<3>() -articulation_externe_end_.head<3>();
                    BB.normalize();
                    BB = BB*AB.norm();

                    joint[0] = articulations_[size_courbure - 2][0] + AB[0]/2 + AA[0]/16 - BB[0]/16;
                    joint[1] = articulations_[size_courbure - 2][1] + AB[1]/2 + AA[1]/16 - BB[1]/16;
                    joint[2] = articulations_[size_courbure - 2][2] + AB[2]/2 + AA[2]/16 - BB[2]/16;

                    /*
                    joint[0] = (9*articulations_[size_courbure - 2][0] - articulations_[size_courbure - 3][0] + 9*articulations_[size_courbure - 1][0] - articulation_externe_end_[0])/16;
                    joint[1] = (9*articulations_[size_courbure - 2][1] - articulations_[size_courbure - 3][1] + 9*articulations_[size_courbure - 1][1] - articulation_externe_end_[1])/16;
                    joint[2] = (9*articulations_[size_courbure - 2][2] - articulations_[size_courbure - 3][2] + 9*articulations_[size_courbure - 1][2] - articulation_externe_end_[2])/16;
                    */

                    joint[3] = (articulations_[size_courbure - 2][3] + articulations_[size_courbure - 1][3])/2;

                    std::cout<< " c =" << courbure_[size_courbure -2] << " dernier indice "<< std::endl;
                    points_inter.push_back(joint);
                    pos.push_back(size_courbure - 2);
                }
            }

            /*
            unsigned int counter = 0;


            while(!pos.empty())
            {
                modif=true;

                size_indices = pos.size(); //pourrait être commenté

                decalage = pos[size_indices - 1];
                //decalage = pos.back();
                nouv_arti = points_inter[size_indices - 1];
                //nouv_arti = points_inter.back();

                pos.pop_back();
                points_inter.pop_back();

                articulations_.insert(articulations_.begin() + decalage, nouv_arti);

                std::cout<< " itération "<< counter << std::endl;
                counter++;
            }*/

            size_indices = pos.size();
    /*
            if(size_indices != 0)
                modif = true;*/

            for(unsigned int j = 0; j < size_indices ; j++ )
            {
                modif = true;
                decalage = pos[size_indices - 1 - j] + 1;
                nouv_arti = points_inter[size_indices - 1 - j];
                articulations_.insert(articulations_.begin() + decalage, nouv_arti);
                std::cout<< " itération "<< j << "  X  "<< nouv_arti[0] << "  Y  "<< nouv_arti[1] << "  Z  "<< nouv_arti[2] << "  R  "<< nouv_arti[3] << std::endl;

            }

            offset++;

            sleep(1);

            ComputeMatrixFromBranch();

        }


    }





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

        //loop_count++;


}


void Branch::CalculateTorsion()
{
    //premier terme n'a pas de torsion
    //(utiliser les tangentes 1/2)
    Vec3 normalized_dB_prec;
    Vec3 normalized_dB_suiv;
    Vec3 normalized_dB_moy; // correspondrait en théorie à la normale


    torsion_.clear();

    torsion_.push_back(0.0);

    for(int i = 1; i < branch_size_ - 1; i++)
    {
        normalized_dB_prec = B_axis_[i] - B_axis_[i-1];
        normalized_dB_prec.normalize();
        normalized_dB_suiv = B_axis_[i+1] - B_axis_[i];
        normalized_dB_suiv.normalize();
        normalized_dB_moy = (normalized_dB_prec + normalized_dB_suiv )/2;
        torsion_.push_back(sqrt(normalized_dB_moy[0]*normalized_dB_moy[0] + normalized_dB_moy[1]*normalized_dB_moy[1] + normalized_dB_moy[2]*normalized_dB_moy[2] ));
        std::cout << " iteration " << i << " et torsion " << torsion_[i] << std::endl;
        //normalized_dB_moy = normalized_dB_moy/torsion_[i];
        //std::cout << normalized_dB_moy[0]
    }

    torsion_.push_back(torsion_[branch_size_ - 2]);


    //puis recuperer la norm de se vecteur
    //pour verifier, afficher cette torsion
    //et comparer se vecteur à N_axis



}

void Branch::CreateTrianglesCoordinates(const unsigned int& primitive_size)
{
    Vec4 coord4_int;
    double TermeN, TermeB, TermeT;
    branch_size_ =  articulations_.size();

    for(int i = 0; i < branch_size_; i++)
    {
        for(int j = 0; j < primitive_size; j++)
        {
            TermeN = std::cos(2*Pi/primitive_size*j)*articulations_[i][3];
            TermeB = std::sin(2*Pi/primitive_size*j)*articulations_[i][3];
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
    branch_size_= articulations_.size();
    NBT_to_xyz_.clear();

    ComputeT();
    ComputeN();
    ComputeB();

    Eigen::Matrix4d M; // M(indice_ligne, indice_colonne) pour get/set les valeurs

    // Le repère est (N,B,T) ; le cercle se trouvera dans le plan (articulation[i], N[i], B(i))

    // empecher l'inversion du repère lorsque la normale s'inverse
    for(int i = 0; i < branch_size_; i++)
    {
        if(i != 0)
        {
            if(N_axis_[i].dot(N_axis_[i-1]) < 0)
            {
                N_axis_[i] = - N_axis_[i];
                B_axis_[i] = - B_axis_[i];
            }
        }


        M << N_axis_[i][0], B_axis_[i][0], T_axis_[i][0], articulations_[i][0],
             N_axis_[i][1], B_axis_[i][1], T_axis_[i][1], articulations_[i][1],
             N_axis_[i][2], B_axis_[i][2], T_axis_[i][2], articulations_[i][2],
                  0       ,      0       ,      0       ,      1               ;

        NBT_to_xyz_.push_back(M);
    }

    //se fait apres les computeT,N,B et l'inversion des axes N,B
    //CalculateTorsion();


    /*
    for(int i = 0; i < branch_size_; i++)
    {
        std::cout << " iteration " << i << std::endl;
        std::cout << " coord " << articulations_[i][0] << " ; " << articulations_[i][1] << " ; " << articulations_[i][2] << std::endl;
        std::cout << " tangente " << T_axis_[i][0] << " ; " << T_axis_[i][1] << " ; " << T_axis_[i][2] << std::endl;
        std::cout << " normale " << N_axis_[i][0] << " ; " << N_axis_[i][1] << " ; " << N_axis_[i][2] << std::endl;
        std::cout << " Bitangente " << B_axis_[i][0] << " ; " << B_axis_[i][1] << " ; " << B_axis_[i][2] << std::endl;
        std::cout << " courbure " << courbure_[i] << std::endl;
        //std::cout << " torsion " << torsion_[i] << std::endl;
    }*/


}

void Branch::ComputeT()
{
    /*
    Vec3 normalized_tan;

    normalized_tan = (articulations_[1].head<3>() - articulation_externe_begin_.head<3>())/2;
    normalized_tan = normalized_tan/sqrt(normalized_tan[0]*normalized_tan[0] + normalized_tan[1]*normalized_tan[1] + normalized_tan[2]*normalized_tan[2]);
    T_axis_.push_back(normalized_tan);

    for( int i = 1; i < branch_size_ - 1; i++ )
    {
        normalized_tan = (articulations_[i+1].head<3>() - articulations_[i-1].head<3>())/2;
        normalized_tan = normalized_tan/sqrt(normalized_tan[0]*normalized_tan[0] + normalized_tan[1]*normalized_tan[1] + normalized_tan[2]*normalized_tan[2]);
        T_axis_.push_back(normalized_tan);
    }

    normalized_tan = (articulation_externe_end_.head<3>() - articulations_[branch_size_-2].head<3>())/2;
    normalized_tan = normalized_tan/sqrt(normalized_tan[0]*normalized_tan[0] + normalized_tan[1]*normalized_tan[1] + normalized_tan[2]*normalized_tan[2]);
    T_axis_.push_back(normalized_tan);
    */

    T_axis_.clear();

    Vec3 normalized_tan_moye;
    Vec3 normalized_tan_prec;
    Vec3 normalized_tan_suiv;

    //normalisation de la tangente T(-1)
    normalized_tan_prec = articulations_[0].head<3>() - articulation_externe_begin_.head<3>();
    normalized_tan_prec = normalized_tan_prec/sqrt(normalized_tan_prec[0] * normalized_tan_prec[0] + normalized_tan_prec[1] * normalized_tan_prec[1] + normalized_tan_prec[2] * normalized_tan_prec[2]);
    //normalisation de la tangente T(0)
    normalized_tan_suiv = articulations_[1].head<3>() - articulations_[0].head<3>();
    normalized_tan_suiv = normalized_tan_suiv/sqrt(normalized_tan_suiv[0] * normalized_tan_suiv[0] + normalized_tan_suiv[1] * normalized_tan_suiv[1] + normalized_tan_suiv[2] * normalized_tan_suiv[2]);
    //Calcul de la tangente moyenne TM(0)
    normalized_tan_moye = (normalized_tan_prec + normalized_tan_suiv )/2;
    normalized_tan_moye.normalize();
    T_axis_.push_back(normalized_tan_moye);

    for( int i = 1; i < branch_size_ - 1; i++ )
    {
        //normalisation de la tangente T(i-1)
        normalized_tan_prec = articulations_[i].head<3>() - articulations_[i-1].head<3>();
        normalized_tan_prec = normalized_tan_prec/sqrt(normalized_tan_prec[0] * normalized_tan_prec[0] + normalized_tan_prec[1] * normalized_tan_prec[1] + normalized_tan_prec[2] * normalized_tan_prec[2]);
        //normalisation de la tangente T(i)
        normalized_tan_suiv = articulations_[i+1].head<3>() - articulations_[i].head<3>();
        normalized_tan_suiv = normalized_tan_suiv/sqrt(normalized_tan_suiv[0] * normalized_tan_suiv[0] + normalized_tan_suiv[1] * normalized_tan_suiv[1] + normalized_tan_suiv[2] * normalized_tan_suiv[2]);
        //Calcul de la tangente moyenne TM(i)
        normalized_tan_moye = (normalized_tan_prec + normalized_tan_suiv )/2;
        normalized_tan_moye.normalize();
        T_axis_.push_back(normalized_tan_moye);
    }
    //normalisation de la tangente T(size -2)
    normalized_tan_prec = articulations_[branch_size_-1].head<3>() - articulations_[branch_size_-2].head<3>();
    normalized_tan_prec = normalized_tan_prec/sqrt(normalized_tan_prec[0] * normalized_tan_prec[0] + normalized_tan_prec[1] * normalized_tan_prec[1] + normalized_tan_prec[2] * normalized_tan_prec[2]);
    //normalisation de la tangente T(size -1)
    normalized_tan_suiv = normalized_tan_suiv/sqrt(normalized_tan_suiv[0] * normalized_tan_suiv[0] + normalized_tan_suiv[1] * normalized_tan_suiv[1] + normalized_tan_suiv[2] * normalized_tan_suiv[2]);
    normalized_tan_suiv = articulation_externe_end_.head<3>() - articulations_[branch_size_-1].head<3>();
    //Calcul de la tangente moyenne TM(size -1b)
    normalized_tan_moye = (normalized_tan_prec + normalized_tan_suiv )/2;
    normalized_tan_moye.normalize();
    T_axis_.push_back(normalized_tan_moye);
}

void Branch::ComputeN()
{
    N_axis_.clear();
    courbure_.clear();

    Vec3 normalized_nor;
    Vec3 normalized_tan_prec;
    Vec3 normalized_tan_suiv;

    //normalisation de la tangente T(-1)
    normalized_tan_prec = articulations_[0].head<3>() - articulation_externe_begin_.head<3>();
    normalized_tan_prec = normalized_tan_prec/sqrt(normalized_tan_prec[0] * normalized_tan_prec[0] + normalized_tan_prec[1] * normalized_tan_prec[1] + normalized_tan_prec[2] * normalized_tan_prec[2]);
    //normalisation de la tangente T(0)
    normalized_tan_suiv = articulations_[1].head<3>() - articulations_[0].head<3>();
    normalized_tan_suiv = normalized_tan_suiv/sqrt(normalized_tan_suiv[0] * normalized_tan_suiv[0] + normalized_tan_suiv[1] * normalized_tan_suiv[1] + normalized_tan_suiv[2] * normalized_tan_suiv[2]);
    //Calcul de la normale N(0) + normalisation et calcul de la courbure
    normalized_nor = normalized_tan_suiv - normalized_tan_prec;
    courbure_.push_back(sqrt(normalized_nor[0]*normalized_nor[0] + normalized_nor[1]*normalized_nor[1] + normalized_nor[2]*normalized_nor[2]));
    normalized_nor = normalized_nor/courbure_[0];
    N_axis_.push_back(normalized_nor);

    for( int i = 1; i < branch_size_ - 1; i++ )
    {
        //normalisation de la tangente T(i-1)
        normalized_tan_prec = articulations_[i].head<3>() - articulations_[i-1].head<3>();
        normalized_tan_prec = normalized_tan_prec/sqrt(normalized_tan_prec[0] * normalized_tan_prec[0] + normalized_tan_prec[1] * normalized_tan_prec[1] + normalized_tan_prec[2] * normalized_tan_prec[2]);
        //normalisation de la tangente T(i)
        normalized_tan_suiv = articulations_[i+1].head<3>() - articulations_[i].head<3>();
        normalized_tan_suiv = normalized_tan_suiv/sqrt(normalized_tan_suiv[0] * normalized_tan_suiv[0] + normalized_tan_suiv[1] * normalized_tan_suiv[1] + normalized_tan_suiv[2] * normalized_tan_suiv[2]);
        //Calcul de la normale + normalisation et calcul de la courbure
        normalized_nor = normalized_tan_suiv - normalized_tan_prec;
        courbure_.push_back(sqrt(normalized_nor[0]*normalized_nor[0] + normalized_nor[1]*normalized_nor[1] + normalized_nor[2]*normalized_nor[2]));
        normalized_nor = normalized_nor/courbure_[i];
        N_axis_.push_back(normalized_nor);
    }

    //normalisation de la tangente T(size -2)
    normalized_tan_prec = articulations_[branch_size_ - 1].head<3>() - articulations_[branch_size_ - 2].head<3>();
    normalized_tan_prec = normalized_tan_prec/sqrt(normalized_tan_prec[0] * normalized_tan_prec[0] + normalized_tan_prec[1] * normalized_tan_prec[1] + normalized_tan_prec[2] * normalized_tan_prec[2]);
    //normalisation de la tangente T(size -1)
    normalized_tan_suiv = articulation_externe_end_.head<3>() - articulations_[branch_size_ - 1].head<3>();
    normalized_tan_suiv = normalized_tan_suiv/sqrt(normalized_tan_suiv[0] * normalized_tan_suiv[0] + normalized_tan_suiv[1] * normalized_tan_suiv[1] + normalized_tan_suiv[2] * normalized_tan_suiv[2]);
    //Calcul de la normale N(size-1) + normalisation et calcul de la courbure
    normalized_nor = normalized_tan_suiv - normalized_tan_prec;
    courbure_.push_back(sqrt(normalized_nor[0]*normalized_nor[0] + normalized_nor[1]*normalized_nor[1] + normalized_nor[2]*normalized_nor[2]));
    normalized_nor = normalized_nor/courbure_[branch_size_ -1];
    N_axis_.push_back(normalized_nor);

}

void Branch::ComputeB()
{
    /* Pour calculer la torsion, utiliser l egalité B' = -torsion_ *N
      Donc, avec B' = (B(i+1)-B(i-1))/2 => dérivée approximée
      */

    B_axis_.clear();


    Vec3 normalized_vec3;

    for( int i = 0 ; i < T_axis_.size() ; i++ )
    {

        normalized_vec3 = T_axis_[i].cross(N_axis_[i]);
        normalized_vec3 = normalized_vec3/sqrt(normalized_vec3[0]*normalized_vec3[0] + normalized_vec3[1]*normalized_vec3[1] + normalized_vec3[2]*normalized_vec3[2]);
        B_axis_.push_back(normalized_vec3);
    }

}
