#ifndef WINDOW_H
#define WINDOW_H



#include <QSpinBox>
#include <QGroupBox>
#include <QVBoxLayout>
#include <cgogn/io/map_import.h>
#include "config.h"


class Window : public QWidget
{
    // possible grace à automoc ON dans le CMakelist et #include "main.moc" à la fin de ce fichier
    // produit un cpp contenant le code meta-objet necessaire à l'utilisation des signaux
    Q_OBJECT

public:
    Window();

    int primitive_type_;

public slots:
    void changePrecision(int primitive);

private:
    void createSpinBoxes();

    QGroupBox *spinboxesgroup_;
};

#endif // WINDOW_H
