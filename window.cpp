#include "window.h"

Window::Window()
{
    createSpinBoxes();
    QHBoxLayout *layout = new QHBoxLayout;
    layout->addWidget(spinboxesgroup_);
    setLayout(layout);

}

void Window::createSpinBoxes()
{
    QSpinBox *integerSpinBox = new QSpinBox;
    integerSpinBox->setRange(3,128);
    integerSpinBox->setSingleStep(1);
    integerSpinBox->setValue(0);

    QVBoxLayout *spinBoxLayout = new QVBoxLayout;
    spinBoxLayout->addWidget(integerSpinBox);

    spinboxesgroup_ = new QGroupBox;
    spinboxesgroup_->setLayout(spinBoxLayout);

    connect(integerSpinBox, SIGNAL(valueChanged(int) ), this, SLOT(changePrecision(int)) );

}

void Window::changePrecision(int primitive)
{

    /*primitive_type_ = primitive;*/
    primitive_type_ = primitive;
    std::cout << primitive_type_ << std::endl;

}
