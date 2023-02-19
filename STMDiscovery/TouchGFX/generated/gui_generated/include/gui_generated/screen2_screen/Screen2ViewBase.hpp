/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#ifndef SCREEN2VIEWBASE_HPP
#define SCREEN2VIEWBASE_HPP

#include <gui/common/FrontendApplication.hpp>
#include <mvp/View.hpp>
#include <gui/screen2_screen/Screen2Presenter.hpp>
#include <touchgfx/widgets/Box.hpp>
#include <touchgfx/widgets/RadioButton.hpp>
#include <touchgfx/widgets/RadioButtonGroup.hpp>
#include <touchgfx/widgets/TextAreaWithWildcard.hpp>
#include <touchgfx/widgets/TextArea.hpp>

class Screen2ViewBase : public touchgfx::View<Screen2Presenter>
{
public:
    Screen2ViewBase();
    virtual ~Screen2ViewBase();
    virtual void setupScreen();

protected:
    FrontendApplication& application() {
        return *static_cast<FrontendApplication*>(touchgfx::Application::getInstance());
    }

    /*
     * Member Declarations
     */
    touchgfx::Box __background;
    touchgfx::RadioButtonGroup<2> radioButtonGroup1;
    touchgfx::RadioButton radioButton2;
    touchgfx::TextAreaWithOneWildcard tA14;
    touchgfx::TextAreaWithOneWildcard tA13;
    touchgfx::TextAreaWithOneWildcard tA12;
    touchgfx::TextAreaWithOneWildcard tA11;
    touchgfx::TextAreaWithOneWildcard tA10;
    touchgfx::TextAreaWithOneWildcard tA9;
    touchgfx::TextAreaWithOneWildcard tA8;
    touchgfx::TextAreaWithOneWildcard tA7;
    touchgfx::TextAreaWithOneWildcard tA6;
    touchgfx::TextAreaWithOneWildcard tA5;
    touchgfx::TextAreaWithOneWildcard tA4;
    touchgfx::TextAreaWithOneWildcard tA3;
    touchgfx::TextAreaWithOneWildcard tA2;
    touchgfx::TextArea tA1;
    touchgfx::RadioButton radioButton1;

    /*
     * Wildcard Buffers
     */
    static const uint16_t TA14_SIZE = 20;
    touchgfx::Unicode::UnicodeChar tA14Buffer[TA14_SIZE];
    static const uint16_t TA13_SIZE = 20;
    touchgfx::Unicode::UnicodeChar tA13Buffer[TA13_SIZE];
    static const uint16_t TA12_SIZE = 20;
    touchgfx::Unicode::UnicodeChar tA12Buffer[TA12_SIZE];
    static const uint16_t TA11_SIZE = 20;
    touchgfx::Unicode::UnicodeChar tA11Buffer[TA11_SIZE];
    static const uint16_t TA10_SIZE = 20;
    touchgfx::Unicode::UnicodeChar tA10Buffer[TA10_SIZE];
    static const uint16_t TA9_SIZE = 20;
    touchgfx::Unicode::UnicodeChar tA9Buffer[TA9_SIZE];
    static const uint16_t TA8_SIZE = 20;
    touchgfx::Unicode::UnicodeChar tA8Buffer[TA8_SIZE];
    static const uint16_t TA7_SIZE = 20;
    touchgfx::Unicode::UnicodeChar tA7Buffer[TA7_SIZE];
    static const uint16_t TA6_SIZE = 20;
    touchgfx::Unicode::UnicodeChar tA6Buffer[TA6_SIZE];
    static const uint16_t TA5_SIZE = 20;
    touchgfx::Unicode::UnicodeChar tA5Buffer[TA5_SIZE];
    static const uint16_t TA4_SIZE = 20;
    touchgfx::Unicode::UnicodeChar tA4Buffer[TA4_SIZE];
    static const uint16_t TA3_SIZE = 20;
    touchgfx::Unicode::UnicodeChar tA3Buffer[TA3_SIZE];
    static const uint16_t TA2_SIZE = 20;
    touchgfx::Unicode::UnicodeChar tA2Buffer[TA2_SIZE];

private:

    /*
     * Callback Declarations
     */
    touchgfx::Callback<Screen2ViewBase, const touchgfx::AbstractButton&> radioButtonSelectedCallback;

    /*
     * Callback Handler Declarations
     */
    void radioButtonSelectedCallbackHandler(const touchgfx::AbstractButton& src);

};

#endif // SCREEN2VIEWBASE_HPP
