/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#ifndef SCREEN3VIEWBASE_HPP
#define SCREEN3VIEWBASE_HPP

#include <gui/common/FrontendApplication.hpp>
#include <mvp/View.hpp>
#include <gui/screen3_screen/Screen3Presenter.hpp>
#include <touchgfx/widgets/Box.hpp>
#include <touchgfx/widgets/RadioButton.hpp>
#include <touchgfx/widgets/RadioButtonGroup.hpp>
#include <touchgfx/widgets/TextArea.hpp>
#include <touchgfx/widgets/Image.hpp>

class Screen3ViewBase : public touchgfx::View<Screen3Presenter>
{
public:
    Screen3ViewBase();
    virtual ~Screen3ViewBase();
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
    touchgfx::RadioButton radioButton1;
    touchgfx::TextArea tA1_1_1;
    touchgfx::TextArea tA1_1;
    touchgfx::TextArea tA1;
    touchgfx::Image Image1;

private:

    /*
     * Callback Declarations
     */
    touchgfx::Callback<Screen3ViewBase, const touchgfx::AbstractButton&> radioButtonSelectedCallback;

    /*
     * Callback Handler Declarations
     */
    void radioButtonSelectedCallbackHandler(const touchgfx::AbstractButton& src);

};

#endif // SCREEN3VIEWBASE_HPP
