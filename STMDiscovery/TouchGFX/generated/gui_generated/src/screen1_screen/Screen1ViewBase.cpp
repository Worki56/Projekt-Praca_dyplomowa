/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <touchgfx/Color.hpp>
#include <images/BitmapDatabase.hpp>
#include <texts/TextKeysAndLanguages.hpp>

Screen1ViewBase::Screen1ViewBase() :
    radioButtonSelectedCallback(this, &Screen1ViewBase::radioButtonSelectedCallbackHandler),
    buttonCallback(this, &Screen1ViewBase::buttonCallbackHandler)
{
    __background.setPosition(0, 0, 480, 272);
    __background.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    add(__background);

    radioButtonGroup1.setRadioButtonSelectedHandler(radioButtonSelectedCallback);
    
    radioButton2.setXY(0, 237);
    radioButton2.setBitmaps(touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_MEDIUM_ROUNDED_OFF_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_MEDIUM_ROUNDED_OFF_PRESSED_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_MEDIUM_ROUNDED_ON_DARK_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_MEDIUM_ROUNDED_ON_PRESSED_ID));
    radioButton2.setSelected(false);
    radioButton2.setDeselectionEnabled(false);
    radioButtonGroup1.add(radioButton2);
    add(radioButton2);

    tA1_1.setXY(213, 187);
    tA1_1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    tA1_1.setLinespacing(0);
    tA1_1.setTypedText(touchgfx::TypedText(T___SINGLEUSE_EMPC));
    add(tA1_1);

    tA10_1.setPosition(320, 104, 145, 25);
    tA10_1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    tA10_1.setLinespacing(0);
    tA10_1Buffer[0] = 0;
    tA10_1.setWildcard(tA10_1Buffer);
    tA10_1.setTypedText(touchgfx::TypedText(T___SINGLEUSE_1806));
    add(tA10_1);

    tA10_0.setPosition(161, 104, 159, 25);
    tA10_0.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    tA10_0.setLinespacing(0);
    tA10_0Buffer[0] = 0;
    tA10_0.setWildcard(tA10_0Buffer);
    tA10_0.setTypedText(touchgfx::TypedText(T___SINGLEUSE_1DKF));
    add(tA10_0);

    tA7_2.setPosition(322, 80, 143, 25);
    tA7_2.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    tA7_2.setLinespacing(0);
    tA7_2Buffer[0] = 0;
    tA7_2.setWildcard(tA7_2Buffer);
    tA7_2.setTypedText(touchgfx::TypedText(T___SINGLEUSE_LOCP));
    add(tA7_2);

    tA7_1.setPosition(161, 79, 159, 25);
    tA7_1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    tA7_1.setLinespacing(0);
    tA7_1Buffer[0] = 0;
    tA7_1.setWildcard(tA7_1Buffer);
    tA7_1.setTypedText(touchgfx::TypedText(T___SINGLEUSE_EDAB));
    add(tA7_1);

    tA7_0.setPosition(0, 79, 150, 25);
    tA7_0.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    tA7_0.setLinespacing(0);
    tA7_0Buffer[0] = 0;
    tA7_0.setWildcard(tA7_0Buffer);
    tA7_0.setTypedText(touchgfx::TypedText(T___SINGLEUSE_BYTC));
    add(tA7_0);

    tA6_2.setPosition(320, 55, 145, 25);
    tA6_2.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    tA6_2.setLinespacing(0);
    tA6_2Buffer[0] = 0;
    tA6_2.setWildcard(tA6_2Buffer);
    tA6_2.setTypedText(touchgfx::TypedText(T___SINGLEUSE_WPIT));
    add(tA6_2);

    tA6_1.setPosition(160, 57, 160, 22);
    tA6_1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    tA6_1.setLinespacing(0);
    tA6_1Buffer[0] = 0;
    tA6_1.setWildcard(tA6_1Buffer);
    tA6_1.setTypedText(touchgfx::TypedText(T___SINGLEUSE_B633));
    add(tA6_1);

    tA6_0.setPosition(-1, 50, 150, 29);
    tA6_0.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    tA6_0.setLinespacing(0);
    tA6_0Buffer[0] = 0;
    tA6_0.setWildcard(tA6_0Buffer);
    tA6_0.setTypedText(touchgfx::TypedText(T___SINGLEUSE_JH5O));
    add(tA6_0);

    tA5.setXY(153, 0);
    tA5.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    tA5.setLinespacing(0);
    tA5.setTypedText(touchgfx::TypedText(T___SINGLEUSE_Z27A));
    add(tA5);

    tA3.setXY(0, 25);
    tA3.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    tA3.setLinespacing(0);
    tA3.setTypedText(touchgfx::TypedText(T___SINGLEUSE_I99L));
    add(tA3);

    tA2.setXY(160, 30);
    tA2.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    tA2.setLinespacing(0);
    tA2.setTypedText(touchgfx::TypedText(T___SINGLEUSE_PJEP));
    add(tA2);

    tA1.setXY(320, 30);
    tA1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    tA1.setLinespacing(0);
    tA1.setTypedText(touchgfx::TypedText(T___SINGLEUSE_0WJ3));
    add(tA1);

    radioButton1.setXY(444, 237);
    radioButton1.setBitmaps(touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_MEDIUM_ROUNDED_OFF_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_MEDIUM_ROUNDED_OFF_PRESSED_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_MEDIUM_ROUNDED_ON_DARK_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_MEDIUM_ROUNDED_ON_PRESSED_ID));
    radioButton1.setSelected(false);
    radioButton1.setDeselectionEnabled(false);
    radioButtonGroup1.add(radioButton1);
    add(radioButton1);

    toggleButton1.setXY(155, 213);
    toggleButton1.setBitmaps(touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_TOGGLEBUTTON_LARGE_ROUNDED_TEXT_ON_ACTIVE_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_TOGGLEBUTTON_LARGE_ROUNDED_TEXT_OFF_NORMAL_ID));
    toggleButton1.setAction(buttonCallback);
    add(toggleButton1);
}

Screen1ViewBase::~Screen1ViewBase()
{

}

void Screen1ViewBase::setupScreen()
{

}

void Screen1ViewBase::radioButtonSelectedCallbackHandler(const touchgfx::AbstractButton& src)
{
    if (&src == &radioButton1)
    {
        //Interaction1
        //When radioButton1 selected change screen to Screen3
        //Go to Screen3 with no screen transition
        application().gotoScreen3ScreenNoTransition();
    }
    if (&src == &radioButton2)
    {
        //Interaction2
        //When radioButton2 selected change screen to Screen2
        //Go to Screen2 with no screen transition
        application().gotoScreen2ScreenNoTransition();
    }
}

void Screen1ViewBase::buttonCallbackHandler(const touchgfx::AbstractButton& src)
{
    if (&src == &toggleButton1)
    {
        //Interaction3
        //When toggleButton1 clicked execute C++ code
        //Execute C++ code

    }
}
