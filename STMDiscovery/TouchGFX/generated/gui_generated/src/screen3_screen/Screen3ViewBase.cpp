/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/screen3_screen/Screen3ViewBase.hpp>
#include <touchgfx/Color.hpp>
#include <images/BitmapDatabase.hpp>
#include <texts/TextKeysAndLanguages.hpp>

Screen3ViewBase::Screen3ViewBase() :
    radioButtonSelectedCallback(this, &Screen3ViewBase::radioButtonSelectedCallbackHandler)
{
    __background.setPosition(0, 0, 480, 272);
    __background.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    add(__background);

    radioButtonGroup1.setRadioButtonSelectedHandler(radioButtonSelectedCallback);
    
    radioButton2.setXY(0, 238);
    radioButton2.setBitmaps(touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_MEDIUM_ROUNDED_OFF_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_MEDIUM_ROUNDED_OFF_PRESSED_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_MEDIUM_ROUNDED_ON_DARK_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_MEDIUM_ROUNDED_ON_PRESSED_ID));
    radioButton2.setSelected(false);
    radioButton2.setDeselectionEnabled(false);
    radioButtonGroup1.add(radioButton2);
    add(radioButton2);

    radioButton1.setXY(444, 238);
    radioButton1.setBitmaps(touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_MEDIUM_ROUNDED_OFF_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_MEDIUM_ROUNDED_OFF_PRESSED_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_MEDIUM_ROUNDED_ON_DARK_ID), touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_RADIOBUTTON_RADIO_MEDIUM_ROUNDED_ON_PRESSED_ID));
    radioButton1.setSelected(false);
    radioButton1.setDeselectionEnabled(false);
    radioButtonGroup1.add(radioButton1);
    add(radioButton1);

    tA1_1_1.setXY(100, 204);
    tA1_1_1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    tA1_1_1.setLinespacing(0);
    tA1_1_1.setTypedText(touchgfx::TypedText(T___SINGLEUSE_5JPQ));
    add(tA1_1_1);

    tA1_1.setXY(41, 168);
    tA1_1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    tA1_1.setLinespacing(0);
    tA1_1.setTypedText(touchgfx::TypedText(T___SINGLEUSE_A6LB));
    add(tA1_1);

    tA1.setXY(61, 130);
    tA1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    tA1.setLinespacing(0);
    tA1.setTypedText(touchgfx::TypedText(T___SINGLEUSE_UJF8));
    add(tA1);

    Image1.setXY(175, 0);
    Image1.setBitmap(touchgfx::Bitmap(BITMAP_AS_ID));
    add(Image1);
}

Screen3ViewBase::~Screen3ViewBase()
{

}

void Screen3ViewBase::setupScreen()
{

}

void Screen3ViewBase::radioButtonSelectedCallbackHandler(const touchgfx::AbstractButton& src)
{
    if (&src == &radioButton1)
    {
        //Interaction1
        //When radioButton1 selected change screen to Screen2
        //Go to Screen2 with no screen transition
        application().gotoScreen2ScreenNoTransition();
    }
    if (&src == &radioButton2)
    {
        //Interaction2
        //When radioButton2 selected change screen to Screen1
        //Go to Screen1 with no screen transition
        application().gotoScreen1ScreenNoTransition();
    }
}
