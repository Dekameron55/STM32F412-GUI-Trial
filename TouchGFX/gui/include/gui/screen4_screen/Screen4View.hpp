#ifndef SCREEN4VIEW_HPP
#define SCREEN4VIEW_HPP

#include <gui_generated/screen4_screen/Screen4ViewBase.hpp>
#include <gui/screen4_screen/Screen4Presenter.hpp>

class Screen4View : public Screen4ViewBase
{
public:
    Screen4View();
    virtual ~Screen4View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:


    /*
     * Callback Declarations
     */
    touchgfx::Callback<Screen4View, const touchgfx::Slider&, int> sliderValueStartedChangeCallback;
    touchgfx::Callback<Screen4View, const touchgfx::Slider&, int> sliderValueChangedCallback;
    touchgfx::Callback<Screen4View, const touchgfx::Slider&, int> sliderValueConfirmedCallback;

    /*
     * Callback Handler Declarations
     */
    void sliderValueStartedChangeCallbackHandler(const touchgfx::Slider& src, int value);
    void sliderValueChangedCallbackHandler(const touchgfx::Slider& src, int value);
    void sliderValueConfirmedCallbackHandler(const touchgfx::Slider& src, int value);
};

#endif // SCREEN4VIEW_HPP
