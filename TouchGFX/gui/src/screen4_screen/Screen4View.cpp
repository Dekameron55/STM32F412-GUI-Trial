#include <gui/screen4_screen/Screen4View.hpp>
#include <main.h>

Screen4View::Screen4View():
    	    sliderValueStartedChangeCallback(this, &Screen4View::sliderValueStartedChangeCallbackHandler),
    	    sliderValueChangedCallback(this, &Screen4View::sliderValueChangedCallbackHandler),
    	    sliderValueConfirmedCallback(this, &Screen4View::sliderValueConfirmedCallbackHandler)
{

}

void Screen4View::setupScreen()
{
    Screen4ViewBase::setupScreen();
    slider1.setStartValueCallback(sliderValueStartedChangeCallback);

    slider1.setValue(pwm_div);

    slider1.setNewValueCallback(sliderValueChangedCallback);
    slider1.setStopValueCallback(sliderValueConfirmedCallback);

    //Setup previous value of slider.
    Unicode::snprintf(textArea2Buffer, 10, "%d",pwm_div);
    textArea2.invalidate();
}

void Screen4View::tearDownScreen()
{
    Screen4ViewBase::tearDownScreen();
}
void Screen4View::sliderValueStartedChangeCallbackHandler(const touchgfx::Slider& src, int value)
{
    if (&src == &slider1)
    {
        //execute code whenever the slider starts changing value.
    }
}

void Screen4View::sliderValueChangedCallbackHandler(const touchgfx::Slider& src, int value)
{
    if (&src == &slider1)
    {
        //execute code whenever the value of the slider changes.
    }
}

void Screen4View::sliderValueConfirmedCallbackHandler(const touchgfx::Slider& src, int value)
{

    	pwm_div = slider1.getValue();
        //execute code whenever the slider stops the changing value.
    	PwmSetDC();
        Unicode::snprintf(textArea2Buffer, 10, "%d",pwm_div);
        textArea2.invalidate();


}
