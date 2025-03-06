/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#ifndef FRONTENDAPPLICATIONBASE_HPP
#define FRONTENDAPPLICATIONBASE_HPP

#include <mvp/MVPApplication.hpp>
#include <gui/model/Model.hpp>

class FrontendHeap;

class FrontendApplicationBase : public touchgfx::MVPApplication
{
public:
    FrontendApplicationBase(Model& m, FrontendHeap& heap);
    virtual ~FrontendApplicationBase() { }

    virtual void changeToStartScreen()
    {
        gotoScreen1ScreenNoTransition();
    }

    // Screen1
    void gotoScreen1ScreenNoTransition();

    void gotoScreen1ScreenCoverTransitionEast();

    // Screen2
    void gotoScreen2ScreenBlockTransition();

    // Screen3
    void gotoScreen3ScreenWipeTransitionEast();

    // Screen4
    void gotoScreen4ScreenWipeTransitionWest();

protected:
    touchgfx::Callback<FrontendApplicationBase> transitionCallback;
    FrontendHeap& frontendHeap;
    Model& model;

    // Screen1
    void gotoScreen1ScreenNoTransitionImpl();

    void gotoScreen1ScreenCoverTransitionEastImpl();

    // Screen2
    void gotoScreen2ScreenBlockTransitionImpl();

    // Screen3
    void gotoScreen3ScreenWipeTransitionEastImpl();

    // Screen4
    void gotoScreen4ScreenWipeTransitionWestImpl();
};

#endif // FRONTENDAPPLICATIONBASE_HPP
