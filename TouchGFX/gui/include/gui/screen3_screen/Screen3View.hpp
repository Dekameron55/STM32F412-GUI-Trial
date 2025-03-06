#ifndef SCREEN3VIEW_HPP
#define SCREEN3VIEW_HPP

#include <gui_generated/screen3_screen/Screen3ViewBase.hpp>
#include <gui/screen3_screen/Screen3Presenter.hpp>

class Screen3View : public Screen3ViewBase
{
public:
    Screen3View();
    virtual ~Screen3View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
    int tickCounter;
    void handleTickEvent();
};

class GraphLabelsX : public GraphLabelsBase
{
public:

	/** EXTENSION:
	 * Variable for X -label offset , when using extended time- type X labels
	 *
	 */
	int labelOffset=0;

    virtual void drawString(const Rect& invalidatedArea, const Font* fontToDraw, const AbstractDataGraph* graph, const int valueScaled, const int labelScaled, const uint8_t a) const;

	/** EXTENSION:
	 * Function for set X -label offset
	 *
	 *   offset The X label offset of first the label value
	 */
    virtual void setXStartOffset(int offset)
    {
    	labelOffset=offset;
    }

    virtual void invalidateGraphPointAt(int16_t index);
   };
#endif // SCREEN3VIEW_HPP
