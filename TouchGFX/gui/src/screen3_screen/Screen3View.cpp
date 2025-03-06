#include <gui/screen3_screen/Screen3View.hpp>
#include <main.h>

#define ADC_SAMPLES_NUM 128

extern volatile uint32_t shadowBuffer[ADC_SAMPLES_NUM];;
uint8_t i = 0;

Screen3View::Screen3View()
{
    tickCounter = 0;

}

void Screen3View::setupScreen()
{
    Screen3ViewBase::setupScreen();
}

void Screen3View::tearDownScreen()
{
    Screen3ViewBase::tearDownScreen();
}
void Screen3View::handleTickEvent()
{
    tickCounter++;

    // Insert each second tick
    if (tickCounter % 2 == 0)
    {
      // Insert data point
		if(i == 127){
			i=0;
//			AdcDmaStart();
		}
        Unicode::snprintfFloat(textArea3Buffer, 50, "%.3f",adc_avg);

        textArea3.invalidate();

      dynamicGraph1.addDataPoint((float)(shadowBuffer[i]*0.0008056640625));
      i++;

    }

}
