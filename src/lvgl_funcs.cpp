#include "include.hpp"
#include <string>
#include "lvgl_funcs.hpp"

#define BRAIN_SCREEN_WIDTH 478.f
#define BRAIN_SCREEN_HEIGHT 240.f

#define NUMBER_OF_MOTORS 8

#define BAR_HEIGHT 200.f
#define BAR_OFFSET 7.f
static constexpr int BAR_WIDTH = ((BRAIN_SCREEN_WIDTH) - (BAR_OFFSET * (NUMBER_OF_MOTORS + 1)))/NUMBER_OF_MOTORS;

#define MIN_BAR_TEMP 0.f
#define MAX_BAR_TEMP 100.f

#define ANIM_TIME_MS 200.f

static constexpr float TEMPERATURE_LABEL_CONSTANT = BAR_HEIGHT/(MAX_BAR_TEMP - MIN_BAR_TEMP);

#define WARNING_THRESH 45.f
#define DANGER_THRESH 55.f

#define INITIAL_TEMPERATURE 35.f

#define TEMPERATURE_LABEL_BUFFER_SIZE 8

struct lvTemperatureBundle
{
    lv_obj_t *bar;
    lv_obj_t *nameLabel;
    lv_obj_t *temperatureLabel;
    std::string motorName;
    pros::Motor *motor;
};

lv_obj_t* parent = lv_obj_create(lv_scr_act());

const lvTemperatureBundle temperatureBundles[NUMBER_OF_MOTORS] = {
    {lv_bar_create(parent), lv_label_create(parent), lv_label_create(parent), std::string("FL"), &frontleft},
    {lv_bar_create(parent), lv_label_create(parent), lv_label_create(parent), std::string("ML"), &midleft},
    {lv_bar_create(parent), lv_label_create(parent), lv_label_create(parent), std::string("BL"), &backleft},
    {lv_bar_create(parent), lv_label_create(parent), lv_label_create(parent), std::string("FR"), &frontright},
    {lv_bar_create(parent), lv_label_create(parent), lv_label_create(parent), std::string("MR"), &midright},
    {lv_bar_create(parent), lv_label_create(parent), lv_label_create(parent), std::string("BR"), &backright},
    {lv_bar_create(parent), lv_label_create(parent), lv_label_create(parent), std::string("I"),  &intake},
    {lv_bar_create(parent), lv_label_create(parent), lv_label_create(parent), std::string("H"),  &hang}
};

lv_style_t parentStyle;
lv_style_t barStyle;
lv_style_t barNormalStyle;
lv_style_t barWarningStyle;
lv_style_t barDangerStyle;
lv_style_t textStyle;


/* Initialize a bar object 

void setBarAttributes(lv_obj_t* barObj, lv_obj_t* allignTo, int offSet)
{
    lv_bar_set_range(barObj, MIN_BAR_TEMP, MAX_BAR_TEMP);
    lv_bar_set_style(barObj, LV_BAR_STYLE_BG, &barStyle);
    lv_bar_set_style(barObj, LV_BAR_STYLE_INDIC, &barNormalStyle);
    lv_obj_set_size(barObj, BAR_WIDTH, BAR_HEIGHT);
    lv_obj_align(barObj, allignTo, LV_ALIGN_IN_LEFT_MID, offSet, 0);
}

*/


/* Initialize the entire bar graph */

/*
void initBarGraph()
{
    //INITIALIZE STYLES
    lv_style_copy(&parentStyle, &lv_style_plain);
    parentStyle.body.opa = 0;
    parentStyle.body.radius = 0;

    lv_style_copy(&barStyle, &lv_style_transp);

    lv_style_copy(&barNormalStyle, &lv_style_plain);
    barNormalStyle.body.main_color = LV_COLOR_MAKE(0, 255, 0);
    barNormalStyle.body.grad_color = LV_COLOR_MAKE(0, 255, 0);
    barNormalStyle.body.radius = 0;
    barNormalStyle.body.border.opa = LV_OPA_0;
    barNormalStyle.body.padding.hor = 0;            
    barNormalStyle.body.padding.ver = 0;

    lv_style_copy(&barWarningStyle, &barNormalStyle);
    barWarningStyle.body.main_color = LV_COLOR_MAKE(255, 255, 0);
    barWarningStyle.body.grad_color = LV_COLOR_MAKE(255, 255, 0);

    lv_style_copy(&barDangerStyle, &barNormalStyle);
    barDangerStyle.body.main_color = LV_COLOR_MAKE(255, 0, 0);
    barDangerStyle.body.grad_color = LV_COLOR_MAKE(255, 0, 0);

    lv_style_copy(&textStyle, &lv_style_plain);
    textStyle.text.color = LV_COLOR_WHITE;

    lv_obj_set_style(parent, &parentStyle);
    lv_obj_set_pos(parent, 0, 0);
    lv_obj_set_size(parent, BRAIN_SCREEN_WIDTH, BRAIN_SCREEN_HEIGHT);

*/

    /* Initialize all of the bar objects */

/*
    setBarAttributes(temperatureBundles[0].bar, NULL, 10);
    for(int i = 1; i < NUMBER_OF_MOTORS; i++)
    {
        setBarAttributes(temperatureBundles[i].bar, temperatureBundles[i-1].bar, (BAR_WIDTH + BAR_OFFSET));
    }

    for(int i = 0; i < NUMBER_OF_MOTORS; i++)
    {
*/
        
        /* Set label attributes */

        /*
        lv_label_set_text(temperatureBundles[i].nameLabel, temperatureBundles[i].motorName.c_str());
        lv_label_set_style(temperatureBundles[i].nameLabel, &textStyle);
        lv_obj_align(temperatureBundles[i].nameLabel, temperatureBundles[i].bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
        lv_label_set_style(temperatureBundles[i].temperatureLabel, &textStyle);
        */

        /* Set temperature values to INITIAL_TEMPERATURE by default */

        /*
        setBarValAndColor(temperatureBundles[i].bar, temperatureBundles[i].temperatureLabel, INITIAL_TEMPERATURE);
    }
}

*/

/* Set the height, color, and value text of a single unit */

/*
void setBarValAndColor(lv_obj_t *bar, lv_obj_t *temperatureLabel, int temperature)
{
    lv_bar_set_value_anim(bar, temperature, ANIM_TIME_MS);

    char buffer[TEMPERATURE_LABEL_BUFFER_SIZE];
    if(temperature != PROS_ERR)
    {
        if(temperature >= DANGER_THRESH) lv_bar_set_style(bar, LV_BAR_STYLE_INDIC, &barDangerStyle);
        else if(temperature >= WARNING_THRESH) lv_bar_set_style(bar, LV_BAR_STYLE_INDIC, &barWarningStyle);
        else lv_bar_set_style(bar, LV_BAR_STYLE_INDIC, &barNormalStyle);

        sprintf(buffer, "%i", temperature);
    }
    else
    {
        lv_bar_set_style(bar, LV_BAR_STYLE_INDIC, &barDangerStyle);
        sprintf(buffer, "ERR");
    }

    lv_label_set_text(temperatureLabel, buffer);
    lv_obj_align(temperatureLabel, bar, LV_ALIGN_IN_BOTTOM_MID, 0, -(temperature*TEMPERATURE_LABEL_CONSTANT));
}

*/

/* Update the entire bar graph */


void updateBarGraph()
{
    for(int i = 0; i < NUMBER_OF_MOTORS; i++)
    {
        int temp = (*temperatureBundles[i].motor).get_temperature();
        setBarValAndColor(temperatureBundles[i].bar, temperatureBundles[i].temperatureLabel, temp);
    }
}

/* Function to be passed to a task, updates the entire bar graph */

void updateBarGraph_fn(void* param)
{
    uint32_t startTime = pros::millis();
    pros::Task::delay_until(&startTime, 1000);
    while(true)
    {
        updateBarGraph();
        pros::Task::delay_until(&startTime, 1000);  
    }
}