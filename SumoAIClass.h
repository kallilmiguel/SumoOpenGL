/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/**
 * \file SumoAIClass.h
 *
 * \brief the simulated AI agent. Students should modify this file
 *
 * \author Zongyi Yang, https://www.youtube.com/watch?v=ezp7sibEMmA
 */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#pragma once


#define MAX_EPISODE 1000
#define NUM_STATES 17
#define NUM_ACTIONS 6
#define ALPHA 0.1
#define GAMMA 0.4
#define EPSILON 0.1

#ifndef PI
#define PI 3.14159265
#endif

#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

class SumoAIClass
{
private:
    // students should put "memory" variables here, they will be retained between frames
    double Q[NUM_STATES][NUM_ACTIONS];
    float epsilon;
    int frameCounter;
    int prev_state;
    int state;
    int prev_action;
    int action;
    double reward;
    unsigned int revcounter;
    bool revflag;
public:
    // the AI agent must update L and R after the driverAI is executed
    float L, R;

    void sum_reward(double rew);
    void setState(int state);
    void run_training(int initial_state);
    int episode_iterator(int init_state, double Q[NUM_STATES][NUM_ACTIONS], double R[NUM_STATES][NUM_ACTIONS]);
    void read_csv(std::string path);
    void write_csv(std::string path);
    int max_qtable_value(double Q[NUM_STATES][NUM_ACTIONS], int state);

    // todo: right now we hardcode 2 AI slots. In the future, there should be a
    //       virtual AI class that each agent inherets from. Each agent should only
    //       have one AI strategy, and the SumoBotClass will call driveAI from the
    //       virtual parent
   double get_max_q(double Q[NUM_STATES][NUM_ACTIONS], int state);

   void setRandomParameter(float epsilon);
    void driverAI1(const unsigned int num_distance_sensors, const unsigned int num_line_sensors,
                 const float * distance_sensor_rad, const float * distance_sensor_angle, const float * distance_sensor_max_sense,
                 const float * line_sensor_rad, const float * line_sensor_angle,
                 const float * distance_sensor_readings, const bool *line_sensor_readings);
    void driverAI2(const unsigned int num_distance_sensors, const unsigned int num_line_sensors,
                 const float * distance_sensor_rad, const float * distance_sensor_angle, const float * distance_sensor_max_sense,
                 const float * line_sensor_rad, const float * line_sensor_angle,
                 const float * distance_sensor_readings, const bool *line_sensor_readings);
    void driverAI3(const unsigned int num_distance_sensors, const unsigned int num_line_sensors,
                 const float * distance_sensor_rad, const float * distance_sensor_angle, const float * distance_sensor_max_sense,
                 const float * line_sensor_rad, const float * line_sensor_angle,
                 const float * distance_sensor_readings, const bool *line_sensor_readings);
    SumoAIClass();
};
