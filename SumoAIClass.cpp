/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/**
 * \file SumoAIClass.h
 *
 * \brief the simulated AI agent. Students should modify this file
 *
 * \author Zongyi Yang, https://www.youtube.com/watch?v=ezp7sibEMmA
 */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include "SumoAIClass.h"
#include <stdlib.h>


void SumoAIClass::sum_reward(double rew)
{
    reward+=rew;
}

void SumoAIClass::setState(int state)
{
    this->state=state;
}


int SumoAIClass::episode_iterator(int init_state, double Q[][NUM_ACTIONS], double R[][NUM_ACTIONS])
{
    double Q_before, Q_after;
    // next action
    int next_action;
    double max_q;

    // start series event loop
    int step=0;
    while (1){
        cout << "-- step " << step << ": initial state: " << init_state << endl;
        // memset possible_action array

        // get next action
        next_action = rand()%NUM_STATES;
        cout << "-- step " << step << ": next action: " << next_action << endl;
        // treat next action as state, and we can get max{Q(s', a')}
        max_q = get_max_q(Q, next_action);

        Q_before = Q[init_state][next_action];
        // update formula Q(s,a)=R(s,a)+alpha * max{Q(s', a')}
        Q[init_state][next_action] = R[init_state][next_action] + ALPHA * max_q;
        Q_after = Q[init_state][next_action];

        // next episode rules
        // if next_action is destination state, then go next episode
        // if not, then go back to this loop
        if (next_action == 15 || next_action == 14){
            init_state = rand()%NUM_STATES;
            break;
        }else{
            // if not destination state, then next action becomes initial state
            init_state = next_action;
        }
        step++;
    }
    return init_state;
}

void SumoAIClass::read_csv(string path)
{
    ifstream in(path);

        string line, field;
        getline(in,line);
        int i =0;
        while ( getline(in,line) )    // get next line in file
        {
            stringstream ss(line);
            int j =0;
            while (getline(ss,field,','))  // break line into comma delimitted fields
            {
                Q[i][j] = stod(field);  // add each field to the Q table
                j++;
            }
            i++;
        }

        return;
}

void SumoAIClass::write_csv(string path)
{
    ofstream File;
    File.open(path);
    File << "0,1,2,3,4,5\n";
    for(int i=0;i<NUM_STATES;i++){
        for(int j=0;j<NUM_ACTIONS;j++){
            File << std::to_string(Q[i][j]) << ",";
        }
        File << endl;
    }
    File.close();
}

int SumoAIClass::max_qtable_value(double Q[][NUM_ACTIONS], int state)
{
    double max;
    int index=0;
    bool flag=false;
    for(int i=0;i<NUM_ACTIONS;i++){
        if(flag==false){
            max= Q[state][i];
            flag=true;
        }
        else{
            if(Q[state][i]>max){
                max = Q[state][i];
                index = i;
            }
        }

    }
    return index;
}

double SumoAIClass::get_max_q(double Q[][NUM_ACTIONS], int state)
{
    bool flag= false;
    double temp_max;
    for (int i = 0; i < NUM_ACTIONS; ++i) {
        if(flag == false){
            temp_max = Q[state][i];
            flag=true;
        }
        else{
            if (Q[state][i] > temp_max){
                temp_max = Q[state][i];
            }
        }

    }
    return temp_max;
}

void SumoAIClass::setRandomParameter(float epsilon)
{
    this->epsilon = epsilon;
}

void SumoAIClass::driverAI1(const unsigned int num_distance_sensors, const unsigned int num_line_sensors,
                 const float * distance_sensor_rad, const float * distance_sensor_angle, const float * distance_sensor_max_sense,
                 const float * line_sensor_rad, const float * line_sensor_angle,
                 const float * distance_sensor_readings, const bool *line_sensor_readings)
{
    // example agent that will push other robots out of the ring

    // things for students to add:
    // -use the back line sensor
    // -the loser of the previous match places robot 2nd, so he can encode using switches
    //  as an initial input the pos/orient of robot and pos/orient of opponent for a bettery start
    // -modify the sensor placement to match the one on the student's robot
    // -students should add a state machine with some hysteresis based on sensor inputs

    // if the line sensor in front of the robot is triggered.
    // this prevents robot from driving out of ring
    if (line_sensor_readings[0] == 1)
    {
        // we will reverse for 20 simulation steps
        revflag = 1;
        revcounter = 20;
    }

    // counting down the reverse here
    if (revflag && revcounter>0)
        revcounter--;
    else
        revflag = 0;

    // do the reverse operation if we are reversing
    if (revflag)
    {
        L = -1; R = -1;
    }
    // if the sensor on the right sees something, go to right by maxing L and throttling R
    else if (distance_sensor_readings[1]!=distance_sensor_max_sense[1])
    {
        L = 1; R = 1;
    }
    // if the sensor on the left sees something, go to left by maxing R and throttling L
    else if (distance_sensor_readings[2]!=distance_sensor_max_sense[2])
    {
        R = 1; L = distance_sensor_readings[2]/distance_sensor_max_sense[2];
    }
    // if the sensor in the center sees something, but left and right do not see anything, go straight
    else if (distance_sensor_readings[0]!=distance_sensor_max_sense[0])
    {
        R = distance_sensor_readings[0]/distance_sensor_max_sense[0]; L = 1;
    }
    else if (distance_sensor_readings[3]!=distance_sensor_max_sense[3])
    {
         L = 0; R = -1;
    }
    // distances sensors see nothing, just go straight
    else{
        L=1;R=-1;
    }
    return;
}

void SumoAIClass::driverAI2(const unsigned int num_distance_sensors, const unsigned int num_line_sensors,
                 const float * distance_sensor_rad, const float * distance_sensor_angle, const float * distance_sensor_max_sense,
                 const float * line_sensor_rad, const float * line_sensor_angle,
                 const float * distance_sensor_readings, const bool *line_sensor_readings)
{
    // example of a stall agent
    L = 0; R = 0;

    return;
}

void SumoAIClass::driverAI3(const unsigned int num_distance_sensors, const unsigned int num_line_sensors,
                 const float * distance_sensor_rad, const float * distance_sensor_angle, const float * distance_sensor_max_sense,
                 const float * line_sensor_rad, const float * line_sensor_angle,
                 const float * distance_sensor_readings, const bool *line_sensor_readings)
{
    //get the actual system state
    if(line_sensor_readings[0]==1){
        if(distance_sensor_readings[1] != distance_sensor_max_sense[1]){
            //state 5
            this->state = 5;
        }
        else if(distance_sensor_readings[0] != distance_sensor_max_sense[0]){
            //state 6
            this->state =6;
        }
        else if(distance_sensor_readings[2] != distance_sensor_max_sense[2]){
            //state 7
            this->state =7;
        }
        else if(distance_sensor_readings[3] != distance_sensor_max_sense[3]){
            //state 8
            this->state =8;
        }
        else {
            //state 4
            this->state=4;
        }
    }
    else if(line_sensor_readings[1]==1){
        if(distance_sensor_readings[1] != distance_sensor_max_sense[1]){
            //state 10
            this->state = 10;
        }
        else if(distance_sensor_readings[0] != distance_sensor_max_sense[0]){
            //state 11
            this->state =11;
        }
        else if(distance_sensor_readings[2] != distance_sensor_max_sense[2]){
            //state 12
            this->state =12;
        }
        else if(distance_sensor_readings[3] != distance_sensor_max_sense[3]){
            //state 13
            this->state =13;
        }
        else {
            //state 9
            this->state=9;
        }
    }
    else{
        if(distance_sensor_readings[1] != distance_sensor_max_sense[1]){
            //state 0
            this->state = 0;
        }
        else if(distance_sensor_readings[0] != distance_sensor_max_sense[0]){
            //state 1
            this->state =1;
        }
        else if(distance_sensor_readings[2] != distance_sensor_max_sense[2]){
            //state 2
            this->state =2;
        }
        else if(distance_sensor_readings[3] != distance_sensor_max_sense[3]){
            //state 3
            this->state =3;
        }
        else {
            //state 16
            this->state=16;
        }
    }
    if(state == prev_state && frameCounter>0){
        frameCounter--;
    }
    else{

        //get maximum value in Q-table in that row
        double max = get_max_q(Q, this->state);



        //generate a pseudo-random number between 0 and 1 and compare with the epsilon value
        float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

        //in case EPSILON is higher than the random number, pick up a random action, otherwise, select that action from the Q-Table
        if(r < epsilon && (state!=15 || state!=14)){
            action = rand()%(NUM_ACTIONS-1);
        }
        else{
            action = max_qtable_value(Q,state);
        }


        if(state!=14 && state!=15){
            //select the action
            switch (action) {
            case 0:
                L=1;R=1;
                break;
            case 1:
                L=-1;R=-1;
                break;
            case 2:
                L=0;R=1;
                break;
            case 3:
                L=1;R=0;
                break;
            case 4:
                L=-1;R=0;
                break;
            case 5:
                L=0;R=-1;
                break;
            }
        }

        //update Q-Table
         Q[prev_state][prev_action] = (1-ALPHA) * Q[prev_state][prev_action]+ ALPHA*(reward+GAMMA*max);


        //make previous action and previous states be the actuals so we can update the variable in the next iteration

        prev_state = this->state;
        prev_action = this->action;

        //use frames to the same action
        frameCounter=5;

        //clear reward
        reward=0;
    }
    return;
}


SumoAIClass::SumoAIClass()
{
    read_csv("/home/kallil/Documentos/Reconhecimento de Padrões/Artigão-Seminário/Q_Table_Updated.csv");
   // run_training(16);
    frameCounter=0;
    reward=0;
    prev_action =0;
    prev_state = 16;
    // initialize your memory variables here
    revcounter = 0;
    revflag = 0;
}
