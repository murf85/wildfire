//Modified version of work started by Soulier and Tratnik - https://github.com/jsoulier/wildfire_simulator 

#pragma once

#include <iostream>
#include <string>
#include <behave/surface.h>
#include <behave/fuelModels.h>
#include <nlohmann/json.hpp>

struct State
{
    int fuelModelNumber;
    double windDirection;
    double windSpeed;
    double elevation;
    int x;
    int y;
    double ignitionTime;
    bool willIgnite;
    bool ignited;
    double sigma;
    int spreadDir;
    double rateOfSpread;

    //set the initial conditions for state variables
    State()
        : fuelModelNumber(0)        //fuelModelNumber
        , windDirection(0.0)        //direction wind is coming from
        , windSpeed(0.0)            //wind speed
        , elevation(0.0)            //elevation of the cell
        , x(0)                      //x value of the cell (grid coordinate in m from defined point on map)
        , y(0)                      //y value of the cell (grid coordinate in m from defined point on map)
        , ignitionTime(INFINITY)    //time that the cell is going to ignite
        , willIgnite(false)         //whether the fire will ignite                 
        , ignited(false)            //whether the cell has ignited yet
        , sigma(0.0)                //time advance
        , spreadDir(0)              //direction that the fire is coming from
        , rateOfSpread(0.0){}       //rate of spread of fire from that direction
        
};

//Check whether the cell has changed
inline bool operator!=(const State& x, const State& y)
{
    return x.ignited != y.ignited || x.willIgnite != y.willIgnite || x.ignitionTime != y.ignitionTime || x.sigma != y.sigma;
}

//Set the information for the logging stream
inline std::ostream& operator<<(std::ostream& os, const State& x)
{
    return os << x.x << ":" << x.y << ":" << x.ignited << ":" << x.willIgnite << ":" << x.ignitionTime  << ":" << x.sigma << ":" << x.spreadDir << ":" << x.rateOfSpread;
}

//pull the data from the JSON file
inline void from_json(const nlohmann::json& j, State& s)
{
    s.ignited = j.at("ignited");
    s.x = j.at("x");
    s.y = j.at("y");
    s.elevation = j.at("elevation");
    s.fuelModelNumber = j.at("fuelModelNumber");
    s.windDirection = j.at("windDirection");
    s.windSpeed = j.at("windSpeed");
}