//Modified version of work started by Soulier and Tratnik - https://github.com/jsoulier/wildfire_simulator 

#pragma once

#include <cmath>
#include <cfloat>
#include <nlohmann/json.hpp>
#include <cadmium/celldevs/asymm/cell.hpp>
#include <cadmium/celldevs/asymm/config.hpp>
#include <behave/surface.h>
#include <behave/fuelModels.h>
#include "state.hpp"

class Cell : public cadmium::celldevs::AsymmCell<State, double>
{
public:
    Cell(const std::string& id, const std::shared_ptr<const cadmium::celldevs::AsymmCellConfig<State, double>>& config)
        : cadmium::celldevs::AsymmCell<State, double>(id, config) {}

    [[nodiscard]] State localComputation(State state, const std::unordered_map<
        std::string, cadmium::celldevs::NeighborData<State, double>>& neighborhood) const override
    {
        //If the cell has already ignited, it has already told its neightbours and it can passivate
        if (state.ignited)
        {
            state.sigma = std::numeric_limits<double>::infinity();
            return state;
        }
        //When it reaches the ignition time it needs to change to ignited and tell its neighbours immediately
        if (state.ignitionTime <= this->clock){
            state.ignited = true;
            state.sigma = 0.0;
            return state;
        }
        //if it's not yet ignited it should check whether whatever woke the cell will cause it to ignite, or ignite faster than previously calculated
        for (const auto& [neighborId, neighborData]: neighborhood)
        {
            if (!neighborData.state->ignited)   //check if the neighbour is ignited, if not go to the next neighbour
            {
                continue;
            }
            //calculate neighbour distance, direction and elevation change relative to current cell
            double deltaX = state.x - neighborData.state->x;
            double deltaY = state.y - neighborData.state->y;
            double direction = atan2(deltaX, deltaY) * 180.0 / M_PI;
            double distance_btwn = sqrt(pow(deltaX,2) + pow(deltaY,2));
            double elevation_change = state.elevation - neighborData.state->elevation;
            double slope;
            double aspect;
            
            // set aspect for which direction between the two center points is uphill
            if (elevation_change < 0){
                slope = -100*elevation_change/distance_btwn;
                aspect = direction + 180;
            }
            else {
                slope = 100*elevation_change/distance_btwn;
                aspect = direction;
            }

            //Set parameters for the BehavePlus model
            FuelModels fuelModels;
            Surface surface(fuelModels);
            surface.updateSurfaceInputsForTwoFuelModels(
                neighborData.state->fuelModelNumber,
                state.fuelModelNumber,
                5.0, // moistureOneHour
                6.0, // moistureTenHour
                7.0, // moistureHundredHour
                30.0, // moistureLiveHerbaceous
                30.0, // moistureLiveWoody,
                FractionUnits::Percent, // moistureUnits
                neighborData.state->windSpeed,
                SpeedUnits::KilometersPerHour, // windSpeedUnits
                WindHeightInputMode::DirectMidflame, // windHeightInputMode
                neighborData.state->windDirection,
                WindAndSpreadOrientationMode::RelativeToNorth, // windAndSpreadOrientationMode
                50.0, // firstFuelModelCoverage,
                FractionUnits::Percent, // firstFuelModelCoverageUnits
                TwoFuelModelsMethod::Arithmetic, // twoFuelModelMethod
                slope,
                SlopeUnits::Percent, // slopeUnits
                aspect,
                30.0, // canopyCover
                FractionUnits::Percent, // canopyCoverUnits
                10.0, // canopyHeight,
                LengthUnits::Meters, // canopyHeightUnits,
                40.0, // crownRatio,
                FractionUnits::Percent // crownRatioUnits
            );
            //Calculate the spread rate
            surface.doSurfaceRunInDirectionOfInterest(direction, SurfaceFireSpreadDirectionMode::FromIgnitionPoint);
            const double spreadRate = surface.getSpreadRateInDirectionOfInterest(SpeedUnits::MetersPerSecond);
            
            //If the spread rate is not high enough, ignore it
            if (spreadRate < DBL_EPSILON || spreadRate != spreadRate)
            {
                continue;
            }
            
            //Calculate how long it will take to spread to this cell
            const double timeToWait = distance_btwn / spreadRate;

            //If the fire is going to spread from another cell faster, then keep that ignition time, recalculate the TA
            if(state.ignitionTime < this->clock + timeToWait){
                state.sigma = std::max(state.ignitionTime - this->clock, 0.0);
                continue;
            }
            //calculate new ignition time, set the spread rate and direction
            state.ignitionTime =  this->clock + timeToWait;
            state.rateOfSpread = spreadRate;
            
            //make the direction more readable for the log
            if (direction<0){
                direction = 360+direction;
            }
            state.spreadDir = (int)direction;
            
            //set the cell to waiting to ignite mode
            state.willIgnite = true;
            //passivate until ignition time
            state.sigma = std::max(state.ignitionTime - this->clock, 0.0);
        }
        return state;
    }
    //return sigma as output delay
    [[nodiscard]] double outputDelay(const State& state) const override
    {
        return state.sigma;
    }
};
