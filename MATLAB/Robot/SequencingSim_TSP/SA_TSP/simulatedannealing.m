function s = simulatedannealing(inputcities,initial_temperature,...
    cooling_rate,threshold,numberofcitiestoswap)
% SIMULATEDANNEALING
% S = SIMULATEDANNEALING(inputcities,initial_temperature,cooling_rate)
% returns the new configuration of cities with an optimal solution for the
% traveling salesman problem for n cities. 
%
%The input arguments are
% INPUTCITIES         - The cordinates for n cities are represented as 2
%                       rows and n columns and is passed as an argument for
%                       SIMULATEDANNEALING.
% INITIAL_TEMPERATURE - The initial temperature to start the
%                       simulatedannealing process.
% COOLING_RATE        - Cooling rate for the simulatedannealing process. 
%                       Cooling rate should always be less than one.
% THRESHOLD           - Threshold is the stopping criteria and it is the
%                       acceptable distance for n cities.
% NUMBEROFCITIESTOSWAP- Specify the maximum number of pair of cities to
%                       swap. As temperature decreases the number of cities
%                       to be swapped decreases and eventually reaches one
%                       pair of cities.

% Keep the count for number of iterations.
global iterations;

% Set the current temperature to initial temperature.
temperature = initial_temperature;

% This is specific to TSP problem. In this algorithm as the temperature
% decreases the number of pairs of cities to swap reduces. Which means as
% the temperature cools down the search is carried without by gradient
% descent and search is carried out locally.
initial_cities_to_swap = numberofcitiestoswap;

% Initialize the iteration number.
iterations = 1;

% This is a flag used to cool the current temperature after 10 iterations
% irrespective of wether or not the function is minimized. This is my
% receipie and done based on my experience. This is not part of the
% original algorithm.
complete_temperature_iterations = 0;

% This is my objective function, the total distance for the routes.
previous_distance = distance_htetro(inputcities);

while iterations < threshold
    temp_cities = swapcities(inputcities,numberofcitiestoswap);
    current_distance = distance_htetro(inputcities);
    diff = abs(current_distance - previous_distance);
    if current_distance < previous_distance
        inputcities = temp_cities;
        if rem(iterations,100) == 0
                plotcities(inputcities);
        end
        if complete_temperature_iterations >= 10
            temperature = cooling_rate*temperature;
            complete_temperature_iterations = 0;
        end
        numberofcitiestoswap = round(numberofcitiestoswap...
            *exp(-diff/(iterations*temperature)));
        if numberofcitiestoswap == 0
            numberofcitiestoswap = 1;
        end
        previous_distance = current_distance;
        iterations = iterations + 1;
        complete_temperature_iterations = complete_temperature_iterations + 1;
    else
        if rand(1) < exp(-diff/(temperature))
            inputcities = temp_cities;
            if rem(iterations,100) == 0
                plotcities(inputcities);
            end
            numberofcitiestoswap = round(numberofcitiestoswap...
                *exp(-diff/(iterations*temperature)));
            if numberofcitiestoswap == 0
                numberofcitiestoswap = 1;
            end
            previous_distance = current_distance;
            complete_temperature_iterations = complete_temperature_iterations + 1;
            iterations = iterations + 1;
        end
    end
end