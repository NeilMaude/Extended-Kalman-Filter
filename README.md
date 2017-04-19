# Extended Kalman Filter Project 
Self-Driving Car Engineer Nanodegree Program
Implementation of an EKF in C++

Neil Maude, April 2017

---

## Dependencies
Originally provided as SDCND starter code to build under CMake.
Project further developed using Visual Studio 2017 (as a CMake project).
* cmake >= 3.5

## Basic Build Instructions

1. Clone this repo.
2. Compile using your preferred build system 
3. Run executable: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Development Notes

Kept the Update() and UpdateEKF() functions separate, to demonstrate the difference
between Lidar and Radar updates.  Could re-factor into a single function for more
elegance.

Note also that duplicate calculations have been removed for efficiency in calcs.

### Changes to Provided Code

1. Changed the type of the timestamp variable in `main.cpp` to `long long` (was `long`)
  - this allows the file reader to read the sample sensor data correctly when using
    Visual Studio/Windows (otherwise see strange behaviour when reading the file, 
    such as identical ground truth data values for every sample data point).

### Types: float vs double

It would be normal to default to using `double` types as these have higher range and 
precision in comparison to `float`.  However, `float` requires less memory and is the
type used in the provided template code.  Therefore this was not changed for this
project and floating point variables were left as `float` type.

## Project Results

### File #1 results
Results when using file `sample-laser-radar-measurement-data-1.txt`:

RMSE values:
0.0651648
0.0605379
0.533212
0.544193

### File #2 results
Results when using file `sample-laser-radar-measurement-data-2.txt`:

RMSE values:
0.185496
0.190302
0.476754
0.804469

(Project approved to spec, 17/4/2017.)
