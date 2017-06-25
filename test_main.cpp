// Comment / Uncomment lines in CMakeLists.txt that refer to *.cpp with main() functions
// then Uncomment next line
#define CATCH_CONFIG_MAIN // This tells Catch to provide a main() - only do this in one cpp file.
//                          Once you have more than one file with unit tests in you'll just #include "catch.hpp" and go.
//                          https://github.com/philsquared/Catch/blob/master/docs/tutorial.md
#include "catch.hpp"

//#include <math.h>
#include <iostream>
//#include "Eigen/Dense"
#include <string>

#include "main.cpp"  // NOTE : block comment-out main() function in main.cpp 1st

/**
 * Test hello world text.
 */

SCENARIO("Program says 'Hello, World!'",
         "[main_app]") {

    GIVEN("main.cpp exists") {
        REQUIRE(true);

        WHEN("main function is called") {
            REQUIRE(true);

            THEN("the correct welcome message is displayed") {
                std::cout << showMessage() << std::endl;
                REQUIRE((showMessage()) == "Hello, World!");
            }
        }
    }
}
