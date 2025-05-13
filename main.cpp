#include "viewer/SimViewer.h"
#include "util/Types.h"

#include <cstdlib>
#include <ctime>


int main(int argc, char* argv[])
{
    std::srand(std::time(nullptr));

    SimViewer app;
    app.start();

    return 0;
}
