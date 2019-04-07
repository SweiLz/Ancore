#include <unistd.h>
#include "Ancore.h"

//          foreground background
// black        30         40
// red          31         41
// green        32         42
// yellow       33         43
// blue         34         44
// magenta      35         45
// cyan         36         46
// white        37         47

// reset             0  (everything back to normal)
// bold/bright       1  (often a brighter shade of the same colour)
// underline         4
// inverse           7  (swap foreground and background colours)
// bold/bright off  21
// underline off    24
// inverse off      27

Ancore *ancore;

int main(int argc, char *argv[])
{
    string port = "/dev/ttySTM32";
    uint32_t baudrate = 115200;
    ancore = new Ancore(port, baudrate);

    while (true)
    {
        try
        {
            ancore->ser.open();
        }
        catch (serial::IOException &e)
        {
            cout << "\033[31mUnable to open port.\033[0m" << endl;
        }

        if (ancore->ser.isOpen())
        {
            cout << "\033[32mSuccessfully connected to serial port.\033[0m" << endl;
            try
            {
                while (true)
                {
                                }
            }
            catch (const std::exception &e)
            {
                cout << "\033[31m" << e.what() << "\033[0m" << endl;
                cout << "\033[33mAttempting reconnection after error.\033[0m" << endl;
                sleep(1);
            }
        }
        else
        {
            cout << "\033[33mCould not connect to serial device : \033[0m" << port << endl;
            sleep(1);
        }
    }

    return 0;
}
