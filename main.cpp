#include "core.hpp"
#include "main.h"

int main()
{
    stdio_init_all();
    setup();
    while (true) {
        try {
        if (Reset.press()) {
            sys.reset();
        }
        loop();
        } catch(exception_handler_t handler)
        {
            usb_serial.println(exception.get_current_exception());
        }
        tight_loop_contents();
    }
    return 0;
}
