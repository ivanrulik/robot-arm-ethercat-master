#include "csvfile.h"

int main()
{
    try
    {
        csvfile csv("testLog.csv",false); // throws exceptions!
        // Hearer
        csv <<"Time"<< "J1_pos" << "J1_vel" << "J1_tor" <<
               "J2_pos" << "J2_vel" << "J2_tor" <<
               "J3_pos" << "J3_vel" << "J3_tor" <<
               "J4_pos" << "J4_vel" << "J4_tor" <<
               "J5_pos" << "J5_vel" << "J5_tor" <<
               "J6_pos" << "J6_vel" << "J6_tor" <<  endrow;
        // Data
        int i = 1;
        csv << i++ << "String value" << endrow;
        csv << i++ << 123 << endrow;
        csv << i++ << 1.f << endrow;
        csv << i++ << 1.2 << endrow;
        csv << i++ << "One more string" << endrow;
        csv << i++ << "\"Escaped\"" << endrow;
        csv << i++ << "=HYPERLINK(\"https://playkey.net\"; \"Playkey Service\")" << endrow;
    }
    catch (const std::exception &ex)
    {
        std::cout << "Exception was thrown: " << ex.what() << std::endl;
    }
    return 0;
}