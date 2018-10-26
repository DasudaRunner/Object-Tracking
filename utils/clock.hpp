//
// Created by dasuda on 18-10-26.
//

#ifndef TRACKING_CLOCK_HPP
#define TRACKING_CLOCK_HPP
#include <time.h>
#include <iostream>
#include <stdio.h>

namespace wong {
    using namespace std;

    class clocker {

        private:
            clock_t last_time;

        public:
            void clock_start(){
                last_time = clock();
            }

            void print_time_us(string ID){
                cout<<"Total time["<<ID<<"]: "<<((double)((clock()-last_time)*1000000.0)/CLOCKS_PER_SEC)<<"us"<<endl;
            }
            void print_time_ms(string ID){
                cout<<"Total time["<<ID<<"]: "<<((double)((clock()-last_time)*1000.0)/CLOCKS_PER_SEC)<<"ms"<<endl;
            }

            double getTime_ms(){
                return ((double)((clock()-last_time)*1000.0)/CLOCKS_PER_SEC);
            }
            double getTime_us(){
                return ((double)((clock()-last_time)*1000000.0)/CLOCKS_PER_SEC);
            }
    };
}

#endif //TRACKING_CLOCK_HPP
