#include "Y2Matrix/YMatrix.hpp"

int main() {
 
    YMatrix m1 = {{1, 2}, {3, 4}};
    YMatrix m2 = {{5}, {7}};
    
    YMatrix m4 = m1 * m2;
    YMatrix m5 = m4.transpose();
    m4.print();
    m5.print();
    
    return 0;


    return 0;
}