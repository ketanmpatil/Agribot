#include <iostream>
#include <cstring>

using namespace std;  
int main(){

    string name = "ab cvd dd";
    string name2 = "ab";

    size_t found = name.find(name2);

    cout << found;
    return 0;
}