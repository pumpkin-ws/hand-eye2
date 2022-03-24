#include <iostream>
#include <cstdlib>
#include <unordered_set>
#include <set>
#include <chrono>

int SET_SIZE = 10'000'000;

int main(int argc, char** argv) {
    // char* num = 0;
    
    // double n = atof(num);
    // std::cout << n << std::endl;
    
    std::unordered_set<int> unordered_set; // unordered set
    std::set<int> regular_set; // regular set
    
    // comparison of the time for inserting 10 million elements into an unorder set and an ordered set
    auto start = std::chrono::steady_clock::now();
    printf("Starting to insert elements...\n");
    for(int i = 0; i < SET_SIZE; i++) {
        unordered_set.insert(i);
    }
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elps = end - start;
    std::cout << "The time spent on inserting into unordered set is " << elps.count() << " ms." << std::endl;
    std::cout << "The count of unordered set is " << unordered_set.count(234) << std::endl;

    printf("\n\n---------------------\n\n");
    start = std::chrono::steady_clock::now();
    printf("Starting to insert elements...\n");
    for(int i = 0; i < SET_SIZE; i++) {
        regular_set.insert(i);
    }

    end = std::chrono::steady_clock::now();
    elps = end - start;
    std::cout << "The time spent on inserting into set is " << elps.count() << " ms." << std::endl;
    std::cout << "The count of unordered set is " << unordered_set.count(234) << std::endl;

    return EXIT_SUCCESS;
}