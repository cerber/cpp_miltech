#include <iostream>
#include <cmath>
#include <cassert>
#include <chrono>

void benchmark_pow(double base, int exponent);

int main() {
    // Benchmarking the pow function for double type vs double multiplication

    // Measure time taken by pow function
    auto start = std::chrono::high_resolution_clock::now();
    double result_pow = std::pow(2.0, 2);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_pow = end - start;
    // Measure time taken by double multiplication
    start = std::chrono::high_resolution_clock::now();
    double result_mul = 2.0 * 2.0; // 2^2
    end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_mul = end - start;
    // Output the results
    std::cout << "Result of pow: " << result_pow << " computed in " 
              << duration_pow.count() << " seconds." << std::endl;
    std::cout << "Result of multiplication: " << result_mul << " computed in " 
              << duration_mul.count() << " seconds." << std::endl;
    // Assert that the results are approximately equal
    assert(std::abs(result_pow - result_mul) < 1e-9);

    // Measure time taken by pow function
    start = std::chrono::high_resolution_clock::now();
    result_pow = std::pow(2.0, 5);
    end = std::chrono::high_resolution_clock::now();
    duration_pow = end - start;
    // Measure time taken by double multiplication
    start = std::chrono::high_resolution_clock::now();
    result_mul = 2.0 * 2.0 * 2.0 * 2.0 * 2.0; // 2^5
    end = std::chrono::high_resolution_clock::now();
    duration_mul = end - start;
    // Output the results
    std::cout << "Result of pow: " << result_pow << " computed in " 
              << duration_pow.count() << " seconds." << std::endl;
    std::cout << "Result of multiplication: " << result_mul << " computed in " 
              << duration_mul.count() << " seconds." << std::endl;
    // Assert that the results are approximately equal
    assert(std::abs(result_pow - result_mul) < 1e-9);

    return 0;   
}
