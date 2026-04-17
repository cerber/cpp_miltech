#include <iostream>
#include <cstring>

using namespace std;

int main() {
    static int arr[5] = {1, 2, 3, 4, 5};
    for (size_t i = 0; i < 5; ++i) {
        cout << arr[i] << " ";
    }
    cout << endl;

    arr[2] = 10; // Modifying the array element at index 2
    for (size_t i = 0; i < 5; ++i) {
        cout << arr[i] << " ";
    }
    cout << endl;
    return 0;
}