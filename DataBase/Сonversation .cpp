#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <locale.h>


void Show(const std::vector<int>& vector) {
	std::cout << "---VECTOR INFO---\n";
	std::cout << "Elements: ";
	if (vector.empty())
	{
		std::cout << "None";
	}
	else
	{
		for (size_t i = 0; i < vector.size(); i++)
		{
			std::cout << vector[i] << " ";
		}
	}
	std::cout << std::endl;
	std::cout << "Size: " << vector.size() << std::endl;
	std::cout << "Capacity = " << vector.capacity() << std::endl;
	std::cout << "-------------------\n";
}

int main() {

	setlocale(LC_ALL, "Russian");

	std::string filename = "history.txt";
	std::ofstream fileStream(filename, std::ios::out);

	if (!fileStream.is_open()) {
		std::cerr << "Clound`t open the file. :(\n";
		return 1;
	}
	std::cout << "> ";
	std::string text;
	std::getline(std::cin, text);

	size_t textLength = text.length();
	for (size_t i = 0; i < textLength; i++) {
		fileStream.put(text[i]);
	}


}
