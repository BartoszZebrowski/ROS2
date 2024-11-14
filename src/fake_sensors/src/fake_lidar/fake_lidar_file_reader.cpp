#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

class FakeLidarFileReader 
{
    public:
        FakeLidarFileReader(const std::string& file_name, rclcpp::Node* node);
    
    private:
        const char SEPARATOR = ';';

        rclcpp::Node* _node;
        std::ifstream _file;
        std::map<int, std::vector<float>> _lines;
        int _numbersOfColumns;
        int _line = 0;

    public:
        std::vector<float> get_next_fake_reading();
        void reset_fake_reading();

    private:
        void _read_file( std::ifstream& file, const std::string& file_name);
        std::vector<float> _split(const std::string& text, char separator);
};

FakeLidarFileReader::FakeLidarFileReader(const std::string& file_name, rclcpp::Node* node)
{
    _node = node;
    _file = std::ifstream(file_name);

    _read_file(_file, file_name);
}

std::vector<float> FakeLidarFileReader::get_next_fake_reading()
{
    //return _lines[1]; - jesli chcemy zwarac ciagle jedna linie, dobre do obserwacji aplikacji szumu gausa;
    return _lines[_line++];
}

void FakeLidarFileReader::reset_fake_reading()
{
    _line = 0;
}

void FakeLidarFileReader::_read_file(std::ifstream& file, const std::string& file_name)
{
    if (!file.is_open())
        RCLCPP_ERROR(_node->get_logger(), "Can't open this: %s", file_name.c_str());

    std::string line;
    int lineNumber = 0;    

    while (std::getline(file, line)) 
    {
        if(lineNumber != 0)
            _lines[lineNumber] = _split(line, SEPARATOR);
        else 
            _numbersOfColumns = _split(line, SEPARATOR).size();

        lineNumber++;
    }
}



std::vector<float> FakeLidarFileReader::_split(const std::string& text, char separator) 
{
    std::vector<float> tokens;
    std::string token;

    for (char ch : text) {
        if (ch == separator) {
            tokens.push_back(std::stof(token));
            token.clear();
        } 
        else 
        {
            token += ch;
        }
    }

    if (!token.empty())
        tokens.push_back(std::stof(token));

    return tokens;
}





