#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

class FakeLidarFileReader 
{
    public:
        FakeLidarFileReader(const std::string& file_name, rclcpp::Node* node);

    public:
    
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
        void read_file( std::ifstream& file, const std::string& file_name);
        std::vector<float> split(const std::string& text, char separator);
};

FakeLidarFileReader::FakeLidarFileReader(const std::string& file_name, rclcpp::Node* node)
{
    _node = node;
    _file = std::ifstream(file_name);

    read_file(_file, file_name);
}

std::vector<float> FakeLidarFileReader::get_next_fake_reading()
{
    //RCLCPP_INFO(_node->get_logger(), "%i", (int)_lines.size());


    return _lines[2];
}

void FakeLidarFileReader::reset_fake_reading()
{
    _line = 0;
}

void FakeLidarFileReader::read_file(std::ifstream& file, const std::string& file_name)
{
    if (!file.is_open())
        RCLCPP_ERROR(_node->get_logger(), "Can't open this dupa: %s", file_name.c_str());
        //printf("dupa");

    std::string line;
    int lineNumber = 0;    

    while (std::getline(file, line)) 
    {
        if(lineNumber != 0)
            _lines[lineNumber] = split(line, SEPARATOR);
        else 
            _numbersOfColumns = split(line, SEPARATOR).size();

        lineNumber++;
    }
}



std::vector<float> FakeLidarFileReader::split(const std::string& text, char separator) 
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





