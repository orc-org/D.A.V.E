#pragma once 

/*
    UserInput
    ----------
    A singleton that will contain information about the current input to the program
    It can be polled anytime another object needs the current input
*/
class UserInput {

private:

    UserInput() {}

public:

    UserInput(const UserInput&) = delete;
    UserInput& operator=(const UserInput&) = delete;

    static UserInput& getInstance()
    {
        static UserInput instance;
        return instance;
    }

    
};