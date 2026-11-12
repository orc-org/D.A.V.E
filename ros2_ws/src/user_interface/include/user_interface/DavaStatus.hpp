#pragma once 

/*
    DaveStatus
    ----------
    A singleton that will contain all the information that is being sent/received from our ros2 topics and services
*/
class DaveStatus {

private:

    DaveStatus() {}

public:

    DaveStatus(const DaveStatus&) = delete;
    DaveStatus& operator=(const DaveStatus&) = delete;

    static DaveStatus& getInstance()
    {
        static DaveStatus instance;
        return instance;
    }

    
};