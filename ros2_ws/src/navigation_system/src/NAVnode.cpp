#include "NAVnode.hpp"
#include "nav_nmea.hpp"
#include "nav_detour.hpp"
#include <algorithm>

using std::placeholders::_1;
using std::placeholders::_2;
using nav_nmea::nmeaToDegrees;
using nav_nmea::nmeaFields;

class DummyGPS{
public:
    DummyGPS(){
        std::cout << "WARNING: DUMMY GPS IN USE \nDISABLE THIS IF YOU ARE NOT TESTING";
    }
    void begin() {

    }
    string read(){
        return "thing";
    }
    void write(string thing){

    }
};

//DummyGPS* GNSS = new DummyGPS();

void NAVnode::checkWaypointCollisions(){
    ListNode* current = routes[routeToEdit].getHead();
    while (current != nullptr) {
    ListNode* next = current->next;

    for (int i = 0; i < obstacleCount; i++) {
        double dx = current->point->getX() - obstacles[i]->getX();
        double dy = current->point->getY() - obstacles[i]->getY();

        if (std::sqrt(dx * dx + dy * dy) < obstacles[i]->getRadius()) {
            waypointCollisionDetected(current, obstacles[i]);
            break;
        }
    }

        current = next;
    }
}

int NAVnode::findWaypointCollision(Waypoint* collidingWaypoint){
    for (int i = 0; i < obstacleCount; i++) {
            double dx = collidingWaypoint->getX() - obstacles[i]->getX();
            double dy = collidingWaypoint->getY() - obstacles[i]->getY();
            if (sqrt(dx*dx + dy*dy) < obstacles[i]->getRadius()) {
                return i;
            }
        }
    return -1;
}

bool NAVnode::isWaypointColliding(ListNode* node){
    return isWaypointColliding(node->point);
}

bool NAVnode::isWaypointColliding(Waypoint* point){
    for (int i = 0; i < obstacleCount; i++) {
        double dx = point->getX() - obstacles[i]->getX();
        double dy = point->getY() - obstacles[i]->getY();
        if (sqrt(dx*dx + dy*dy) < obstacles[i]->getRadius()) {
            return true;
        }
    }
    return false;
}

void NAVnode::checkTrackCollisions(){
    ListNode* current = routes[routeToEdit].getHead();

    if(current == nullptr)
        return;

    current = current->next;
    while (current != nullptr) {
        for (int i = 0; i < obstacleCount; i++) {
            auto pointOfInterest = calculatePointOfInterest(
                current->previous,
                current,
                obstacles[i]
            );
            double dx = obstacles[i]->getX() - pointOfInterest[0]; 
            double dy = obstacles[i]->getY() - pointOfInterest[1];
            double distanceToCenter = sqrt(dx * dx + dy * dy);
            if (distanceToCenter < obstacles[i]->getRadius()){
                trackCollisionDetected(current, obstacles[i]);
            }
        }
        current = current->next;
    }
}

void NAVnode::checkTrackCollisions(ListNode* start, ListNode* end, Obstacle* obstacle){
        
    auto pointOfInterest = calculatePointOfInterest(start, end, obstacle);
    double dx = obstacle->getX() - pointOfInterest[0]; 
    double dy = obstacle->getY() - pointOfInterest[1];
    double distanceToCenter = sqrt(dx * dx + dy * dy);
    if (distanceToCenter < obstacle->getRadius()){
        generateDetour(pointOfInterest[0], pointOfInterest[1], end, obstacle);
    }
}
    
    /*
        Description of Circumnavigation Styles:
        ------------------------------------------------
        Reroute: If a collision is detected between our route and an Obstacle, we will create a new waypoint that is 1m outside of the edge of the obstacle to ensure we avoid it,
                 the algorithm will be recursively called to ensure that any changes made to the route do not create new collisions.

                 any waypoints found to be within the radius of the obstacle will be removed when using this circumnavigation style

                 the inspiration for this approach came from the opening and closing angle method of drift correction used in civil aviation.
                 when navigating in the air, pilots would estimate how far they are off track and then use that angle and the distance to the next waypoint to calculate a new heading
                 that will take them from their current position to the next waypoint while also compensating for the wind that pushed them off course.

                 Obviously, we do not need to worry about wind correction, but the idea is still the same.
                 the goal here is to arrive at the destination and not necessarily to follow a specific route.

        Track Crawling: If a collision is detected between our route and an Obstacle, we will add new waypoints along our track that stop short of the obstacles keep out radius,
                        from there, the Rerouting method will be recursively called to chart a path that roughly follows the outline of the obstacle until we intercept our desired track

                        any Waypoints found within the radius of the obstacle will still be removed, but not before new waypoints are added that will preserve the original track
                        as much as possible.

                        the Inspiration for this approach once again came from aviation. The double angle approach is a method of drift correction that is used when the drift is
                        noticed early in the flight. The pilot will estimate the angle between the current track and desired track to determine what angle they need to add to their
                        current heading to compensate for the wind. The pilot will add double this angle to their current heading until they intercept their original route again,
                        at which point they will fly at their calculated heading to fly directly to the next waypoint while still compensating for the wind.

                        that approach involves some math, so sometimes people will cheat and just eyeball their way back to the desired track. of course, since they don't know the
                        heading to fly to compenste for the wind, they will get pushed off track again and will need to continuously correct their heading. 
                        This is known as "Track Crawling" which is generally fround upon since you don't fly directly to your destination and you don't 100% regain your track.
                        However, it is exactly the kind of approximate perfection we need.

                        since we will be trying to stick to our route despite constantly getting forced off course by obstacles, the moniker seemed appropriate.

                        this approach will follow the original path to the best of the rover's ability and will always make redirections that will stray as little as possible.
        
        Automatic Circumnavigation Off: If a collision is detected between our route and an Obstacle, we will simply print a message to the terminal to alert the operator 
                                        of the collision and leave it up to them to decide how to proceed.
    
    */

void NAVnode::waypointCollisionDetected(ListNode* node, Obstacle* obstacle){
    // protect against nodes at the ends of the list which may have null previous/next
    if (!node || !node->point) return;
    if (node->previous == nullptr || node->next == nullptr) {
        // fallback safe behavior: remove the node for reroute/trackCrawling, or log for automatic off
        if (circumnavigationStyle == reroute || circumnavigationStyle == trackCrawling) {
            routes[routeToEdit].remove(node);
            return;
        } else {
            cout << "collision Detected at waypoint (" << node->point->getX() << ", " << node->point->getY() << ")" << endl;
            return;
        }
    }

    switch (circumnavigationStyle){
        case(reroute): routes[routeToEdit].remove(node); break;
        case(trackCrawling): {
            if (node == nullptr || node->point == nullptr) {
                return;
            }

            if (node->previous == nullptr || node->next == nullptr) {
                routes[routeToEdit].remove(node);
                return;
            }

            auto pointOfInterest = calculatePointOfInterest(node->previous, node, obstacle);
                
            /* create a new node to store another waypoint
               to get this new waypoint, we will move back 1 Radii + 1m from the Point if Interest (POI) towards the previous waypoint
               if the POI is the centre of the waypoint, this will make a new waypoint 1m from the edge of the obstacle
               if not, the new waypoint will be further from the edge of the obstacle
               this was done for simplicity, but it really doesn't matter because the further the POI is from the centre, the less drastic the course correction anyway
               so we'll still be following the original route reasonably closely
            */ 
            ListNode* newWaypoint = new ListNode(nullptr, nullptr, 
                new Waypoint(node->point->getX() - (obstacle->getRadius() + 1) * pointOfInterest[2], node->point->getY() - (obstacle->getRadius() + 1) * pointOfInterest[3]));
            
            //check if our new waypoint is colliding with an obstacle
            if (isWaypointColliding(newWaypoint)){
                // we'll try the same thing again, taking care not to go past the previous waypoint

                // check if the previous waypoint is within 1 radii of the waypoint we just made
                double dx = node->previous->point->getX() - newWaypoint->point->getX();
                double dy = node->previous->point->getY() - newWaypoint->point->getY();
                
                delete newWaypoint;
                
                if (!(sqrt(dx * dx + dy * dy) < obstacle->getRadius())){
                    // if moving back one more radii will NOT make a new waypoint behind the first one, then we add a new waypoint 1 radii back from the last one, else we simply omit it
                    newWaypoint = new ListNode(nullptr, nullptr, 
                        new Waypoint(node->point->getX() - (obstacle->getRadius() * 2 + 1) * pointOfInterest[2], node->point->getY() - (obstacle->getRadius() * 2+ 1) * pointOfInterest[3]));
                    
                    routes[routeToEdit].addBefore(newWaypoint, node);
                }
            }
            else{
                routes[routeToEdit].addBefore(newWaypoint, node);
            }

            /*  for the next waypoint, we will need to calculate the unit vector to the next waypoint as there's a good change the the path turns to some degree at the waypoint
                therefore, the direction described by the unit vector calculated with the POI will be incorrect
            
                once we have that, the process is identical to above
            */
            double dx = node->next->point->getX() - node->point->getX();
            double dy = node->next->point->getY() - node->point->getY();

            double distanceToNextWaypoint = std::sqrt(dx * dx + dy * dy);
            
            if (distanceToNextWaypoint < 1e-9){
                routes[routeToEdit].remove(node);
                return;
            }

            double* unitVectorToNextWaypoint = new double[2]{
                dx / distanceToNextWaypoint,
                dy / distanceToNextWaypoint
            };
                
            newWaypoint = new ListNode(nullptr, nullptr, 
                new Waypoint(node->point->getX() + (obstacle->getRadius() + 1) * unitVectorToNextWaypoint[0], node->point->getY() + (obstacle->getRadius() + 1) * unitVectorToNextWaypoint[1]));
            
            // check if our new waypoint is colliding with an obstacle
            if (isWaypointColliding(newWaypoint)){
                // we'll try the same thing again, taking care not to go past the next waypoint

                // check if the next waypoint is within 1 radii of the waypoint we just made
                double dx = node->next->point->getX() - newWaypoint->point->getX();
                double dy = node->next->point->getY() - newWaypoint->point->getY();
                delete newWaypoint;
                
                if (!(sqrt(dx * dx + dy * dy) < obstacle->getRadius())){
                    // if moving back one more radii will NOT make a new waypoint behind the first one, then we add a new waypoint 1 radii back from the last one, else we simply omit it
                    newWaypoint = new ListNode(nullptr, nullptr, 
                        new Waypoint(
                            node->point->getX() + (obstacle->getRadius() * 2 + 1) * unitVectorToNextWaypoint[0],
                            node->point->getY() + (obstacle->getRadius() * 2 + 1) * unitVectorToNextWaypoint[1]
                        ));
                    
                    routes[routeToEdit].addBefore(newWaypoint, node);
                }
            }
            else{
                routes[routeToEdit].addBefore(newWaypoint, node);
            }

            routes[routeToEdit].remove(node);
            delete[] unitVectorToNextWaypoint;
        }; break;
        case(automatic_Circumnavigation_Off): cout << "collision Detected at waypoint (" << node->point->getX() << ", " << node->point->getY() << ")" << endl; break;
    };
}

void NAVnode::trackCollisionDetected(ListNode* nextNode, Obstacle* obstacle){
    auto pointOfInterest = calculatePointOfInterest(nextNode->previous, nextNode, obstacle);
    switch (circumnavigationStyle){
        case(reroute): generateDetour(pointOfInterest[0], pointOfInterest[1], nextNode, obstacle); break;
        case(trackCrawling): {
            ListNode* segmentStart = nextNode->previous;
            // to start, the process is identical to when a waypoint collision is detected, but it's even simpler since we know that there isn't a turn or a bend
            // in the route midway through the obstacle
                
            /* create a new node to store another waypoint
               to get this new waypoint, we will move back 1 Radii + 1m from the Point of Interest (POI) towards the previous waypoint
               if not, the new waypoint will be further from the edge of the obstacle
               this was done for simplicity, but it really doesn't matter because the further the POI is from the centre, the less drastic the course correction anyway
               so we'll still be following the original route reasonably closely
            */ 
            ListNode* newWaypoint = new ListNode(nullptr, nullptr, 
                new Waypoint(pointOfInterest[0] - (obstacle->getRadius() + 1) * pointOfInterest[2], pointOfInterest[1] - (obstacle->getRadius() + 1) * pointOfInterest[3]));
            
            //check if our new waypoint is colliding with an obstacle
            if (isWaypointColliding(newWaypoint)){
                // we'll try the same thing again, taking care not to go past the previous waypoint

                // check if the previous waypoint is within 1 radii of the waypoint we just made
                double dx = nextNode->previous->point->getX() - newWaypoint->point->getX();
                double dy = nextNode->previous->point->getY() - newWaypoint->point->getY();
                delete newWaypoint;
                
                if (!(sqrt(dx * dx + dy * dy) < obstacle->getRadius())){
                    // if moving back one more radii will NOT make a new waypoint behind the first one, then we add a new waypoint 1 radii back from the last one, else we simply omit it
                    newWaypoint = new ListNode(nullptr, nullptr, 
                        new Waypoint(
                            pointOfInterest[0] - (obstacle->getRadius() * 2 + 1) * pointOfInterest[2],
                            pointOfInterest[1] - (obstacle->getRadius() * 2 + 1) * pointOfInterest[3]
                        ));
                    routes[routeToEdit].addBefore(newWaypoint, nextNode);
                }
            }
            else{
                routes[routeToEdit].addBefore(newWaypoint, nextNode);
            }

            newWaypoint = new ListNode(nullptr, nullptr, 
                new Waypoint(pointOfInterest[0] + (obstacle->getRadius() + 1) * pointOfInterest[2], pointOfInterest[1] + (obstacle->getRadius() + 1) * pointOfInterest[3]));
                
            //check if our new waypoint is colliding with an obstacle
            if (isWaypointColliding(newWaypoint)){
                // we'll try the same thing again, taking care not to go past the previous waypoint

                // check if the previous waypoint is within 1 radii of the waypoint we just made
                double dx = nextNode->point->getX() - newWaypoint->point->getX();
                double dy = nextNode->point->getY() - newWaypoint->point->getY();
                delete newWaypoint;
                
                if (!(sqrt(dx * dx + dy * dy) < obstacle->getRadius())){
                    // if moving back one more radii will NOT make a new waypoint behind the first one, then we add a new waypoint 1 radii back from the last one, else we simply omit it
                    newWaypoint = new ListNode(nullptr, nullptr, 
                        new Waypoint(pointOfInterest[0] + (obstacle->getRadius() * 2 + 1) * pointOfInterest[2], pointOfInterest[1] + (obstacle->getRadius() * 2 + 1) * pointOfInterest[3]));
                    
                    routes[routeToEdit].addBefore(newWaypoint, nextNode);
                }
            }
            else{
                routes[routeToEdit].addBefore(newWaypoint, nextNode);
            }

            // The waypoints above stop short of the obstacle on either side, but the segment between them
            // still crosses it. Reroute that segment (and any other new one) as the description promises.
            for (ListNode* a = segmentStart; a != nullptr && a != nextNode && a->next != nullptr; a = a->next) {
                checkTrackCollisions(a, a->next, obstacle);
            }
        }; break;
        case(automatic_Circumnavigation_Off): cout << "collision Detected with Obstacle (" << obstacle->getX() << ", " << obstacle->getY() << ")" << endl; break;
    };
}

void NAVnode::generateDetour(double pointOfInterest_X, double pointOfInterest_Y, ListNode* nextNode, Obstacle* obstacle){
    if (nextNode == nullptr || nextNode->previous == nullptr || obstacle == nullptr) {
        return;
    }
    // each detour re-checks the two segments it creates, so bound the recursion
    if (detourDepth >= kMaxDetourDepth) {
        cout << "detour recursion limit reached near obstacle (" << obstacle->getX() << ", " << obstacle->getY()
             << "); route left as is" << endl;
        return;
    }
    struct DepthGuard { int& d; explicit DepthGuard(int& v) : d(v) { ++d; } ~DepthGuard() { --d; } } depthGuard(detourDepth);

    // unit vector from the POI towards the centre (the track's normal if the track passes through the centre)
    const std::array<double, 2> unitVector = nav_detour::detourDirection(
        obstacle->getX(), obstacle->getY(), pointOfInterest_X, pointOfInterest_Y,
        nextNode->point->getX() - nextNode->previous->point->getX(),
        nextNode->point->getY() - nextNode->previous->point->getY());
    if (unitVector[0] == 0.0 && unitVector[1] == 0.0) {
        return;
    }
    // candidates are measured from the obstacle CENTRE, so they sit exactly that far from it
    auto candidate = [&](double distance, int side) {
        const auto p = nav_detour::detourPoint(obstacle->getX(), obstacle->getY(), unitVector, distance, side);
        return new ListNode(nullptr, nullptr, new Waypoint(p[0], p[1]));
    };
    // insert before nextNode, then re-check the segments into and out of the new waypoint
    auto insertAndRecheck = [&](ListNode* waypoint) {
        routes[routeToEdit].addBefore(waypoint, nextNode);
        checkTrackCollisions(waypoint->previous, waypoint, obstacle);
        checkTrackCollisions(waypoint, nextNode, obstacle);
    };
    double dx, dy;

    // following obstacles are used in the worst case scenario of chaining obstacles
    int obstacle1_index;
    int obstacle2_index;

    // try to put new waypoint 1m past the nearest apex
    ListNode* newWaypoint = candidate(obstacle->getRadius() + 1, -1);

    if (!isWaypointColliding(newWaypoint)){
        // putting it 1m past the nearest apex works fine
        insertAndRecheck(newWaypoint);
    }
    else {
        // try to put the new waypoint at the nearest apex
        obstacle1_index = findWaypointCollision(newWaypoint->point);
        delete newWaypoint;
        newWaypoint = candidate(obstacle->getRadius(), -1);

        if (!isWaypointColliding(newWaypoint)){
            // putting it at the nearest apex works fine
            insertAndRecheck(newWaypoint);
        }
        else {
            // try putting it 1m past the far apex
            delete newWaypoint;
            newWaypoint = candidate(obstacle->getRadius() + 1, +1);

            if (!isWaypointColliding(newWaypoint)){
                // putting it 1m past the far apex works fine
                insertAndRecheck(newWaypoint);
            }
            else {
                // try putting it at the far apex
                obstacle2_index = findWaypointCollision(newWaypoint->point);
                delete newWaypoint;
                newWaypoint = candidate(obstacle->getRadius(), +1);

                if (!isWaypointColliding(newWaypoint)){
                    // putting it at the far apex works fine
                    insertAndRecheck(newWaypoint);
                }
                else {
                    /*
                        At this point, we are dealing with a chain of obstacles where the first obstacle we looked at is not at the end
                        This means that we must change our strategy
                        now, we will leap frog along to find the ends of the chain and we will create 2 new waypointsd at either end
                        we will then compare these to the POI to find out which one is the shortest detour
                        the waypoint that takes us the least off track will be added and the other will be removed
                    */
                    int originalObstacleIndex = getIndexOfObstacle(obstacle);
                    if (originalObstacleIndex < 0 ||
                        obstacle1_index < 0 ||
                        obstacle2_index < 0) {
                        delete newWaypoint;
                        return;
                    }

                    Waypoint* alternative1 = findAlternateRoute(originalObstacleIndex, obstacle1_index);
                    Waypoint* alternative2 = findAlternateRoute(originalObstacleIndex, obstacle2_index);
                    delete newWaypoint;
                    if (alternative1 == nullptr || alternative2 == nullptr) {
                        delete alternative1;
                        delete alternative2;
                        return;
                    }
                        
                    // calculate the distance to alternative 1
                    dx = alternative1->getX() - pointOfInterest_X;
                    dy = alternative1->getY() - pointOfInterest_Y;
                    double length1 = sqrt(dx * dx + dy * dy);

                    // calculate the distancec to alternative 2
                    dx = alternative2->getX() - pointOfInterest_X;
                    dy = alternative2->getY() - pointOfInterest_Y;
                    double length2 = sqrt(dx * dx + dy * dy);

                    // compare and keep the nearest one
                    if(length1 <= length2){
                        newWaypoint = new ListNode(nullptr, nullptr, alternative1);
                        delete alternative2;
                    }
                    else {
                        newWaypoint = new ListNode(nullptr, nullptr, alternative2);
                        delete alternative1;
                    }
                    insertAndRecheck(newWaypoint);
                }
            }
        }

    }
}

Waypoint* NAVnode::findAlternateRoute(int first, int second, int depth){
    if (depth > 16 || depth > obstacleCount) {
        return nullptr;
    }
    if (first < 0 || second < 0 || first >= obstacleCount || second >= obstacleCount) {
        return nullptr;
    }
    // define unit vector going from the centre of the original obstacle to the next one
    double dx = obstacles[second]->getX() - obstacles[first]->getX();
    double dy = obstacles[second]->getY() - obstacles[first]->getY();
    double length = sqrt(dx * dx + dy * dy);
    if (length < 1e-9) {
        return nullptr;
    }
    double* unitVector = new double[2]{dx / length, dy / length};

    Waypoint* possibleWaypoint = new Waypoint(obstacles[second]->getX() + (obstacles[second]->getRadius() + 1) * unitVector[0], obstacles[second]->getY() + (obstacles[second]->getRadius() + 1) * unitVector[1]);

    if(!isWaypointColliding(possibleWaypoint)){
        // We found a potential waypoint
        delete[] unitVector;
        return possibleWaypoint;
    }
    else {
        //try the same trick as before where we put the new point right on the edge of the obstacle to thread the nedle between 2 adjacent obstacles
        delete possibleWaypoint;
        possibleWaypoint = new Waypoint(obstacles[second]->getX() + obstacles[second]->getRadius() * unitVector[0], obstacles[second]->getY() + obstacles[second]->getRadius() * unitVector[1]);
            
        if(!isWaypointColliding(possibleWaypoint)){
            // we found a potential waypoint
            delete[] unitVector;
            return possibleWaypoint;
        }

        else {
            //the chain continues. if this is the case, we recursively call this function until we get to the end
            delete[] unitVector;
            int indexOfNextObstacle = findWaypointCollision(possibleWaypoint);
            delete possibleWaypoint;

            if (indexOfNextObstacle < 0 || indexOfNextObstacle >= obstacleCount || indexOfNextObstacle == second) {
                return nullptr;
            }

            return findAlternateRoute(second, indexOfNextObstacle, depth + 1);
        }
    }
}

array<double, 4> NAVnode::calculatePointOfInterest(ListNode* start, ListNode* end, Obstacle* obstacle){
    double x, y, x_hat, y_hat;

    // calculate vectors to the next waypoint and to the obstacle
    double* vectorToNextWaypoint = new double[2]{end->point->getX() - start->point->getX(), end->point->getY() - start->point->getY()};
    double* vectorToObstacle = new double[2]{obstacle->getX() - start->point->getX(), obstacle->getY() - start->point->getY()};

    double segmentLength = std::sqrt(
    vectorToNextWaypoint[0] * vectorToNextWaypoint[0] +
    vectorToNextWaypoint[1] * vectorToNextWaypoint[1]
        );

    if (segmentLength < 1e-9) {
        delete[] vectorToNextWaypoint;
        delete[] vectorToObstacle;

        return {
            start->point->getX(),
            start->point->getY(),
            0.0,
            0.0
        };
    }
    // calculate unit vector representing the direction of the route
    double* unitVectorToNextWaypoint = new double[2]{
    vectorToNextWaypoint[0] / segmentLength,
    vectorToNextWaypoint[1] / segmentLength
        };  
    // project one vector onto the other to find out how far along the route the POI is
    double projectionLength =
        vectorToObstacle[0] * unitVectorToNextWaypoint[0] +
        vectorToObstacle[1] * unitVectorToNextWaypoint[1];

    projectionLength = std::clamp(
        projectionLength,
        0.0,
        segmentLength
    );
        
    // use the projection length and the direction to get the coordinates of the POI
    x = start->point->getX() + projectionLength * unitVectorToNextWaypoint[0];
    y = start->point->getY() + projectionLength * unitVectorToNextWaypoint[1];
        
    x_hat = unitVectorToNextWaypoint[0];
    y_hat = unitVectorToNextWaypoint[1];

    // free up memory
    delete[] vectorToNextWaypoint;
    delete[] vectorToObstacle;
    delete[] unitVectorToNextWaypoint;

    return {x, y, x_hat, y_hat};
}

int NAVnode::getIndexOfObstacle(Obstacle* thing){
    for(int i = 0; i < obstacleCount; i++){
        if(thing->getX() == obstacles[i]->getX() && thing->getY() == obstacles[i]->getY())
            return i;
    }
    return -1;
}

std::string NAVnode::getRouteToFollowName(){
    string name;
    switch(routeToFollow){
        case Route1: name = "Route 1"; break;
        case Route2: name = "Route 2"; break;
        case Route3: name = "Route 3"; break;
        case Route4: name = "Route 4"; break;
        case Route5: name = "Route 5"; break;
    }
    return name;
}

std::string NAVnode::getRouteToEditName(){
    string name;
    switch(routeToEdit){
        case Route1: name = "Route 1"; break;
        case Route2: name = "Route 2"; break;
        case Route3: name = "Route 3"; break;
        case Route4: name = "Route 4"; break;
        case Route5: name = "Route 5"; break;
    }
    return name;
}

std::string NAVnode::getPreferenceName(){
    string name;
    switch(preference){
        case geodetic: name = "Geodetic"; break;
        case local: name = "Local"; break;
        case earthCentred: name = "Earth Centred Earth Fixed"; break;
    }
    return name;
}

std::string NAVnode::getCircumnavigationStyleName(){
    string name;
    switch(circumnavigationStyle){
        case reroute: name = "Reroute"; break;
        case trackCrawling: name = "Track Crawling"; break;
        case automatic_Circumnavigation_Off: name = "Automatic Circumnavigation Off"; break;
    }
    return name;
}

static bool eraseObstacleAt(Obstacle** obstacles, int& obstacleCount, int index) {
    if (index < 0 || index >= obstacleCount) return false;
    if (index != obstacleCount - 1) {
        Obstacle* temp = obstacles[index];
        obstacles[index] = obstacles[obstacleCount - 1];
        obstacles[obstacleCount - 1] = temp;
    }
    delete obstacles[--obstacleCount];
    obstacles[obstacleCount] = nullptr;
    return true;
}

bool NAVnode::removeLocalObstacle_helper(double x, double y){
    for(int i = 0; i < obstacleCount; i++){
        if (std::abs(obstacles[i]->getX() - x) < 1e-6 && std::abs(obstacles[i]->getY() - y) < 1e-6){
            return eraseObstacleAt(obstacles, obstacleCount, i);
        }
    }
    return false;
}

bool NAVnode::removeGeodeticObstacle_helper(double latitude, double longitude){
    for(int i = 0; i < obstacleCount; i++){
        if (std::abs(obstacles[i]->getLatitude() - latitude) < 1e-6 && std::abs(obstacles[i]->getLongitude() - longitude) < 1e-6){
            return eraseObstacleAt(obstacles, obstacleCount, i);
        }
    }
    return false;
}

bool NAVnode::removeEarthCentredObstacle_helper(double x, double y, double z){
    for(int i = 0; i < obstacleCount; i++){
        if (std::abs(obstacles[i]->getECEF_x() - x) < 1e-6 &&
            std::abs(obstacles[i]->getECEF_y() - y) < 1e-6 &&
            std::abs(obstacles[i]->getECEF_z() - z) < 1e-6){
            return eraseObstacleAt(obstacles, obstacleCount, i);
        }
    }
    return false;
}

    // Constructor
NAVnode::NAVnode() : Node("NAVnode"), count_(0) {
        
    currentPosition = new Waypoint(0.0, 0.0);

    debug = false;
    travellingForwards = true;   // the target starts at the head of the route, so travel towards the tail
    routeFinished = false;
        
    obstacleCount = 0;
    obstacleLimit = 10;
    waypointCount = 0;
    pointsLogged = 0;
    cyclesSinceLogUpdated = 0;
    

    date = "";

    obstacles = new Obstacle*[obstacleLimit];
    for (int i = 0; i < obstacleLimit; i++) {
        obstacles[i] = nullptr;
    }

    for (int i = 0; i < 5; i++) {
        lastFiveWaypoints[i] = new Waypoint(currentPosition->getX(), currentPosition->getY());
    }
        
    preference = local;
    circumnavigationStyle = automatic_Circumnavigation_Off;
    routeToFollow = Route1;
    routeToEdit = Route1;

    // Ensure nextWaypoint is initialized (may be null if route is empty)
    nextWaypoint = routes[routeToFollow].getHead();

    std::string gnss_port = this->declare_parameter<std::string>("gnss_port", "/dev/ttyUSB0");
    int gnss_baud = this->declare_parameter<int>("gnss_baudrate", 38400);
    verify_nmea_checksum_ = this->declare_parameter<bool>("verify_nmea_checksum", true);
    use_gps_fix_topic_ = this->declare_parameter<bool>("use_gps_fix_topic", false);
    std::string gps_fix_topic = this->declare_parameter<std::string>("gps_fix_topic", "/gps/fix");

    GNSS = nullptr;
    if (use_gps_fix_topic_) {
        gps_fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_fix_topic, 10, std::bind(&NAVnode::gpsFixCallback, this, _1));
    } else {
        GNSS = new SerialPort(gnss_port, gnss_baud);
        GNSS->begin();
    }

    // create topics
    // here, we're using a queue size of 1 because it is more desireable to lose some data
    // than to be using outdated position data
    DisplayInfoPublisher = this->create_publisher<navigation_interfaces::msg::DisplayInfo>("NAVDisplayInfo", 10);


    // create services
    StopNavigatingServer      = this->create_service<VoidService>("StopNavigating", std::bind(&NAVnode::stopNavigating, this, _1, _2));
    SelectRouteToFollowServer = this->create_service<SelectRoute>("SelectRouteToFollow", std::bind(&NAVnode::selectRouteToFollow, this, _1, _2));
    SelectRouteToEditServer   = this->create_service<SelectRoute>("SelectRouteToEdit", std::bind(&NAVnode::selectRouteToEdit, this, _1, _2));
    ResetHomeServer           = this->create_service<ResetHome>  ("ResetHome",      std::bind(&NAVnode::resetHome, this, _1, _2));
    ClearRouteServer          = this->create_service<VoidService>("ClearRoute",     std::bind(&NAVnode::clearRoute, this, _1, _2));
        
    ToggleDebugServer                 = this->create_service<VoidService>("ToggleDebug",                 std::bind(&NAVnode::toggleDebug, this, _1, _2));
    TogglePreferenceServer            = this->create_service<VoidService>("TogglePreference",            std::bind(&NAVnode::togglePreference, this, _1, _2));
    ToggleRouteToFollowServer         = this->create_service<VoidService>("ToggleRouteToFollow",         std::bind(&NAVnode::toggleRouteToFollow, this, _1, _2));
    ToggleRouteToEditServer           = this->create_service<VoidService>("ToggleRouteToEdit",           std::bind(&NAVnode::toggleRouteToEdit, this, _1, _2));
    ToggleCircumnavigationStyleServer = this->create_service<VoidService>("ToggleCircumnavigationStyle", std::bind(&NAVnode::toggleCircumnavigationStyle, this, _1, _2));
    ToggleDirectionServer             = this->create_service<VoidService>("ToggleDirection",             std::bind(&NAVnode::toggleDirection, this, _1, _2));

    TargetNextWaypointServer      = this->create_service<VoidService>("TargetNextWaypoint",      std::bind(&NAVnode::targetNextWaypoint, this, _1, _2));
    TargetPreviousWaypointServer  = this->create_service<VoidService>("TargetPreviousWaypoint",  std::bind(&NAVnode::targetPreviousWaypoint, this, _1, _2));

    AddLocalWaypointServer        = this->create_service<AddLocalWaypoint>       ("AddLocalWaypoint",        std::bind(&NAVnode::addLocalWaypoint, this, _1, _2));
    AddLocalWaypointAtIndexServer = this->create_service<AddLocalWaypointAtIndex>("AddLocalWaypointAtIndex", std::bind(&NAVnode::addLocalWaypointAtIndex, this, _1, _2));
    AddLocalObstacleServer        = this->create_service<AddLocalObstacle>       ("AddLocalObstacle",        std::bind(&NAVnode::addLocalObstacle, this, _1, _2));

    AddGeodeticWaypointServer        = this->create_service<AddGeodeticWaypoint>       ("AddGeodeticWaypoint",        std::bind(&NAVnode::addGeodeticWaypoint, this, _1, _2));
    AddGeodeticWaypointAtIndexServer = this->create_service<AddGeodeticWaypointAtIndex>("AddGeodeticWaypointAtIndex", std::bind(&NAVnode::addGeodeticWaypointAtIndex, this, _1, _2));
    AddGeodeticObstacleServer        = this->create_service<AddGeodeticObstacle>       ("AddGeodeticObstacle",        std::bind(&NAVnode::addGeodeticObstacle, this, _1, _2));

    AddEarthCentredWaypointServer        = this->create_service<AddEarthCentredWaypoint>       ("AddEarthCentredWaypoint",        std::bind(&NAVnode::addEarthCentredWaypoint, this, _1, _2));
    AddEarthCentredWaypointAtIndexServer = this->create_service<AddEarthCentredWaypointAtIndex>("AddEarthCentredWaypointAtIndex", std::bind(&NAVnode::addEarthCentredWaypointAtIndex, this, _1, _2));
    AddEarthCentredObstacleServer        = this->create_service<AddEarthCentredObstacle>       ("AddEarthCentredObstacle",        std::bind(&NAVnode::addEarthCentredObstacle, this, _1, _2));

    RemoveLocalServer        = this->create_service<RemoveLocal>        ("RemoveLocal",        std::bind(&NAVnode::removeLocal, this, _1, _2));

    RemoveLocalObstacleServer        = this->create_service<RemoveLocal>        ("RemoveLocalObstacle",        std::bind(&NAVnode::removeLocalObstacle, this, _1, _2));
    RemoveGeodeticObstacleServer     = this->create_service<RemoveGeodetic>     ("RemoveGeodeticObstacle",     std::bind(&NAVnode::removeGeodeticObstacle, this, _1, _2));
    RemoveEarthCentredObstacleServer = this->create_service<RemoveEarthCentred> ("RemoveEarthCentredObstacle", std::bind(&NAVnode::removeEarthCentredObstacle, this, _1, _2));

    RemoveLocalWaypointServer        = this->create_service<RemoveLocal>        ("RemoveLocalWaypoint",        std::bind(&NAVnode::removeLocalWaypoint, this, _1, _2));
    RemoveGeodeticWaypointServer     = this->create_service<RemoveGeodetic>     ("RemoveGeodeticWaypoint",     std::bind(&NAVnode::removeGeodeticWaypoint, this, _1, _2));
    RemoveEarthCentredWaypointServer = this->create_service<RemoveEarthCentred> ("RemoveEarthCentredWaypoint", std::bind(&NAVnode::removeEarthCentredWaypoint, this, _1, _2));
    
    RemoveLastWaypointServer         = this->create_service<RemoveLast> ("RemoveLastWaypoint",         std::bind(&NAVnode::removeLastWaypoint, this, _1, _2));
    RemoveLastObstacleServer         = this->create_service<RemoveLast> ("RemoveLastObstacle",         std::bind(&NAVnode::removeLastObstacle, this, _1, _2));

    SendToGNSSServer   = this->create_service<SendToGNSS>   ("SendToGNSS",   std::bind(&NAVnode::sendToGNSS, this, _1, _2));
    ReadFromGNSSServer = this->create_service<ReadFromGNSS> ("ReadFromGNSS", std::bind(&NAVnode::readFromGNSS, this, _1, _2));

    // create action server
    FollowRouteServer = rclcpp_action::create_server<NAVaction>(
        this,
        "FollowRoute",
        std::bind(&NAVnode::followRoute_Goal, this, _1, _2),
        std::bind(&NAVnode::followRoute_Cancel, this, _1),
        std::bind(&NAVnode::followRoute_Accepted, this, _1)
    );

    // main timer will process gnss data 5 times per second
    timer = this->create_wall_timer(200ms , std::bind(&NAVnode::mainTimer, this));

    }

NAVnode::~NAVnode() {
    ListNode* current = pointsVisited.getHead();
    int count = 0;
    string fileName = "captains log " + date + ".txt";
    std::filesystem::path directory = "src/navigation_system/captainsLogs";
    std::filesystem::path filePath = directory / fileName;

    try {
        std::filesystem::create_directories(directory); // ensure the captains logs directory exists

        std::ofstream outputFile(filePath); // create and open file

        if (outputFile.is_open()){ // verity the file was created

            while(current != nullptr){
                /*
                    Print out the location of each waypoint in both local and geodetic coordinates
                    local coordinates are surrounded by parenteses
                    geodetic coordinates are surrounded by braces
                    ex: Waypoint 7: (5, 7) {5013.21201 N, 11913.15950 W}
                */
                outputFile << "Waypoint " << ++count << ": "
                           << "(" << current->point->getX() << ", " << current->point->getY() << ") "
                           << "{" << current->point->getLatitude() << " N, " << current->point->getLongitude() << " W}\n";   // was currentPosition for every line
                
                current = current->next;
            }
        }
        else{ // if we cannot print it to a file, we will print it to the console and hopefully we remember to record it form there
            while(current != nullptr){
                /*
                    Print out the location of each waypoint in both local and geodetic coordinates
                    local coordinates are surrounded by parenteses
                    geodetic coordinates are surrounded by braces
                    ex: Waypoint 7: (5, 7) {5013.21201 N, 11913.15950 W}
                */
                cout << "Waypoint " << ++count << ": "
                     << "(" << current->point->getX() << ", " << current->point->getY() << ") "
                     << "{" << current->point->getLatitude() << " N, " << current->point->getLongitude() << " W}\n";   // was currentPosition for every line

                current = current->next;
            }
        }
    } catch (const std::filesystem::filesystem_error& e){
        cout << "Filesystem error: " << e.what() << endl;
    }

    cout << "number of waypoints visited: " << count << endl;

    delete GNSS;
    GNSS = nullptr;
    delete currentPosition;
    currentPosition = nullptr;
    for (int i = 0; i < 5; ++i) {
        delete lastFiveWaypoints[i];
        lastFiveWaypoints[i] = nullptr;
    }
    if (obstacles != nullptr) {
        for (int i = 0; i < obstacleCount; ++i) {
            delete obstacles[i];
            obstacles[i] = nullptr;
        }
        delete[] obstacles;
        obstacles = nullptr;
    }
}

    // timers
void NAVnode::validateTarget(){
    ListNode* node = routes[routeToFollow].getHead();
    if (nextWaypoint != nullptr) {
        // pointer comparison only: the node may already have been deleted by a remove/clear service
        for (ListNode* n = node; n != nullptr; n = n->next) {
            if (n == nextWaypoint) return;
        }
        nextWaypoint = nullptr;
    }
    if (!routeFinished) {
        nextWaypoint = travellingForwards ? routes[routeToFollow].getHead() : routes[routeToFollow].getTail();
    }
}

void NAVnode::mainTimer(){
    // nextWaypoint used to be set once, in the constructor, while every route was still empty, so no
    // waypoint added later was ever targeted; a removed waypoint also left it dangling.
    validateTarget();
    updatePositonData();

    if (currentPosition == nullptr) {
        RCLCPP_WARN(this->get_logger(), "Current position is not available yet; skipping timer update.");
        return;
    }

    // check if we have arrived at the waypoint (we will count "arrival" as being within 2m)
    if (nextWaypoint != nullptr && nextWaypoint->point != nullptr) {

        double dx = nextWaypoint->point->getX() - currentPosition->getX();
        double dy = nextWaypoint->point->getY() - currentPosition->getY();

        if (sqrt(dx * dx + dy * dy) < 2.0){
            if(travellingForwards)
                nextWaypoint = nextWaypoint->next;
            else
                nextWaypoint = nextWaypoint->previous;
            if (nextWaypoint == nullptr) routeFinished = true;   // do not start the route again by itself
        }
    }
    updateFollowRouteGoal();

    if (debug){
        // debug mode will print the current value for all our variables to the ros terminal thatt the navigation node is active in
        
        printf("Current coordinates: %f N %f W \n", currentPosition->getLatitude(), currentPosition->getLongitude());
        printf("Route to Follow: %s Route to Edit: %s Coordinate Preference: %s Circumnavigation style: %s \n", getRouteToFollowName().c_str(), getRouteToEditName().c_str(), getPreferenceName().c_str(), getCircumnavigationStyleName().c_str());
        printf("Number of Waypoints: %d Number of Obstacles: %d Current Maximun number of obstacles: %d \n\n\n", waypointCount, obstacleCount, obstacleLimit);
    }

    // log data
    /*
        we will save data that will record where the rover went over the course of the task
        this data will be saved to a file called "captainsLog.txt" when this node is shutdown

        when testing, this file got to be 107Gb, that is absolutely ridiculous
        to avoid this, we will only record data every 20 seconds and we will cap the amount of data we store to 250 waypoints
        sampling at a rate of 20 seconds will give us about 180 waypoints to sift through after a 1hour excursion
        this is plenty of detail, and not so much data that it becomes unmanageable
    */
    if(++cyclesSinceLogUpdated >= 100 && pointsLogged < 250){
        // store a copy: currentPosition is deleted and replaced every tick, and ListNode deletes its point,
        // so adding the pointer itself left dangling pointers and a double free at shutdown
        pointsVisited.add(new Waypoint(*currentPosition));
        pointsLogged += 1;
        cyclesSinceLogUpdated = 0;
    }

   
}

void NAVnode::gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg){
    if (msg->status.status < 0 || !std::isfinite(msg->latitude) || !std::isfinite(msg->longitude)) {
        have_prev_fix_ = false;
        return;
    }
    if (std::abs(msg->latitude) < 1e-4 && std::abs(msg->longitude) < 1e-4) {
        return;
    }
    const double alt = std::isfinite(msg->altitude) ? msg->altitude : 0.0;
    Waypoint* updated = new Waypoint(msg->longitude, msg->latitude, alt, true);
    const int64_t stamp_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL
                           + static_cast<int64_t>(msg->header.stamp.nanosec);
    if (have_prev_fix_ && Waypoint::isHomeSet() && stamp_ns > prev_fix_ns_) {
        const double dt_s = static_cast<double>(stamp_ns - prev_fix_ns_) * 1e-9;
        const double dx = updated->getX() - prev_fix_x_;
        const double dy = updated->getY() - prev_fix_y_;
        const double dist = std::hypot(dx, dy);
        if (dt_s > 0.05) {
            lastSpeed = (dist / dt_s) * 3.6;  // km/h
            if (dist > 0.05) {
                double h = std::atan2(dx, dy);
                if (h < 0.0) h += 2.0 * Waypoint::pi;
                lastHeading = h;
            }
        }
    }
    if (Waypoint::isHomeSet()) {
        prev_fix_x_ = updated->getX();
        prev_fix_y_ = updated->getY();
        prev_fix_ns_ = stamp_ns;
        have_prev_fix_ = true;
    }
    hasFix = true;
    delete currentPosition;
    currentPosition = updated;
}

void NAVnode::updatePositonData(){
    double heading = lastHeading;
    double speed = lastSpeed;

    if (GNSS != nullptr) {
        // Drain up to 16 buffered chunks so a burst of NMEA sentences settles within one tick.
        for (int reads = 0; reads < 16; ++reads) {
            std::string chunk = GNSS->read();
            if (chunk.empty()) break;
            gnssBuffer += chunk;
        }
        if (gnssBuffer.size() > 4096) gnssBuffer.erase(0, gnssBuffer.size() - 4096);
        string message = gnssBuffer;
        size_t lastLineEnd = gnssBuffer.find_last_of('\n');
        if (lastLineEnd != std::string::npos) gnssBuffer.erase(0, lastLineEnd + 1);   // keep only the unfinished sentence

        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;

        bool havePosition = false;
        try {
            std::vector<std::string> rmc = nmeaFields(message, "RMC", verify_nmea_checksum_);
            std::vector<std::string> gga = nmeaFields(message, "GGA", verify_nmea_checksum_);
            std::vector<std::string> vtg = nmeaFields(message, "VTG", verify_nmea_checksum_);

            // RMC: 1 time, 2 status, 3 latitude, 4 N/S, 5 longitude, 6 E/W, 7 speed (knots), 8 true track (deg), 9 date
            if (rmc.size() > 9 && rmc[2] == "A" && !rmc[3].empty() && !rmc[5].empty()) {
                latitude = nmeaToDegrees(stod(rmc[3]));
                if (rmc[4] == "S") latitude = -latitude;
                longitude = nmeaToDegrees(stod(rmc[5]));
                if (rmc[6] == "W") longitude = -longitude;
                havePosition = true;
                if (!rmc[7].empty()) speed = stod(rmc[7]) * 1.852;                     // knots to km/h
                if (!rmc[8].empty()) heading = stod(rmc[8]) * Waypoint::pi / 180.0;    // course over ground
                if (date.empty() && rmc[9].size() >= 6)                                 // ddmmyy, used in the log file name
                    date = rmc[9].substr(0, 2) + "-" + rmc[9].substr(2, 2) + "-" + rmc[9].substr(4, 2);
            }

            // GGA: 2 latitude, 3 N/S, 4 longitude, 5 E/W, 6 fix quality, 9 altitude (m)
            if (gga.size() > 9 && !gga[6].empty() && nav_nmea::isAcceptableGgaQuality(std::stoi(gga[6])) &&
                !gga[2].empty() && !gga[4].empty()) {
                latitude = nmeaToDegrees(stod(gga[2]));
                if (gga[3] == "S") latitude = -latitude;
                longitude = nmeaToDegrees(stod(gga[4]));
                if (gga[5] == "W") longitude = -longitude;
                if (!gga[9].empty()) altitude = stod(gga[9]);
                havePosition = true;
            }

            // VTG: 1 true track (deg), 7 speed (km/h); only used when RMC did not give them
            if (rmc.empty() && vtg.size() > 7) {
                if (!vtg[1].empty()) heading = stod(vtg[1]) * Waypoint::pi / 180.0;
                if (!vtg[7].empty()) speed = stod(vtg[7]);
            }
        }
        catch (...) { return; }   // a malformed number: skip this cycle rather than publish a wrong position
        lastSpeed = speed;
        lastHeading = heading;

        if (currentPosition != nullptr && havePosition && latitude != 0.0 && longitude != 0.0) {
            hasFix = true;
            delete currentPosition;
            currentPosition = new Waypoint(longitude, latitude, altitude, true); // constructor order is (longitude, latitude, altitude)
        } else if (currentPosition == nullptr) {
            currentPosition = new Waypoint(0.0, 0.0);
        }
    } else if (currentPosition == nullptr) {
        currentPosition = new Waypoint(0.0, 0.0);
    }

    auto gnssInformation = navigation_interfaces::msg::DisplayInfo();

    gnssInformation.heading = heading;
    gnssInformation.speed = speed;

    gnssInformation.preference = preference;
    gnssInformation.traveling_forwards = travellingForwards;

    gnssInformation.route2follow = routeToFollow;
    gnssInformation.route2edit = routeToEdit;

    gnssInformation.local_x = currentPosition->getX();
    gnssInformation.local_y = currentPosition->getY();

    if (nextWaypoint && nextWaypoint->point) {
        gnssInformation.waypoint_local_x = nextWaypoint->point->getX();
        gnssInformation.waypoint_local_y = nextWaypoint->point->getY();

        double dx = nextWaypoint->point->getX() - currentPosition->getX();
        double dy = nextWaypoint->point->getY() - currentPosition->getY();

        gnssInformation.range2waypoint = sqrt(dx * dx + dy * dy);

        if (gnssInformation.range2waypoint != 0.0) {
            dx /= gnssInformation.range2waypoint;
            dy /= gnssInformation.range2waypoint;
        } else {
            dx = 0.0; dy = 0.0;
        }

        // compass bearing to the waypoint: radians clockwise from north (+y), in [0, 2*pi)
        double bearing = std::atan2(dx, dy);
        if (bearing < 0.0) bearing += 2.0 * Waypoint::pi;
        gnssInformation.bearing2waypoint = bearing;
    } else {
        gnssInformation.waypoint_local_x = 0.0;
        gnssInformation.waypoint_local_y = 0.0;
        gnssInformation.range2waypoint = 1e12;
        gnssInformation.bearing2waypoint = 0.0;
    }

    
    
    switch (preference){
        case local : {
            gnssInformation.rover_x = currentPosition->getX();
            gnssInformation.rover_y = currentPosition->getY();
            gnssInformation.rover_z = 0;
            if (nextWaypoint) {
                gnssInformation.next_waypoint_x = nextWaypoint->point->getX();
                gnssInformation.next_waypoint_y = nextWaypoint->point->getY();
                gnssInformation.next_waypoint_z = 0;
            } else {
                gnssInformation.next_waypoint_x = 0.0;
                gnssInformation.next_waypoint_y = 0.0;
                gnssInformation.next_waypoint_z = 0.0;
            }
            break;
        }
        case geodetic : {
            gnssInformation.rover_x = currentPosition->getLatitude();
            gnssInformation.rover_y = currentPosition->getLongitude();
            gnssInformation.rover_z = currentPosition->getAltitude();
            if (nextWaypoint) {
                gnssInformation.next_waypoint_x = nextWaypoint->point->getLatitude();
                gnssInformation.next_waypoint_y = nextWaypoint->point->getLongitude();
                gnssInformation.next_waypoint_z = nextWaypoint->point->getAltitude();
            } else {
                gnssInformation.next_waypoint_x = 0.0;
                gnssInformation.next_waypoint_y = 0.0;
                gnssInformation.next_waypoint_z = 0.0;
            }
            break;
        }
        case earthCentred : {
            gnssInformation.rover_x = currentPosition->getECEF_x();
            gnssInformation.rover_y = currentPosition->getECEF_y();
            gnssInformation.rover_z = currentPosition->getECEF_z();
            if (nextWaypoint) {
                gnssInformation.next_waypoint_x = nextWaypoint->point->getECEF_x();
                gnssInformation.next_waypoint_y = nextWaypoint->point->getECEF_y();
                gnssInformation.next_waypoint_z = nextWaypoint->point->getECEF_z();
            } else {
                gnssInformation.next_waypoint_x = 0.0;
                gnssInformation.next_waypoint_y = 0.0;
                gnssInformation.next_waypoint_z = 0.0;
            }
            break;
        }
    }
    
    if (DisplayInfoPublisher) {
        try {
            DisplayInfoPublisher->publish(gnssInformation);
        } catch (const rclcpp::exceptions::RCLError & e) {
            RCLCPP_WARN(this->get_logger(), "DisplayInfo publish failed: %s", e.what());
        }
    }

}

// removed the velocity topic, so the next function is no longer needed (and will cause errors)

// void NAVnode::updateVelocity(){
//     double x = 0.0, y = 0.0, dX = 0.0, dY = 0.0;

//     if (!lastFiveWaypoints[0]) return;

//     for(int i = 1; i < 5; i++){
//         if (!lastFiveWaypoints[i]) continue;
//         dX += lastFiveWaypoints[i]->getX() - lastFiveWaypoints[0]->getX();
//         dY += lastFiveWaypoints[i]->getY() - lastFiveWaypoints[0]->getY();
//     }

//     x = std::abs(dX / 4.0); 
//     y = std::abs(dY / 4.0);

//     // calculate speed
//     currentVelocity.speed = std::sqrt(x * x + y * y) / 4.0;

//     // calculate heading
//     if (dX == 0.0 && dY == 0.0) {
//         // stationary
//     } else if (x > 0.0) {
//         double ratio = y / x;
//         if (ratio > 1.0) ratio = 1.0;
//         double angleDeg = currentPosition->radToDeg(std::asin(ratio));

//         if (dX > 0 && dY >= 0) currentVelocity.heading = 90.0 - angleDeg;
//         else if (dX < 0 && dY >= 0) currentVelocity.heading = 270.0 + angleDeg;
//         else if (dX < 0 && dY < 0) currentVelocity.heading = 270.0 - angleDeg;
//         else if (dX > 0 && dY < 0) currentVelocity.heading = 90.0 + angleDeg;
//     }
// }

    // service callback functions
void NAVnode::stopNavigating(const std::shared_ptr<VoidService::Request> request, 
                                std::shared_ptr<VoidService::Response> response){

}

void NAVnode::togglePreference(const std::shared_ptr<VoidService::Request> request, 
                                std::shared_ptr<VoidService::Response> response){
    switch(preference){
        case(local): preference = geodetic;          break;
        case(geodetic): preference = earthCentred;  break;
        default: preference = local;                 break;
    }
}

void NAVnode::toggleRouteToFollow(const std::shared_ptr<VoidService::Request> request, 
                            std::shared_ptr<VoidService::Response> response){
    switch(routeToFollow){
        case(Route1): routeToFollow = Route2; break;
        case(Route2): routeToFollow = Route3; break;
        case(Route3): routeToFollow = Route4; break;
        case(Route4): routeToFollow = Route5; break;
        default: routeToFollow = Route1;      break;
    }
    routeFinished = false;
    nextWaypoint = nullptr;   // validateTarget() picks the new route's first waypoint
}

void NAVnode::toggleRouteToEdit(const std::shared_ptr<VoidService::Request> request, 
                            std::shared_ptr<VoidService::Response> response){
    switch(routeToEdit){
        case(Route1): routeToEdit = Route2; break;
        case(Route2): routeToEdit = Route3; break;
        case(Route3): routeToEdit = Route4; break;
        case(Route4): routeToEdit = Route5; break;
        default: routeToEdit = Route1;      break;
    }
}

void NAVnode::toggleDebug(const std::shared_ptr<VoidService::Request> request, std::shared_ptr<VoidService::Response> response){
    debug = !debug;
}

void NAVnode::toggleDirection(const std::shared_ptr<VoidService::Request> request, std::shared_ptr<VoidService::Response> response){
    travellingForwards = !travellingForwards;
    routeFinished = false;
}

void NAVnode::toggleCircumnavigationStyle(const std::shared_ptr<VoidService::Request> request, std::shared_ptr<VoidService::Response> response){
    switch(circumnavigationStyle){
        case(reroute):       circumnavigationStyle = trackCrawling;                  break;
        case(trackCrawling): circumnavigationStyle = automatic_Circumnavigation_Off; break;
        default:             circumnavigationStyle = reroute;                        break;
    }
}

void NAVnode::targetNextWaypoint(const std::shared_ptr<VoidService::Request> request, std::shared_ptr<VoidService::Response> response){
    (void)request;
    (void)response;
    validateTarget();
    if (nextWaypoint == nullptr) {
        routeFinished = true;
        return;
    }

    if (travellingForwards)
        nextWaypoint = nextWaypoint->next;
    else
        nextWaypoint = nextWaypoint->previous;
    routeFinished = (nextWaypoint == nullptr);
}

void NAVnode::targetPreviousWaypoint(const std::shared_ptr<VoidService::Request> request, std::shared_ptr<VoidService::Response> response){
    (void)request;
    (void)response;
    validateTarget();
    if (nextWaypoint == nullptr) {
        routeFinished = true;
        return;
    }

    if(travellingForwards)
        nextWaypoint = nextWaypoint->previous;
    else 
        nextWaypoint = nextWaypoint->next;
    routeFinished = (nextWaypoint == nullptr);
}

void NAVnode::selectRouteToFollow(const std::shared_ptr<SelectRoute::Request> request, std::shared_ptr<SelectRoute::Response> response){
    switch(request->route){
        case Route1: routeToFollow = Route1; break;
        case Route2: routeToFollow = Route2; break;
        case Route3: routeToFollow = Route3; break;
        case Route4: routeToFollow = Route4; break;
        case Route5: routeToFollow = Route5; break;
        default: {response->output = "Error: invalid Route selection"; return;}
    }
    response->output = "Route successfully changed";   // was inside the switch after the default case, so never ran
    routeFinished = false;
    nextWaypoint = nullptr;   // validateTarget() picks the new route's first waypoint
}

void NAVnode::selectRouteToEdit(const std::shared_ptr<SelectRoute::Request> request, std::shared_ptr<SelectRoute::Response> response){
    switch(request->route){
        case Route1: routeToEdit = Route1; break;
        case Route2: routeToEdit = Route2; break;
        case Route3: routeToEdit = Route3; break;
        case Route4: routeToEdit = Route4; break;
        case Route5: routeToEdit = Route5; break;
        default: {response->output = "Error: invalid Route selection"; return;}
    }
    response->output = "Route successfully changed";
}

void NAVnode::rebaseWaypointsToHome(){
    have_prev_fix_ = false;
    if (currentPosition != nullptr) {
        currentPosition->EarthCentred_To_Local();
    }
    for (int i = 0; i < 5; ++i) {
        if (lastFiveWaypoints[i] != nullptr) {
            lastFiveWaypoints[i]->EarthCentred_To_Local();
        }
        for (ListNode* n = routes[i].getHead(); n != nullptr; n = n->next) {
            if (n->point != nullptr) {
                n->point->EarthCentred_To_Local();
            }
        }
    }
    for (ListNode* n = pointsVisited.getHead(); n != nullptr; n = n->next) {
        if (n->point != nullptr) {
            n->point->EarthCentred_To_Local();
        }
    }
    for (int i = 0; i < obstacleCount; ++i) {
        if (obstacles[i] != nullptr) {
            obstacles[i]->rebaseToHome();
        }
    }
}

void NAVnode::resetHome(const std::shared_ptr<ResetHome::Request> request, std::shared_ptr<ResetHome::Response> response){
    Waypoint* point = new Waypoint(request->longitude, request->latitude, 0, true); // constructor order is (longitude, latitude, altitude)

    if(point->setHome(point, request->password)) {
        rebaseWaypointsToHome();
        response->success = true;
    } else {
        response->success = false;
    }

    delete point;
}

bool NAVnode::ensureHome(){
    // Local coordinates are relative to home. The add services used to set home silently to the current
    // position whenever it was unset, even before any fix (home then became a meaningless point).
    if (Waypoint::isHomeSet()) return true;
    if (!hasFix || currentPosition == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Home is not set and there is no GNSS fix yet: request ignored. "
                                         "Call ResetHome first, or wait for a fix.");
        return false;
    }
    currentPosition->setHome(currentPosition, 0);
    rebaseWaypointsToHome();
    RCLCPP_WARN(this->get_logger(), "Home was not set: it is now the current position (lat %.7f, lon %.7f). "
                                    "Call ResetHome to choose it explicitly.",
                currentPosition->getLatitude(), currentPosition->getLongitude());
    return true;
}

void NAVnode::clearRoute(const std::shared_ptr<VoidService::Request> request, std::shared_ptr<VoidService::Response> response){
    routes[routeToEdit].clear();
    if (routeToEdit == routeToFollow) { nextWaypoint = nullptr; routeFinished = false; }
}

void NAVnode::addLocalWaypoint(const std::shared_ptr<AddLocalWaypoint::Request> request, 
                                std::shared_ptr<AddLocalWaypoint::Response> response){
    if (!ensureHome()) return;   // was a silent setHome(currentPosition), even with no fix
    routes[routeToEdit].add(new Waypoint(request->x, request->y));
}

void NAVnode::addLocalWaypointAtIndex(const std::shared_ptr<AddLocalWaypointAtIndex::Request> request, 
                                        std::shared_ptr<AddLocalWaypointAtIndex::Response> response){
    if (!ensureHome()) return;   // was a silent setHome(currentPosition), even with no fix
    routes[routeToEdit].add(new Waypoint(request->x, request->y), request->index);
}

void NAVnode::addLocalObstacle(const std::shared_ptr<AddLocalObstacle::Request> request, 
                                std::shared_ptr<AddLocalObstacle::Response> response){
    (void)response;

    if (!ensureHome()) return;   // was a silent setHome(currentPosition), even with no fix

    // check if the array of obstacles is full
    if (obstacleCount == obstacleLimit){
        int newLimit = obstacleLimit + 10;
        Obstacle** biggerArray = new Obstacle*[newLimit];

        for(int i = 0; i < obstacleCount; i++){
            biggerArray[i] = obstacles[i];
        }
        for(int i = obstacleCount; i < newLimit; i++){
            biggerArray[i] = nullptr; // this is so our newly expanded slots don't contain garbage pointers
        }

        // clean up memory and rename our new array (by reusing the old pointer);
        delete[] obstacles;
        obstacles = biggerArray;
        biggerArray = nullptr;
        obstacleLimit = newLimit;
    }

    // add new obstacle. post incrementing to handle indexing differences
    if (request->radius <= 0)
        obstacles[obstacleCount++] = new Obstacle(request->x, request->y);
    else
        obstacles[obstacleCount++] = new Obstacle(request->x, request->y, request->radius);

    checkWaypointCollisions();
    checkTrackCollisions();
}

void NAVnode::addGeodeticWaypoint(const std::shared_ptr<AddGeodeticWaypoint::Request> request, 
                                    std::shared_ptr<AddGeodeticWaypoint::Response> response){
    if (!ensureHome()) return;   // was a silent setHome(currentPosition), even with no fix
    routes[routeToEdit].add(new Waypoint(request->longitude, request->latitude, request->altitude, true));
}

void NAVnode::addGeodeticWaypointAtIndex(const std::shared_ptr<AddGeodeticWaypointAtIndex::Request> request, 
                                            std::shared_ptr<AddGeodeticWaypointAtIndex::Response> response){
    if (!ensureHome()) return;   // was a silent setHome(currentPosition), even with no fix
    routes[routeToEdit].add(new Waypoint(request->longitude, request->latitude, request->altitude, true), request->index);
}

void NAVnode::addGeodeticObstacle(const std::shared_ptr<AddGeodeticObstacle::Request> 
                                    request, std::shared_ptr<AddGeodeticObstacle::Response> response){
    (void)response;

    if (!ensureHome()) return;   // was a silent setHome(currentPosition), even with no fix

    // check if the array of obstacles is full
    if (obstacleCount == obstacleLimit){
        // increase make a larger array to store obstacles
        int newLimit = obstacleLimit + 10;
        Obstacle** biggerArray = new Obstacle*[newLimit];

        // copy over existing obstacles (by iterating through existing array)
        for(int i = 0; i < obstacleCount; i++){
            biggerArray[i] = obstacles[i];
        }
        for(int i = obstacleCount; i < newLimit; i++){
            biggerArray[i] = nullptr;
        }

        // clean up memory and rename our new array (by reusing the old pointer);
        delete[] obstacles;
        obstacles = biggerArray;
        obstacleLimit = newLimit;
    }

    // add new obstacle. post incrementing to handle indexing differences
    if (request->radius <= 0)
        obstacles[obstacleCount++] = new Obstacle(request->longitude, request->latitude, request->altitude, true);
    else
        obstacles[obstacleCount++] = new Obstacle(request->longitude, request->latitude, request->altitude, request->radius, true);

    checkWaypointCollisions();
    checkTrackCollisions();
}

void NAVnode::addEarthCentredWaypoint(const std::shared_ptr<AddEarthCentredWaypoint::Request> request, 
                                        std::shared_ptr<AddEarthCentredWaypoint::Response> response){
    if (!ensureHome()) return;   // was a silent setHome(currentPosition), even with no fix
    routes[routeToEdit].add(new Waypoint(request->x, request->y, request->z, false));
}

void NAVnode::addEarthCentredWaypointAtIndex(const std::shared_ptr<AddEarthCentredWaypointAtIndex::Request> request, 
                                                std::shared_ptr<AddEarthCentredWaypointAtIndex::Response> response){
    if (!ensureHome()) return;   // was a silent setHome(currentPosition), even with no fix
    routes[routeToEdit].add(new Waypoint(request->x, request->y, request->z, false), request->index);
}

void NAVnode::addEarthCentredObstacle(const std::shared_ptr<AddEarthCentredObstacle::Request> request, 
                                        std::shared_ptr<AddEarthCentredObstacle::Response> response){
    (void)response;

    if (!ensureHome()) return;   // was a silent setHome(currentPosition), even with no fix

    // check if the array of obstacles is full
    if (obstacleCount == obstacleLimit){
        // increase make a larger array to store obstacles
        int newLimit = obstacleLimit + 10;
        Obstacle** biggerArray = new Obstacle*[newLimit];

        // copy over existing obstacles (by iterating through existing array)
        for(int i = 0; i < obstacleCount; i++){
            biggerArray[i] = obstacles[i];
        }
        for(int i = obstacleCount; i < newLimit; i++){
            biggerArray[i] = nullptr;
        }

        // clean up memory and rename our new array (by reusing the old pointer);
        delete[] obstacles;
        obstacles = biggerArray;
        obstacleLimit = newLimit;
    }

    // add new obstacle. post incrementing to handle indexing differences
    if (request->radius <= 0)
        obstacles[obstacleCount++] = new Obstacle(request->x, request->y, request->z, false);
    else
        obstacles[obstacleCount++] = new Obstacle(request->x, request->y, request->z, request->radius, false);

    checkWaypointCollisions();
    checkTrackCollisions();
}

void NAVnode::removeLocal(const std::shared_ptr<RemoveLocal::Request> request, std::shared_ptr<RemoveLocal::Response> response){
    if(routes[routeToEdit].removePoint(request->x, request-> y))
        response->output = "Waypoint removed successfully";
    else if(removeLocalObstacle_helper(request->x, request->y))
        response->output = "Obstacle removed successfully";
    else
        response->output = "Point not found";
}

void NAVnode::removeLocalWaypoint(const std::shared_ptr<RemoveLocal::Request> request, std::shared_ptr<RemoveLocal::Response> response){
    if(routes[routeToEdit].removePoint(request->x, request-> y))
        response->output = "Waypoint removed successfully";
    else
        response->output = "Waypoint not in Route";
}

void NAVnode::removeLocalObstacle(const std::shared_ptr<RemoveLocal::Request> request, std::shared_ptr<RemoveLocal::Response> response){
    if(removeLocalObstacle_helper(request->x, request->y))
        response->output = "Obstacle removed successfully";
    else
        response->output = "Obstacle not found";
}

void NAVnode::removeGeodetic(const std::shared_ptr<RemoveGeodetic::Request> request, std::shared_ptr<RemoveGeodetic::Response> response){
    if(routes[routeToEdit].removeGeodeticPoint(request->latitude, request-> longitude))
        response->output = "Waypoint removed successfully";
    else if(removeGeodeticObstacle_helper(request->latitude, request-> longitude))
        response->output = "Obstacle removed successfully";
    else
        response->output = "Point not found";
}

void NAVnode::removeGeodeticWaypoint(const std::shared_ptr<RemoveGeodetic::Request> request, std::shared_ptr<RemoveGeodetic::Response> response){
    if(routes[routeToEdit].removeGeodeticPoint(request->latitude, request-> longitude))
        response->output = "Waypoint removed successfully";
    else
        response->output = "Waypoint not in Route";
}

void NAVnode::removeGeodeticObstacle(const std::shared_ptr<RemoveGeodetic::Request> request, std::shared_ptr<RemoveGeodetic::Response> response){
    if(removeGeodeticObstacle_helper(request->latitude, request-> longitude))
        response->output = "Obstacle removed successfully";
    else
        response->output = "Obstacle not found";
}

void NAVnode::removeEarthCentred(const std::shared_ptr<RemoveEarthCentred::Request> request, std::shared_ptr<RemoveEarthCentred::Response> response){
    if(routes[routeToEdit].removeEarthCentredPoint(request->x, request->y, request->z))
        response->output = "Waypoint removed successfully";
    else if(removeEarthCentredObstacle_helper(request->x, request->y, request->z))
        response->output = "Obstacle removed successfully";
    else
        response->output = "Point not found";
}

void NAVnode::removeEarthCentredWaypoint(const std::shared_ptr<RemoveEarthCentred::Request> request, std::shared_ptr<RemoveEarthCentred::Response> response){
    if(routes[routeToEdit].removeEarthCentredPoint(request->x, request->y, request->z))
        response->output = "Waypoint removed successfully";
    else
        response->output = "Waypoint not in Route";
}

void NAVnode::removeEarthCentredObstacle(const std::shared_ptr<RemoveEarthCentred::Request> request, std::shared_ptr<RemoveEarthCentred::Response> response){
    if(removeEarthCentredObstacle_helper(request->x, request->y, request->z))
        response->output = "Obstacle removed successfully";
    else
        response->output = "Obstacle not found";
}

void NAVnode::removeLastWaypoint(const std::shared_ptr<RemoveLast::Request> request, std::shared_ptr<RemoveLast::Response> response){
    ListNode* temp = routes[routeToEdit].getHead();   // was routeToFollow: checked one route, removed from another

    if(temp == nullptr)
        response->output = "List is already empty";
    else {
        routes[routeToEdit].removeLast();
        response->output = "Last Waypoint Successfull Removed";
    }
}

void NAVnode::removeLastObstacle(const std::shared_ptr<RemoveLast::Request> request, std::shared_ptr<RemoveLast::Response> response){
    if(obstacleCount == 0)
        response->output = "No Obstacles currently in the system";
    else {
        delete obstacles[--obstacleCount];
        response->output = "Last Obstacle successfully removed";
    }
}

void NAVnode::sendToGNSS(const std::shared_ptr<SendToGNSS::Request> request, std::shared_ptr<SendToGNSS::Response> response){
    (void)response;
    if (GNSS != nullptr) {
        GNSS->write(request->outgoing);
    }
}

void NAVnode::readFromGNSS(const std::shared_ptr<ReadFromGNSS::Request> request, std::shared_ptr<ReadFromGNSS::Response> response){
    (void)request;
    if (GNSS != nullptr) {
        response->incoming = GNSS->read();
        cout << response->incoming << '\n';
    } else {
        response->incoming = "";
    }
}


    // action callback functions
rclcpp_action::GoalResponse NAVnode::followRoute_Goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NAVaction::Goal> goal){
    cout << "Received goal6 request" << '\n';
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // accept all new goals without question
}

rclcpp_action::CancelResponse NAVnode::followRoute_Cancel(const std::shared_ptr<NAVGoalHandle> goal_handle){
    cout<< "Received cancel request" << '\n';
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT; // blindly accepts all cancel requests
}

void NAVnode::followRoute_Accepted(const std::shared_ptr<NAVGoalHandle> goal_handle){
    // The goal used to run in a detached thread that read currentPosition (deleted and replaced every
    // 200 ms by the timer) and route nodes (deletable by the remove services) with no locking. It is now
    // tracked by mainTimer() on the executor thread, using the same target and arrival logic as the node.
    // NOTE: the action reports progress only; nothing here drives the rover.
    if (activeGoal && activeGoal->is_active()) {
        auto result = std::make_shared<NAVaction::Result>();
        result->success = "Superseded by a new FollowRoute goal";
        activeGoal->abort(result);
    }
    activeGoal = goal_handle;
    goalFeedbackTicks = 0;
    routeFinished = false;
    nextWaypoint = nullptr;   // validateTarget() starts the followed route from its first waypoint
}

void NAVnode::updateFollowRouteGoal(){
    if (!activeGoal) return;
    if (!activeGoal->is_active()) { activeGoal.reset(); return; }

    auto result = std::make_shared<NAVaction::Result>();
    if (activeGoal->is_canceling()) {
        result->success = "Navigation Prematurely Terminated";
        activeGoal->canceled(result);
        activeGoal.reset();
        return;
    }
    if (routes[routeToFollow].getHead() == nullptr) {
        result->success = "Route is empty. No waypoints to follow.";
        activeGoal->succeed(result);
        activeGoal.reset();
        return;
    }
    if (routeFinished) {
        result->success = "great success";
        activeGoal->succeed(result);
        activeGoal.reset();
        return;
    }

    if (++goalFeedbackTicks % 5 != 0) return;   // feedback at 1 Hz (timer runs at 5 Hz)
    auto feedback = std::make_shared<NAVaction::Feedback>();
    std::stringstream ss;
    if (nextWaypoint != nullptr && nextWaypoint->point != nullptr && currentPosition != nullptr) {
        double dx = nextWaypoint->point->getX() - currentPosition->getX();
        double dy = nextWaypoint->point->getY() - currentPosition->getY();
        ss << "Navigating to waypoint (" << nextWaypoint->point->getX() << ", " << nextWaypoint->point->getY()
           << ") | Current: (" << currentPosition->getX() << ", " << currentPosition->getY()
           << ") | Dist: " << std::fixed << std::setprecision(2) << std::sqrt(dx * dx + dy * dy) << " m";
    } else {
        ss << "Waiting for a target waypoint";
    }
    feedback->progress = ss.str();
    activeGoal->publish_feedback(feedback);
}

//////////////////////////////////////////////////////////////////////////
// TODO
//////////////////////////////////////////////////////////////////////////

/*
    - finish logic behind the Navsystem class (optonal as the circumnaviagation code is no longer used)
                -> add checks for track craling method to ensure any new waypoints created are not colliding with obstacles

    - update documentation
*/

//////////////////////////////////////////////////////////////////////////
// END TODO
//////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NAVnode>());
    rclcpp::shutdown();

    return 0;
}