#include "serveranddrone.h"
#include <QDebug>

Link::Link(Server *n1,Server *n2,const QPair<Vector2D,Vector2D> &edge):
    node1(n1),node2(n2) {
    // computation of the length of the link
    Vector2D center=0.5*(edge.first+edge.second);
    distance = (center-Vector2D(n1->position.x(),n1->position.y())).length();
    distance += (center-Vector2D(n2->position.x(),n2->position.y())).length();
    edgeCenter=QPointF(center.x,center.y);
}

void Link::draw(QPainter &painter) {
    painter.drawLine(node1->position,edgeCenter);
    painter.drawLine(node2->position,edgeCenter);
}

void Drone::move(qreal dt) {

    // Check if two positions are considered close enough
    auto nearPos = [&](const Vector2D& a, const Vector2D& b) -> bool {
        return (a - b).length() <= minDistance;
    };

    // Cannot move without a target or a current server
    if (!target) return;
    if (!connectedTo) return;

    Vector2D connectedPos(connectedTo->position.x(), connectedTo->position.y());
    Vector2D targetPos(target->position.x(), target->position.y());

    // Final stop when reaching the target server
    if (connectedTo == target && nearPos(position, targetPos)) {
        position = targetPos;
        destination = targetPos;
        speed = Vector2D(0,0);
        return;
    }

    // Check if the current destination has been reached
    if (nearPos(position, destination)) {
        position = destination;
        speed = Vector2D(0,0);

        connectedPos = Vector2D(connectedTo->position.x(), connectedTo->position.y());

        // If the drone is at its current server
        if (nearPos(position, connectedPos)) {

            // Follow the shortest path to the target server
            Link* next = connectedTo->bestDistance[target->id].first;
            if (!next) {
                destination = connectedPos;
            } else {
                destination = next->getEdgeCenter();
            }
        } else {

            // Check if the drone is crossing an edge center
            Link* crossed = nullptr;
            for (Link* l : connectedTo->links) {
                if (nearPos(position, l->getEdgeCenter())) {
                    crossed = l;
                    break;
                }
            }

            // If no edge was crossed, return to current server
            if (!crossed) {
                destination = connectedPos;
            } else {
                // Switch to the adjacent server area
                Server* opp = (crossed->getNode1() == connectedTo) ? crossed->getNode2() : crossed->getNode1();
                if (!opp) {
                    destination = connectedPos;
                } else {
                    connectedTo = opp;
                    destination = Vector2D(opp->position.x(), opp->position.y());
                }
            }
        }
    }

    // Compute direction and distance to destination
    Vector2D dir = destination - position;
    double d = dir.length();

    // Slow down when approaching the destination
    if (d < slowDownDistance) {
        speed = (d * speedLocal / slowDownDistance) * dir;
        if (speed.length() > speedMax) {
            speed.normalize();
            speed *= speedMax;
        }
    } else {
        // Accelerate toward destination
        speed += (accelation * dt / d) * dir;
        if (speed.length() > speedMax) {
            speed.normalize();
            speed *= speedMax;
        }
    }

    // Update position
    position += (dt * speed);

    // Update orientation (azimuth) from velocity direction
    double sl = speed.length();
    Vector2D Vn = (1.0 / sl) * speed;

    if (Vn.y == 0) {
        azimut = (Vn.x > 0) ? -90.0 : 90.0;
    } else if (Vn.y > 0) {
        azimut = 180.0 - 180.0 * atan(Vn.x / Vn.y) / M_PI;
    } else {
        azimut = -180.0 * atan(Vn.x / Vn.y) / M_PI;
    }
}


Server* Drone::overflownArea(QList<Server>& list) {
    auto it=list.begin();
    while (it!=list.end() && !it->area.contains(position)) {
        it++;
    }
    connectedTo= it!=list.end()?&(*it):nullptr;
    return connectedTo;
}
