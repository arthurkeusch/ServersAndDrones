#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <canvas.h>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <trianglemesh.h>

#include <QDebug>
#include <queue>
#include <limits>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // load initial simple case
    loadJson("../../../json/simple.json");
}

MainWindow::~MainWindow()
{
    delete ui;
}

bool MainWindow::loadJson(const QString& title) {
    QFile file(title);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Impossible d'ouvrir le fichier:" << title;
        return false;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(data, &error);
    if (error.error != QJsonParseError::NoError) {
        qWarning() << "Erreur JSON:" << error.errorString();
        return false;
    }
    if (!doc.isObject()) {
        qWarning() << "Le document JSON n'est pas un objet.";
        return false;
    }

    QJsonObject root = doc.object();

    // --- Window ---
    if (root.contains("window") && root["window"].isObject()) {
        QJsonObject win = root["window"].toObject();

        auto origin = win.value("origine").toString().split(",");
        auto size   = win.value("size").toString().split(",");
        QPoint wOrigin={origin[0].toInt(),origin[1].toInt()};
        QSize wSize={size[0].toInt(),size[1].toInt()};
        qDebug() << "Window.origine =" << wOrigin;
        qDebug() << "Window.size    =" << wSize;
        ui->canvas->setWindow(wOrigin,wSize);
    }

    // --- Servers ---
    if (root.contains("servers") && root["servers"].isArray()) {
        int num=0;
        QJsonArray arr = root["servers"].toArray();
        for (const QJsonValue &v : arr) {
            if (!v.isObject()) continue;
            QJsonObject obj = v.toObject();
            Server s;
            s.name = obj.value("name").toString();
            QString pos = obj.value("position").toString();
            auto parts = pos.split(',');
            if (parts.size() == 2)
                s.position = QPoint(parts[0].toInt(), parts[1].toInt());
            s.color = QColor(obj.value("color").toString());
            s.id=num++;
            ui->canvas->servers.append(s);
            qDebug() << "Server:" << s.id << "," << s.name << s.position << s.color;
        }
    }

    // --- Drones ---
    if (root.contains("drones") && root["drones"].isArray()) {
        QJsonArray arr = root["drones"].toArray();

        for (const QJsonValue &v : arr) {
            if (!v.isObject()) continue;
            QJsonObject obj = v.toObject();
           Drone d;
            d.name = obj.value("name").toString();
            QString pos = obj.value("position").toString();
            auto parts = pos.split(',');
            if (parts.size() == 2)
                d.position = Vector2D(parts[0].toInt(), parts[1].toInt());
            QString name = obj.value("target").toString();
            // search name in server list
            d.target=nullptr;
            auto it=ui->canvas->servers.begin();
            while (it!=ui->canvas->servers.end() && it->name!=name) it++;
            if (it!=ui->canvas->servers.end()) {
                d.target=&(*it);
                qDebug() << "Drone:" << d.name << "(" << d.position.x << "," << d.position.y << ") →" << d.target->name;
            } else {
                qDebug() << "error in JsonFile: bad destination name: " << name;
            }
            ui->canvas->drones.append(d);
        }
    }

    createVoronoiMap();
    createServersLinks();
    fillDistanceArray();
    return true;
}

void MainWindow::createVoronoiMap() {
    TriangleMesh mesh(ui->canvas->servers);
    mesh.setBox(ui->canvas->getOrigin(),ui->canvas->getSize());

    auto triangles = mesh.getTriangles();
    auto m_servor = ui->canvas->servers.begin();
    QVector<const Triangle*> tabTri;
    while (m_servor!=ui->canvas->servers.end()) {
        // for all vertices of the mesh
        const Vector2D vert((*m_servor).position.x(),(*m_servor).position.y());
        auto mt_it = triangles->begin();
        tabTri.clear(); // tabTri: list of triangles containing m_vert
        while (mt_it!=triangles->end()) {
            if ((*mt_it).hasVertex(vert)) {
                tabTri.push_back(&(*mt_it));
            }
            mt_it++;
        }
        // find left border
        auto first = tabTri.begin();
        auto tt_it = tabTri.begin();
        bool found=false;
        while (tt_it!=tabTri.end() && !found) {
            auto comp_it = tabTri.begin();
            while (comp_it!=tabTri.end() && (*tt_it)->getNextVertex(vert)!=(*comp_it)->getPrevVertex(vert)) {
                comp_it++;
            }
            if (comp_it==tabTri.end()) {
                first=tt_it;
                found=true;
            }
            tt_it++;
        }
        // create polygon

        //poly->setColor((*m_servor)->color);
        tt_it=first;
        if (found && mesh.isInWindow((*tt_it)->getCenter().x,(*tt_it)->getCenter().y)) { // add a point for the left border
            Vector2D V = (*first)->nextEdgeNormal(vert);
            float k;
            if (V.x > 0) { // (circumCenter+k V).x=width
                k = (mesh.getWindowXmax() - (*first)->getCenter().x) / float(V.x);
            } else {
                k = (mesh.getWindowXmin()-(*first)->getCenter().x) / float(V.x);
            }
            if (V.y > 0) { // (circumCenter+k V).y=height
                k = fmin(k, (mesh.getWindowYmax() - (*first)->getCenter().y) / float(V.y));
            } else {
                k = fmin(k, (mesh.getWindowYmin()-(*first)->getCenter().y) / float(V.y));
            }
            m_servor->area.addVertex(Vector2D((*first)->getCenter() + k * V));
            Vector2D pt = (*first)->getCenter() + k * V;
        }
        auto comp_it = first;
        do {
            m_servor->area.addVertex((*tt_it)->getCenter());
            // search triangle on right of tt_it
            comp_it = tabTri.begin();
            while (comp_it!=tabTri.end() && (*tt_it)->getPrevVertex(vert)!=(*comp_it)->getNextVertex(vert)) {
                comp_it++;
            }
            if (comp_it!=tabTri.end()) tt_it = comp_it;
        } while (tt_it!=first && comp_it!=tabTri.end());
        if (found && mesh.isInWindow((*tt_it)->getCenter())) { // add a point for the right border
            Vector2D V = (*tt_it)->previousEdgeNormal(vert);
            float k;
            if (V.x > 0) { // (circumCenter+k V).x=width
                k = (mesh.getWindowXmax() - (*tt_it)->getCenter().x) / float(V.x);
            } else {
                k = (mesh.getWindowXmin()-(*tt_it)->getCenter().x) / float(V.x);
            }
            if (V.y > 0) { // (circumCenter+k V).y=height
                k = fmin(k, (mesh.getWindowYmax() - (*tt_it)->getCenter().y) / float(V.y));
            } else {
                k = fmin(k, (mesh.getWindowYmin()-(*tt_it)->getCenter().y) / float(V.y));
            }
            m_servor->area.addVertex(Vector2D((*tt_it)->getCenter() + k * V));
            Vector2D pt = (*tt_it)->getCenter() + k * V;
        }
        qDebug() << m_servor->name;
        m_servor->area.clip(mesh.getWindowXmin(),mesh.getWindowYmin(),mesh.getWindowXmax(),mesh.getWindowYmax());
        m_servor->area.triangulate();

        m_servor++;
    }
}

void MainWindow::createServersLinks() {
    // Number of servers
    const int n = ui->canvas->servers.size();

    // Clear existing links
    ui->canvas->links.clear();
    for (auto &s : ui->canvas->servers) s.links.clear();

    // Iterate over all server pairs (i < j)
    for (int i = 0; i < n; ++i) {
        Server &si = ui->canvas->servers[i];
        const int ni = si.area.nbVertices(); // Number of edges of server i area

        for (int j = i + 1; j < n; ++j) {
            Server &sj = ui->canvas->servers[j];
            const int nj = sj.area.nbVertices(); // Number of edges of server j area

            bool found = false;
            QPair<Vector2D, Vector2D> commonEdge;

            // Search for a common edge between the two polygons
            for (int ei = 0; ei < ni && !found; ++ei) {
                const auto e1 = si.area.getEdge(ei);
                for (int ej = 0; ej < nj; ++ej) {
                    const auto e2 = sj.area.getEdge(ej);

                    // Check if edges match (both orientations)
                    if (
                        (e1.first == e2.first && e1.second == e2.second) ||
                        (e1.first == e2.second && e1.second == e2.first)
                        ) {
                        commonEdge = e1;
                        found = true;
                        break;
                    }
                }
            }

            // Create a link if a shared edge has been found
            if (found) {
                Link *l = new Link(&si, &sj, commonEdge);
                ui->canvas->links.push_back(l);
                si.links.push_back(l);
                sj.links.push_back(l);
            }
        }
    }
}

void MainWindow::fillDistanceArray() {
    // define a nServers x nServers array
    int nServers = ui->canvas->servers.size();
    distanceArray.resize(nServers);
    for (int i=0; i<nServers; i++) {
        distanceArray[i].resize(nServers);
    }
    // init Servers distanceArray
    for (auto &s:ui->canvas->servers) {
        s.bestDistance.resize(nServers);
        for (int i=0; i<nServers; i++) {
            s.bestDistance[i]={nullptr,0};
        }
    }

    // Helper: returns the opposite server of a link
    auto otherServer = [](Link* l, Server* from) -> Server* {
        if (!l || !from) return nullptr;
        if (l->getNode1() == from) return l->getNode2();
        if (l->getNode2() == from) return l->getNode1();
        return nullptr;
    };

    const qreal INF = std::numeric_limits<qreal>::infinity();

    // Run Dijkstra from each server
    for (auto &src : ui->canvas->servers) {
        const int srcId = src.id;

        QVector<qreal> dist(nServers, INF); // Distance from src
        QVector<int> prev(nServers, -1); // Previous node
        QVector<Link*> prevLink(nServers, nullptr); // Link used to reach node

        // Priority queue item
        struct Item {
            qreal d;
            int v;
            bool operator>(const Item& o) const { return d > o.d; }
        };

        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
        dist[srcId] = 0;
        pq.push({0, srcId});

        // Dijkstra main loop
        while (!pq.empty()) {
            Item cur = pq.top();
            pq.pop();

            // Ignore outdated entries
            if (cur.d != dist[cur.v]) continue;

            Server* u = &ui->canvas->servers[cur.v];

            // Check if going through this link gives a shorter path
            for (Link* l : u->links) {
                Server* vS = otherServer(l, u);
                if (!vS) continue;
                int v = vS->id;

                qreal nd = dist[cur.v] + l->getDistance();
                if (nd < dist[v]) {
                    dist[v] = nd;
                    prev[v] = cur.v;
                    prevLink[v] = l;
                    pq.push({nd, v});
                }
            }
        }

        // Fill distance table and first-hop information
        for (int dstId = 0; dstId < nServers; ++dstId) {
            distanceArray[srcId][dstId] = (dist[dstId] == INF ? -1.0f : (float)dist[dstId]);

            // Source to itself
            if (dstId == srcId) {
                src.bestDistance[dstId] = {nullptr, 0};
                continue;
            }

            // Unreachable destination
            if (dist[dstId] == INF) {
                src.bestDistance[dstId] = {nullptr, INF};
                continue;
            }

            // Backtrack to find first hop
            int cur = dstId;
            while (prev[cur] != -1 && prev[cur] != srcId) cur = prev[cur];

            Link* firstHop = (prev[cur] == srcId ? prevLink[cur] : nullptr);
            src.bestDistance[dstId] = {firstHop, dist[dstId]};
        }
    }

    // Print distance table (debug output)
    QString header = "From/To |";
    for (int j = 0; j < nServers; ++j) header += QString(" %1 |").arg(j, 6);
    qDebug().noquote() << header;

    QString sep = "--------|";
    for (int j = 0; j < nServers; ++j) sep += "--------|";
    qDebug().noquote() << sep;

    for (int i = 0; i < nServers; ++i) {
        QString row = QString("%1      |").arg(i, 2);
        for (int j = 0; j < nServers; ++j) {
            qreal d = ui->canvas->servers[i].bestDistance[j].second;
            if (i == j) row += QString(" %1 |").arg("0", 6);
            else if (!std::isfinite((double)d)) row += QString(" %1 |").arg("INF", 6);
            else row += QString(" %1 |").arg(QString::number(d, 'f', 1), 6);
        }
        qDebug().noquote() << row;
    }

    // Initialize each drone by assigning it to the server of the area it is overflying
    for (auto &d : ui->canvas->drones) {
        Server* s = d.overflownArea(ui->canvas->servers);
        if (s) {
            d.destination = Vector2D(s->position.x(), s->position.y());
        } else {
            d.destination = d.position;
        }
    }
}

void MainWindow::update() {
    static int last=elapsedTimer.elapsed();
    int current=elapsedTimer.elapsed();
    int dt=current-last;
    // update positions of drones
    for (auto &drone:ui->canvas->drones) {
        drone.move(dt/1000.0);
    }
    ui->canvas->repaint();
}

void MainWindow::on_actionShow_graph_triggered(bool checked) {
    ui->canvas->showGraph=checked;
    ui->canvas->repaint();
}


void MainWindow::on_actionMove_drones_triggered() {
    timer = new QTimer(this);
    timer->setInterval(100);
    connect(timer,SIGNAL(timeout()),this,SLOT(update()));
    timer->start();

    elapsedTimer.start();
}


void MainWindow::on_actionQuit_triggered() {
    QApplication::quit();
}


void MainWindow::on_actionCredits_triggered() {
    QMessageBox::information(this,"About DroneAndRooms program",
                             "My tiny project.\nCredit Benoît Piranda");
}


void MainWindow::on_actionLoad_triggered() {
    auto fileName = QFileDialog::getOpenFileName(this,tr("Open json description file"), "../../data", tr("JSON Files (*.json)"));
    if (!fileName.isEmpty()) {
        ui->canvas->clear();
        loadJson(fileName);
        ui->canvas->update();
    }
}

